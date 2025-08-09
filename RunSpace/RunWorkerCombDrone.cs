// Copyright SkyComb Limited 2025. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;


// Continuation of RunWorker.cs, contains both Skycomb-specific runners
namespace SkyCombImage.RunSpace
{
    // The RunCombDual takes as input 1 to 4 files. For example, the DJI M2E Dual drone:
    //      DJI_0049.mp4 - the optical video. OPTIONAL
    //      DJI_0049.srt - a DJI-specific SRT file with basic data and extra optical-camera settings. OPTIONAL
    //      DJI_0050.mp4 - the thermal (aka IR) video. MANDATORY
    //      DJI_0050.srt - a DJI-specific SRT file with basic data (location, orientation, etc). OPTIONAL
    // 
    // For every frame in the thermal video:
    // - "hot" pixels with an intensity above a threshold level are collected.
    // - the Comb-specific image-level CombFeature.PixelNeighborSearch searches the hot pixels, finding clusters, above a minimum density and a minimum size, and generating SkyCombFeatures.
    // - the Comb-specific video-level CombObject.ClaimFeature compares the current and previous frame features to detect significant CombObjects
    //
    // The thermal flight data associated provides drone location, altitude, timestamp & speed information for each frame.
    public class RunWorkerCombDrone : RunWorkerPersist
    {
        public RunWorkerCombDrone(RunUserInterface runUI, RunConfig config, DroneDataStore dataStore, Drone drone)
            : base(runUI, config, dataStore, drone,
                  ProcessFactory.NewCombProcess(drone.GroundData, drone.InputVideo, drone, config.ProcessConfig, runUI))
        {
        }


        public CombProcess CombProcess { get { return ProcessAll as CombProcess; } }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(ProcessScope scope, int prevLegId, int currLegId)
        {
            ProcessAll.ProcessFlightLegStartAndEnd(scope, prevLegId, currLegId);

            ProcessAll.DeleteFeaturePixelsForObjects();
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputRunFrame()
        {
            try
            {
                var currBlock = ProcessAll.AddBlock(this);

                // If camera is too near the horizon, skip this frame.
                if ((currBlock != null) && (currBlock.FlightStep != null) &&
                    !Drone.FlightStepInRunScope(currBlock.FlightStep))
                    // Don't create features. Don't update objects.
                    return currBlock;

                Image<Bgr, byte> currInput = CurrInputImage.Clone();

                Image<Gray, byte> currThreshold = DrawImage.ToGrayScale(currInput);
                DrawImage.Threshold(RunConfig.ProcessConfig, ref currThreshold);

                ProcessFeatureList featuresInBlock = ProcessFactory.NewProcessFeatureList(CombProcess.ProcessConfig);
                if (RunConfig.RunProcess == RunProcessEnum.Threshold)
                    ThresholdFeatureLogic.CreateFeaturesFromImage(CombProcess, featuresInBlock, currBlock, CurrInputImage, currThreshold);
                else
                    CombFeatureLogic.CreateFeaturesFromImage(CombProcess, featuresInBlock, currBlock, CurrInputImage, currThreshold);

                foreach (var feature in featuresInBlock)
                    feature.Value.CalculateSettings_LocationM_HeightM_LOS(ProcessAll.GroundData);

                // If legs are used, we only do comb processing during "legs". 
                if ((!Drone.UseFlightLegs) || (PSM.CurrRunLegId > 0))
                    // Process the features, by preference associated them with existing CombObjects, else creating new objects.
                    CombProcess.ProcessBlockForObjects(this, featuresInBlock);
                else
                    // Outside legs, we store the features so we can draw them on the video frame later.
                    CombProcess.ProcessBlockForFeatures(featuresInBlock);

                currInput.Dispose();
                currThreshold.Dispose();

                return currBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunWorkerCombDrone.AddBlockAndProcessInputVideoFrame", ex);
            }
        }


        // Return the data to show in the ObjectGrid
        public override List<object[]> GetObjectGridData()
        {
            return ProcessAll.ProcessObjects.GetObjectGridData(this, RunConfig, CategoryAll.ObjectCategories);
        }


        // Do any final activity at the end processing of video
        public override void RunEnd()
        {
            // Calculate object summary data
            ProcessAll?.ProcessObjects?.CalculateSettings(this);

            StandardSave dataWriter = new(Drone, DataStore);
            dataWriter.ProcessAll(this, true);

            base.RunEnd();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            StandardSave datawriter = new(Drone, DataStore);
            datawriter.ProcessAll(this, false);
            DataStore.FreeResources();
        }
    }
}