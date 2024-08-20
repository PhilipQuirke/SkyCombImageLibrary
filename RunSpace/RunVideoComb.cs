// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV.Structure;
using Emgu.CV;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;


// Continuation of RunVideo.cs, contains both Skycomb-specific runners
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
    public class RunVideoCombDrone : RunVideoPersist
    {
        public RunVideoCombDrone(RunUserInterface parent, RunConfig config, DroneDataStore dataStore, Drone drone) 
            : base(parent, config, dataStore, drone,
                  ProcessFactory.NewCombProcess(drone.GroundData, drone.InputVideo, drone, config.ProcessConfig ))
        {
        }


        // The Comb model 
        public CombProcess CombProcess { get { return ProcessAll as CombProcess; } }


        // The input video file name to process.
        // User may provide the optical video name in Config.InputFileName
        // We base all output on the companion thermal video.
        public override string InputVideoFileName()
        {
            if ((Drone != null) && Drone.HasInputVideo)
                return Drone.InputVideo.FileName;

            return RunConfig.InputFileName;
        }


        // Describe the objects found
        public override string DescribeSignificantObjects()
        {
            return ProcessAll.ProcessObjects.DescribeSignificantObjects();
        }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(ProcessScope scope, int prevLegId, int currLegId)
        {
            ProcessAll.ProcessFlightLegStartAndEnd(scope, prevLegId, currLegId);

            // Save memory (if compatible with Config settings) by deleting pixel data
            CombProcess.DeleteFeaturePixelsForObjects();
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var currBlock = ProcessAll.AddBlock(this);

                // If camera is too near the horizon, skip this frame.
                if ((currBlock != null) && (currBlock.FlightStep != null) &&
                    !Drone.FlightStepInRunScope(currBlock.FlightStep))
                    // Don't create features. Don't update objects.
                    return currBlock;

                Image<Bgr, byte> imgInput = CurrInputImage.Clone();
                DrawImage.Smooth(RunConfig.ProcessConfig, ref imgInput);

                Image<Gray, byte> imgThreshold = DrawImage.ToGrayScale(imgInput);
                DrawImage.Threshold(RunConfig.ProcessConfig, ref imgThreshold);

                ProcessFeatureList featuresInBlock = ProcessFactory.NewProcessFeatureList(CombProcess.ProcessConfig);
                CombFeatureLogic.CreateFeaturesFromImage(
                    CombProcess, featuresInBlock, currBlock, 
                    CurrInputImage, imgThreshold ); // read-only  images

                foreach (var feature in featuresInBlock)
                {
                    feature.Value.CalculateSettings_LocationM_FlatGround(null);
                    feature.Value.CalculateSettings_LocationM_HeightM_LineofSight(ProcessAll.GroundData);
                }

                // Unless legs are not used, we only do comb processing during "legs". 
                if ( (!Drone.UseFlightLegs) || (PSM.CurrRunLegId > 0))
                    // Process the features, by preference associated them with existing CombObjects, else creating new objects.
                    CombProcess.ProcessBlockForObjects(this, featuresInBlock);
                else
                    // Outside legs, we store the features so we can draw them on the video frame later.
                    CombProcess.ProcessBlockForFeatures(featuresInBlock);

                imgInput.Dispose();
                imgThreshold.Dispose();

                return currBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoCombDrone.AddBlockAndProcessInputVideoFrame", ex);
            }
        }


        // Return the data to show in the ObjectGrid in the Main Form
        public override List<object[]> GetObjectGridData(bool mainForm)
        {
            return ProcessAll.ProcessObjects.GetObjectGridData(this, RunConfig.ProcessConfig, mainForm, CategoryAll.ObjectCategories);
        }


        // Do any final activity at the end processing of video
        public override void RunEnd()
        {
            // Calculate object summary data
            ProcessAll?.ProcessObjects?.CalculateSettings(this);

            StandardSave dataWriter = new(Drone, DataStore);
            dataWriter.ProcessAll(DataStore, RunConfig, GetEffort(), GetSettings(), this, CombProcess, true);

            base.RunEnd();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            StandardSave datawriter = new(Drone, DataStore);
            datawriter.ProcessAll(DataStore, RunConfig, GetEffort(), GetSettings(), this, CombProcess, false);
            DataStore.FreeResources();
        }
    }
}