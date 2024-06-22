// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessModel;
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
        public RunVideoCombDrone(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone) : base(parent, config, dataStore, drone)
        {
            ProcessAll = ProcessFactory.NewCombProcessModel(RunConfig.ProcessConfig, drone.InputVideo, drone.GroundData, drone);
        }


        // The Comb model 
        public CombProcessAll? CombProcess { get { return ProcessAll as CombProcessAll; } }


        // The input video file name to process.
        // User may provide the optical video name in Config.InputFileName
        // We base all output on the companion thermal video.
        public override string InputVideoFileName()
        {
            if ((Drone != null) && Drone.HasInputVideo)
                return Drone.InputVideo.FileName;

            return RunConfig.InputFileName;
        }


        // Load model data from the previous run (if any).
        public override void LoadDataStore()
        {
            try
            {
                BlockLoad datareader1 = new(DataStore);
                datareader1.BlockObjects(CombProcess, Drone);

                CombLoad datareader2 = new(DataStore);
                datareader2.CombFeatures(CombProcess);

                datareader2.CombObjects(CombProcess);
                var objectListSettings = datareader2.ObjectListSettings();
                if (objectListSettings != null)
                    CombProcess.CombObjs.CombObjList.LoadSettings(objectListSettings);

                datareader2.CombSpans(CombProcess, Drone);

                // Reset the FlightStep.FixAltM values from the CombSpan data
                CombProcess.CombSpans.SetFixAltMAfterLoad(Drone.InputVideo, Drone);

                // Link each object to its features
                foreach (var feature in CombProcess.CombFeatures)
                    if (feature.Value.ObjectId >= 0)
                        CombProcess.CombObjs.SetLinksAfterLoad(feature.Value);
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoCombDrone.LoadDataStore", ex);
            }
        }


        // Describe the objects found
        public override string DescribeSignificantObjects()
        {
            return CombProcess.CombObjs.DescribeSignificantObjects();
        }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(int prevLegId, int currLegId)
        {
            CombProcess.ProcessFlightLegStartAndEnd(prevLegId, currLegId);

            // Save memory (if compatible with Config settings) by deleting pixel data
            CombProcess.DeleteFeaturePixelsForObjects();
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var currBlock = CombProcess.AddCombBlock(this);

                // If camera is too near the horizon, skip this frame.
                if ((currBlock != null) && (currBlock.FlightStep != null) &&
                    !Drone.FlightStepInRunScope(currBlock.FlightStep))
                    // Don't create features. Don't update objects.
                    return currBlock;

                // Process the frame using "Image" Comb class, creating features.
                // This "image" class has no "frame to frame" logic, so features are "one image" features.
                CombFeatureList featuresInBlock = CombImage.Process(RunConfig, CombProcess, currBlock, CurrInputVideoFrame);

                foreach (var feature in featuresInBlock)
                {
                    feature.Value.CalculateSettings_LocationM_FlatGround(null);
                    feature.Value.CalculateSettings_LocationM_HeightM_LineofSight();
                }

                // Unless legs are not used, we only do comb processing during "legs". 
                if ( (!Drone.UseFlightLegs) || (PSM.CurrRunLegId > 0))
                    // Process the features, by preference associated them with existing CombObjects, else creating new objects.
                    CombProcess.ProcessBlockForObjects(this, featuresInBlock);
                else
                    // Outside legs, we store the features so we can draw them on the video frame later.
                    CombProcess.ProcessBlockForFeatures(featuresInBlock);

                return currBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoCombDrone.AddBlockAndProcessInputVideoFrame", ex);
            }
        }


        // Called after block processing has finished
        public override void EndBlockProcessing()
        { 
            // If we have been tracking some significant objects, create a CombSpan for them
            CombProcess.Process_CombSpan_Create();
        }


        // Return the data to show in the ObjectGrid in the Main Form
        public override List<object[]> GetObjectGridData(bool mainForm)
        {
            var combModel = CombProcess;
            return combModel.CombObjs.GetObjectGridData(this, mainForm, CategoryAll.ObjectCategories, RunConfig.ProcessConfig.FocusObjectId);
        }


        // Do any final activity at the end processing of video
        public override void EndRunning()
        {
            // Calculate object summary data
            CombProcess?.CombObjs?.CombObjList.CalculateSettings(this, RunConfig.ProcessConfig.FocusObjectId);

            CombSave dataWriter = new(Drone, DataStore);
            dataWriter.Comb(RunConfig, GetEffort(), GetSettings(), this, CombProcess, true);

            base.EndRunning();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            CombSave datawriter = new(Drone, DataStore);
            datawriter.Comb(RunConfig, GetEffort(), GetSettings(), this, CombProcess, false);
            DataStore.Close();
        }
    }
}