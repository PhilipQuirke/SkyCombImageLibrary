// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;


// Namespace for processing of a video made up of multiple images (frames).
// Some classes contain code that persists information between frames
namespace SkyCombImage.RunSpace
{
    // YOLO (You only look once) V8 video processing.
    class RunVideoYolo : RunVideoPersist
    {
        public RunVideoYolo(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone) 
            : base(parent, config, dataStore, drone, ProcessFactory.NewYoloProcess(drone.GroundData, drone.InputVideo, drone, config.ProcessConfig, config.YoloDirectory))
        {
        }


        public YoloProcess YoloProcess { get { return (YoloProcess)ProcessAll; } }


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
                datareader1.BlockObjects(YoloProcess, Drone);

                YoloLoad datareader2 = new(DataStore);
                datareader2.YoloFeatures(YoloProcess);

                datareader2.ProcessObjects(YoloProcess);
                var objectListSettings = datareader2.ObjectListSettings();
                if (objectListSettings != null)
                    YoloProcess.ProcessObjects.LoadSettings(objectListSettings);

                // Link each object to its features
                foreach (var feature in YoloProcess.ProcessFeatures)
                    if (feature.Value.ObjectId >= 0)
                        YoloProcess.ProcessObjects.SetLinksAfterLoad(feature.Value);
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoYolo.LoadDataStore", ex);
            }
        }


        // Describe the objects found
        public override string DescribeSignificantObjects()
        {
            return "#Objects=" + YoloProcess.ProcessObjects.Count;
        }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(ProcessScope scope, int prevLegId, int currLegId)
        {
            YoloProcess.ProcessFlightLegStartAndEnd(scope, prevLegId, currLegId);
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = ProcessAll.AddBlock(this);

                var currGray = DrawImage.ToGrayScale(CurrInputVideoFrame);

                var result = YoloProcess.YoloDetect.Detect(currGray.ToBitmap());

                int numSig = YoloProcess.ProcessBlock(this, PrevGray, currGray, result);

                // Update the persisted gray frame 
                PrevGray = currGray.Clone();

                thisBlock.NumSig = numSig; 

                return thisBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoYolo.AddBlockAndProcessInputVideoFrame", ex);
            }
        }


        // Return the data to show in the ObjectGrid in the Main Form
        public override List<object[]> GetObjectGridData(bool mainForm)
        {
            return YoloProcess.ProcessObjects.GetObjectGridData(this, RunConfig.ProcessConfig, mainForm, CategoryAll.ObjectCategories, RunConfig.ProcessConfig.FocusObjectId);
        }


        // Do any final activity at the end processing of video
        public override void EndRunning()
        {
            YoloSave dataWriter = new(Drone, DataStore);
            dataWriter.Yolo(RunConfig, GetEffort(), GetSettings(), this, YoloProcess);

            base.EndRunning();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            YoloSave datawriter = new(Drone, DataStore);
            datawriter.Yolo(RunConfig, GetEffort(), GetSettings(), this, YoloProcess);
            DataStore.Close();
        }
    }
}