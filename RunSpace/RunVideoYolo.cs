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

                datareader2.YoloObjects(YoloProcess);
                var objectListSettings = datareader2.ObjectListSettings();
                if (objectListSettings != null)
                    YoloProcess.YoloObjects.LoadSettings(objectListSettings);

                // Link each object to its features
                foreach (var feature in YoloProcess.ProcessFeatures)
                    if (feature.Value.ObjectId >= 0)
                        YoloProcess.YoloObjects.SetLinksAfterLoad(feature.Value);
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoYoloDrone.LoadDataStore", ex);
            }
        }



        // Add a block, transferring some flight data and process data into it
        private ProcessBlock AddBlock()
        {
            var newBlock = ProcessFactory.NewBlock(this);
            YoloProcess.Blocks.AddBlock(newBlock, this, Drone);

            if (PSM.CurrRunStepId <= 0)
                PSM.CurrRunStepId = newBlock.TardisId;

            return newBlock;
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
                var thisBlock = AddBlock();

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
                throw ThrowException("RunVideoYolo.ProcessInputVideoFrame", ex);
            }
        }


        // Describe the objects found
        public override string DescribeSignificantObjects()
        {
            return "#Objects=" + YoloProcess.YoloObjects.Count;
        }


        public override (Image<Bgr, byte>, Image<Bgr, byte>) DrawVideoFrames(ProcessBlockModel block, Image<Bgr, byte> InputFrame, Image<Bgr, byte> DisplayFrame)
        {
            var modifiedInputFrame = InputFrame.Clone();

            DrawImage.Palette(RunConfig.ImageConfig, ref modifiedInputFrame);

            DrawYolo.Draw(RunConfig.ImageConfig, YoloProcess, block.BlockId, ref modifiedInputFrame);

            return (modifiedInputFrame.Clone(), DisplayFrame.Clone());
        }


        // Return the data to show in the ObjectGrid in the Main Form
        public override List<object[]> GetObjectGridData(bool mainForm)
        {
            return YoloProcess.YoloObjects.GetObjectGridData(this, RunConfig.ProcessConfig, mainForm, CategoryAll.ObjectCategories, RunConfig.ProcessConfig.FocusObjectId);
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