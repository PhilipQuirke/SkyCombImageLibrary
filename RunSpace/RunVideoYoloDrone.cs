// Copyright SkyComb Limited 2024. All rights reserved. 
using Compunet.YoloV8.Data;
using Emgu.CV;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImageLibrary.RunSpace;


// Namespace for processing of a video made up of multiple images (frames).
// Some classes contain code that persists information between frames
namespace SkyCombImage.RunSpace
{
    // YOLO (You only look once) V8 video processing.
    class RunVideoYoloDrone : RunVideoPersist
    {
        public RunVideoYoloDrone(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone) 
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

                var currGray = DrawImage.ToGrayScale(CurrInputImage);

                // Set pixels hotter than ThresholdValue to 1. Set other pixels to 0.
                var imgThreshold = currGray.Clone();
                DrawImage.Threshold(RunConfig.ProcessConfig, ref imgThreshold);

                DetectionResult result = YoloProcess.YoloDetect.Detect(currGray.ToBitmap());

                thisBlock.NumSig = YoloProcess.ProcessBlock(this, currGray, CurrInputImage, imgThreshold, result);

                currGray.Dispose();
                imgThreshold.Dispose();

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
            return YoloProcess.ProcessObjects.GetObjectGridData(this, RunConfig.ProcessConfig, mainForm, CategoryAll.ObjectCategories);
        }


        // Do any final activity at the end processing of video
        public override void EndRun()
        {
            StandardSave dataWriter = new(Drone, DataStore);
            dataWriter.ProcessAll(DataStore, RunConfig, GetEffort(), GetSettings(), this, YoloProcess, true);

            base.EndRun();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            StandardSave datawriter = new(Drone, DataStore);
            datawriter.ProcessAll(DataStore, RunConfig, GetEffort(), GetSettings(), this, YoloProcess, false);
            DataStore.Close();
        }
    }
}