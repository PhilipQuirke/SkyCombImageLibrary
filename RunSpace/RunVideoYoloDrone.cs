// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;
using YoloDotNet.Models;


// Namespace for processing of a video made up of multiple images (frames).
// Some classes contain code that persists information between frames
namespace SkyCombImage.RunSpace
{
    // YOLO (You only look once) V8 video processing.
    class RunVideoYoloDrone : RunVideoPersist
    {
        public RunVideoYoloDrone(RunUserInterface runUI, RunConfig config, DroneDataStore dataStore, Drone drone)
            : base(runUI, config, dataStore, drone, ProcessFactory.NewYoloProcess(drone.GroundData, drone.InputVideo, drone, config.ProcessConfig, runUI, config.YoloDirectory))
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


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(ProcessScope scope, int prevLegId, int currLegId)
        {
            ProcessAll.ProcessFlightLegStartAndEnd(scope, prevLegId, currLegId);
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = ProcessAll.AddBlock(this);
                int blockID = thisBlock.BlockId;

                List<ObjectDetection>? results = null;
                ProcessFeatureList featuresInBlock = new(RunConfig.ProcessConfig);

                {
                    var currGray = DrawImage.ToGrayScale(CurrInputImage);
                    var currBmp = currGray.ToBitmap();

                    DrawImage.Threshold(RunConfig.ProcessConfig, ref currGray);

                    results = YoloProcess.YoloDetectImage(currBmp, thisBlock);
                    if (results != null)
                        foreach (var result in results)
                        {
                            YoloFeature thisFeature = new(YoloProcess, blockID, result, ProcessModel.FeatureTypeEnum.Real);

                            // The Yolo bounding box is not tight around the hotspot. For Comb it is.
                            // Shrink PixelBox to a smaller bounding box tight around the hot pixels.
                            thisFeature.CalculateHeat_ShrinkBox(CurrInputImage, currGray);

                            featuresInBlock.AddFeature(thisFeature);
                            YoloProcess.LegFrameFeatures.Add(new YoloFeatureSeen { BlockId = blockID, Box = thisFeature.PixelBox, FeatureId = thisFeature.FeatureId });
                        }

                    currGray.Dispose();
                    currBmp.Dispose();
                }

                // Deprecated thisBlock.NumSig = YoloProcess.ProcessBlock(this, CurrInputImage, imgThreshold, results);
                YoloProcess.ProcessBlockForObjects(this, featuresInBlock);

                return thisBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoYoloDrone.AddBlockAndProcessInputVideoFrame", ex);
            }
        }


        // Return the data to show in the ObjectGrid in the Main Form
        public override List<object[]> GetObjectGridData()
        {
            return YoloProcess.ProcessObjects.GetObjectGridData(this, RunConfig.ProcessConfig, CategoryAll.ObjectCategories);
        }


        // Do any final activity at the end processing of video
        public override void RunEnd()
        {
            StandardSave dataWriter = new(Drone, DataStore);
            dataWriter.ProcessAll(DataStore, RunConfig, GetEffort(), GetSettings(), this, YoloProcess, true);

            base.RunEnd();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            StandardSave datawriter = new(Drone, DataStore);
            datawriter.ProcessAll(DataStore, RunConfig, GetEffort(), GetSettings(), this, YoloProcess, false);
            DataStore.FreeResources();
        }
    }
}