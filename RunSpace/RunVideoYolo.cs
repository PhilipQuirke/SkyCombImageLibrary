// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using System.Drawing;


// Namespace for processing of a video made up of multiple images (frames).
// Some classes contain code that persists information between frames
namespace SkyCombImage.RunSpace
{
    // YOLO (You only look once) V8 video processing.
    class RunVideoYolo : RunVideoPersist
    {
        public RunVideoYolo(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone) 
            : base(parent, config, dataStore, drone, ProcessFactory.NewYoloProcessModel(config.ProcessConfig, drone, config.ModelDirectory))
        {
    
        }


        public YoloProcessAll YoloModel { get { return (YoloProcessAll)ProcessAll; } }


        // Add a block, transferring some flight data and process data into it
        private ProcessBlock AddBlock()
        {
            var newBlock = ProcessFactory.NewBlock(this);

            YoloModel.YoloBlocks.AddBlock(newBlock, this, Drone);

            if (PSM.CurrRunStepId <= 0)
                PSM.CurrRunStepId = newBlock.TardisId;

            return newBlock;
        }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(int prevLegId, int currLegId)
        {
            YoloModel.ProcessFlightLegStartAndEnd(prevLegId, currLegId);
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = AddBlock();
                int numSig = 0;

                // We do not use Threshold or Smooth as we want to find as many GFTT features as possible.
                var currGray = DrawImage.ToGrayScale(CurrInputVideoFrame);

                var result = YoloModel.YoloModel.Detect(currGray.ToBitmap());
                if( result.Result != null)
                {
                    numSig = result.Result.Boxes.Count();

                    // Convert Boxes to YoloObjects
                    foreach (var box in result.Result.Boxes)
                    {
                        // We have found a new feature/object
                        var newFeature = YoloModel.YoloFeatures.AddFeature(thisBlock.BlockId, new System.Drawing.Point(box.Bounds.X, box.Bounds.Y));
                        var newObject = YoloModel.YoloObjects.AddObject(this, newFeature, 
                            box.Class.Name, System.Drawing.Color.Red, box.Confidence);
                        YoloModel.ObjectClaimsNewFeature(thisBlock, newObject, newFeature);

                        /*
                        newObject.Size = new(box.Bounds.Width, box.Bounds.Height);
                        newObject.Confidence = box.Confidence;
                        newObject.Class = box.Class.Name;
                        newObject.ClassId = box.Class.Id;
                        newObject.ClassColor = box.Class.Color;
                        newObject.ClassType = box.Class.Type;
                        newObject.ClassDescription = box.Class.Description;
                        */
                    }
                }

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
            return "#Features=" + YoloModel.YoloObjects.Count;
        }


        public override (Image<Bgr, byte>, Image<Bgr, byte>) DrawVideoFrames(ProcessBlockModel block, Image<Bgr, byte> InputFrame, Image<Bgr, byte> DisplayFrame)
        {
            var modifiedInputFrame = InputFrame.Clone();

            DrawImage.Palette(RunConfig.ImageConfig, ref modifiedInputFrame);

            DrawYolo.Draw(RunConfig.ImageConfig, YoloModel, block.BlockId, ref modifiedInputFrame);

            return (modifiedInputFrame.Clone(), DisplayFrame.Clone());
        }


        // Do any final activity at the end processing of video
        public override void EndRunning()
        {
            YoloSave dataWriter = new(Drone, DataStore);
            dataWriter.Yolo(RunConfig, GetEffort(), GetSettings(), this, YoloModel);

            base.EndRunning();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            YoloSave datawriter = new(Drone, DataStore);
            datawriter.Yolo(RunConfig, GetEffort(), GetSettings(), this, YoloModel);
            DataStore.Close();
        }
    }
}