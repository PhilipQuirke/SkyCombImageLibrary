// Copyright SkyComb Limited 2023. All rights reserved. 
using Compunet.YoloV8.Data;
using Compunet.YoloV8.Metadata;
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
            : base(parent, config, dataStore, drone, ProcessFactory.NewFlowProcessModel(config.ProcessConfig, drone))
        {
            // YoloV8 Predictor class
            FlowModel.YoloModel = new YoloV8(config.ModelDirectory);
        }


        public FlowProcessAll FlowModel { get { return (FlowProcessAll)ProcessAll; } }


        // Add a block, transferring some flight data and process data into it
        private FlowBlock AddBlock()
        {
            FlowBlock currBlock = FlowModel.FlowBlocks.AddBlock(this, Drone);

            if (PSM.CurrRunStepId <= 0)
                PSM.CurrRunStepId = currBlock.TardisId;

            return currBlock;
        }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(int prevLegId, int currLegId)
        {
            FlowModel.ProcessFlightLegStartAndEnd(prevLegId, currLegId);
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = AddBlock();

                // We do not use Threshold or Smooth as we want to find as many GFTT features as possible.
                var currGray = DrawImage.ToGrayScale(CurrInputVideoFrame);

                var result = FlowModel.YoloModel.Detect(currGray.ToBitmap());
                if( result.Result != null)
                {
                    // Convert Boxes to FlowObjects
                    foreach (var box in result.Result.Boxes)
                    {
                        /*
                        FlowObject flowObject = new();
                        flowObject.Location = new(box.Bounds.X, box.Bounds.Y);
                        flowObject.Size = new(box.Bounds.Width, box.Bounds.Height);
                        flowObject.Confidence = box.Confidence;
                        flowObject.Class = box.Class.Name;
                        flowObject.ClassId = box.Class.Id;
                        flowObject.ClassColor = box.Class.Color;
                        flowObject.ClassType = box.Class.Type;
                        flowObject.ClassDescription = box.Class.Description;
                        */
                    }
                }

                // Update the persisted gray frame 
                PrevGray = currGray.Clone();

                thisBlock.NumSig = 0; // PQR TODO

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
            return "#Features=" + FlowModel.FlowObjects.Count;
        }


        // Draw a single frame for Optical Flow process as circles with tails
        public override (Image<Bgr, byte>, Image<Bgr, byte>) DrawVideoFrames(ProcessBlockModel block, Image<Bgr, byte> InputFrame, Image<Bgr, byte> DisplayFrame)
        {
            var modifiedInputFrame = InputFrame.Clone();

            DrawImage.Palette(RunConfig.ImageConfig, ref modifiedInputFrame);

            DrawFlow.Draw(FlowModel, block.BlockId, ref modifiedInputFrame);

            return (modifiedInputFrame.Clone(), DisplayFrame.Clone());
        }


        // Do any final activity at the end processing of video
        public override void EndRunning()
        {
            FlowSave dataWriter = new(Drone, DataStore);
            dataWriter.Flow(RunConfig, GetEffort(), GetSettings(), this, FlowModel);

            base.EndRunning();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            FlowSave datawriter = new(Drone, DataStore);
            datawriter.Flow(RunConfig, GetEffort(), GetSettings(), this, FlowModel);
            DataStore.Close();
        }
    }
}