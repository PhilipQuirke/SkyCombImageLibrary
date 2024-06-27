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
            : base(parent, config, dataStore, drone, ProcessFactory.NewYoloProcessModel(config.ProcessConfig, drone, config.YoloDirectory))
        {
    
        }


        public YoloProcess YoloProcess { get { return (YoloProcess)ProcessAll; } }


        // Add a block, transferring some flight data and process data into it
        private ProcessBlock AddBlock()
        {
            var newBlock = ProcessFactory.NewBlock(this);

            YoloProcess.YoloBlocks.AddBlock(newBlock, this, Drone);

            if (PSM.CurrRunStepId <= 0)
                PSM.CurrRunStepId = newBlock.TardisId;

            return newBlock;
        }


        // Process start &/or end of drone flight legs.
        public override void ProcessFlightLegChange(int prevLegId, int currLegId)
        {
            YoloProcess.ProcessFlightLegStartAndEnd(prevLegId, currLegId);
        }


        // Process/analyse a single frame
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = AddBlock();

                // We do not use Threshold or Smooth as we want to find as many GFTT features as possible.
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
            return "#Features=" + YoloProcess.YoloObjects.Count;
        }


        public override (Image<Bgr, byte>, Image<Bgr, byte>) DrawVideoFrames(ProcessBlockModel block, Image<Bgr, byte> InputFrame, Image<Bgr, byte> DisplayFrame)
        {
            var modifiedInputFrame = InputFrame.Clone();

            DrawImage.Palette(RunConfig.ImageConfig, ref modifiedInputFrame);

            DrawYolo.Draw(RunConfig.ImageConfig, YoloProcess, block.BlockId, ref modifiedInputFrame);

            return (modifiedInputFrame.Clone(), DisplayFrame.Clone());
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