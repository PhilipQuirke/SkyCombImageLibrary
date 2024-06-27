// Copyright SkyComb Limited 2023. All rights reserved. 
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
    // Class to implement the CalcOpticalFlowPyrLK method using GFTT method to detect features
    class RunVideoFlow : RunVideoPersist
    {
        public RunVideoFlow(RunParent parent, RunConfig runConfig, DroneDataStore dataStore, Drone drone) 
            : base(parent, runConfig, dataStore, drone, ProcessFactory.NewFlowProcessModel(runConfig.ProcessConfig, drone))
        {
        }


        public FlowProcess FlowModel { get { return (FlowProcess)ProcessAll; } }


        // Add a block, transferring some flight data and process data into it
        private FlowBlock AddBlock()
        {
            FlowBlock currBlock = FlowModel.FlowBlocks.AddBlock(this, Drone);

            if (PSM.CurrRunStepId <= 0)
                PSM.CurrRunStepId = currBlock.TardisId;

            // When there is a big gap (e.g. DJI_088, section 477 jumps to 485) this assert fails.
            // Assert(Math.Abs(this.CurrRunStepId - currBlock.TardisId ) <= 2, "RunVideoFlow.AddBlock: TardisId mismatch");

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

                int numSignificantFlowObjects = 0;

                // For debugging. Place breakpoint on assignment. Value is overridden later.
                if ((PSM.CurrBlockId >= 19) && (PSM.CurrBlockId <= 21))
                    numSignificantFlowObjects = 5;

                if (PSM.CurrBlockId == 1)
                    // Detect new "Good features to track", then add them into the Flow model 
                    numSignificantFlowObjects =
                        FlowModel.ProcessNewGftt(
                            DrawImage.Detect_GFTT(RunConfig.ProcessConfig, currGray),
                            this);
                else
                {
                    // Use CalcOpticalFlowPyrLK (aka Optical Flow) process to detect
                    // movements of the GFTT features between the images.
                    numSignificantFlowObjects = FlowModel.ProcessBlock(this, PrevGray, currGray);

                    thisBlock.CalculateFlowVelocities();

                    // Features will flow off the side of the video, so periodically we add more GFTT features.
                    if (numSignificantFlowObjects < 0.9 * RunConfig.ProcessConfig.GfttMaxCorners)
                        // This will add new "corners" to get us up to or close to GfttMaxCorners 
                        numSignificantFlowObjects =
                            FlowModel.ProcessNewGftt(
                                DrawImage.Detect_GFTT(RunConfig.ProcessConfig, currGray, RunConfig.ProcessConfig.GfttMaxCorners - thisBlock.NumSig),
                                this);
                }


                // Update the persisted gray frame 
                PrevGray = currGray.Clone();

                thisBlock.NumSig = numSignificantFlowObjects;

                return thisBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoFlow.ProcessInputVideoFrame", ex);
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