// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // The Video, Drone & model scope of a processing run
    public class ProcessScope : FlightStepSummaryModel
    {
        public ProcessScopeModel PSM { get; set; } = null;

        // Current flight step to process
        public FlightStep CurrRunFlightStep { get; set; } = null;
        // First step of flight data to process 
        public int FirstRunStepId { get { return MinStepId; } }
        // Last step of flight data to process
        public int LastRunStepId { get { return MaxStepId; } }


        public ProcessScope()
        {
            PSM = new();
        }
        public ProcessScope(ProcessScope other)
        {
            PSM = new(other.PSM);
            CurrRunFlightStep = other.CurrRunFlightStep;
            CopySteps(other);
        }


        public void ResetScope(Drone drone, FlightStep fromStep, FlightStep toStep)
        {
            ResetTardis();

            var fromStepId = 0;
            if (fromStep != null)
                fromStepId = fromStep.StepId;

            var toStepId = UnknownStepId;
            if (toStep != null)
                toStepId = toStep.StepId;
            else if ((drone != null) && drone.HasFlightSteps)
                // Default to the full flight
                toStepId = drone.FlightSteps.MaxStepId;

            if ((drone != null) && drone.HasFlightSteps)
                drone.FlightSteps.Steps.CalculateSettings_Summarise(this, fromStepId, toStepId);
            else
                this.ResetSteps();
        }


        // Set the current flight step member data
        public void SetCurrRunStepAndLeg(FlightStep step)
        {
            CurrRunFlightStep = step;
            PSM.CurrRunStepId = (step != null ? step.FlightSection.TardisId : UnknownValue);
            PSM.CurrRunLegId = (step != null ? step.FlightLegId : UnknownValue);
        }


        // Set the current input video member data
        public void SetCurrVideoFrameData(VideoData inputVideo, VideoData displayVideo)
        {
            PSM.CurrInputFrameId = inputVideo.CurrFrameId;
            PSM.CurrInputFrameMs = inputVideo.CurrFrameMs;

            if (displayVideo != null)
            {
                PSM.CurrDisplayFrameId = displayVideo.CurrFrameId;
                PSM.CurrDisplayFrameMs = displayVideo.CurrFrameMs;
            }
        }


        // Use the exact positions from the Blocks
        public void SetInputScope(ProcessBlock firstBlock, ProcessBlock lastBlock)
        {
            PSM.FirstInputFrameId = firstBlock.InputFrameId;
            PSM.FirstVideoFrameMs = firstBlock.InputFrameMs;

            PSM.LastInputFrameId = lastBlock.InputFrameId;
            PSM.LastVideoFrameMs = lastBlock.InputFrameMs;
        }


        // Given Config.RunVideoFromS and Config.RunVideoToS, which input video frames will we process?
        public void CalculateInputScope(Drone drone, float inputVideoFromS, float inputVideoToS)
        {
            (PSM.FirstInputFrameId, PSM.LastInputFrameId, PSM.FirstVideoFrameMs, PSM.LastVideoFrameMs) =
                drone.InputVideo.CalculateFromToS(inputVideoFromS, inputVideoToS);
        }


        public void ConfigureScope_SetFramePos(Drone drone,
            float inputVideoFromS, float inputVideoToS)
        {
            drone.ResetCurrFrames();

            CalculateInputScope(drone, inputVideoFromS, inputVideoToS);

            drone.SetAndGetCurrFrames(PSM.FirstInputFrameId);

            if (drone.HasFlightSteps)
            {
                SetCurrRunStepAndLeg(drone.MsToNearestFlightStep(PSM.FirstVideoFrameMs));

                var toStep = drone.MsToNearestFlightStep(PSM.LastVideoFrameMs);
                ResetScope(drone, CurrRunFlightStep, toStep);
            }
            else
            {
                ResetScope(null, null, null);
                PSM.CurrRunLegId = 1;
            }

            Assert(drone.InputVideo.CurrFrameId == PSM.FirstInputFrameId, "ProcessScope.ConfigureScope_SetFramePos: Bad FrameID");

            PSM.CurrBlockId = 1;
        }


        public void CalculateSettings(Drone drone)
        {
            SetCurrVideoFrameData(drone.InputVideo, drone.DisplayVideo);
            if (drone.HasFlightSteps)
                SetCurrRunStepAndLeg(drone.MsToNearestFlightStep(PSM.CurrInputFrameMs));
            else
                PSM.CurrRunLegId = 1;
        }


        // Return current input video frame and corresponding display video frame (if any)
        public (Image<Bgr, byte> inputImage, Image<Bgr, byte> displayImage) ConvertImages(Drone drone)
        {
            Image<Bgr, byte> inputImage = null;
            Image<Bgr, byte> displayImage = null;

            if (drone.HaveFrames())
            {
                (Mat inputMat, Mat displayMat) = drone.CurrFrames();

                CalculateSettings(drone);

                inputImage = inputMat.ToImage<Bgr, byte>();
                if (displayMat != null)
                    displayImage = displayMat.ToImage<Bgr, byte>();
                else
                    displayImage = inputMat.ToImage<Bgr, byte>();
            }

            return (inputImage, displayImage);
        }

    }
}
