// Copyright SkyComb Limited 2024. All rights reserved. 
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
        // All drone input data: video(definitely), flight(maybe) and ground(maybe) data
        public Drone? Drone { get; set; }

        public ProcessScopeModel PSM { get; }

        // Current flight step to process
        public FlightStep? CurrRunFlightStep { get; set; } = null;
        // First step of flight data to process 
        public int FirstRunStepId { get { return MinStepId; } }
        // Last step of flight data to process
        public int LastRunStepId { get { return MaxStepId; } }

        public Image<Bgr, byte>? CurrInputImage { get; set; } = null;
        public Image<Bgr, byte>? CurrDisplayImage { get; set; } = null;



        public ProcessScope(Drone? drone = null)
        {
            PSM = new();
            Drone = drone;
        }
        public ProcessScope(ProcessScope other)
        {
            PSM = new(other.PSM);
            Drone = other.Drone;
            CurrRunFlightStep = other.CurrRunFlightStep;
            CopySteps(other);
        }


        // Return the child FlightStep
        public override TardisModel? GetTardisModel(int index)
        {
             return Drone == null ? null : Drone.FlightSteps.GetTardisModel(index);
        }


        public void ResetScope(FlightStep? fromStep, FlightStep? toStep)
        {
            ResetTardis();

            var fromStepId = 0;
            if (fromStep != null)
                fromStepId = fromStep.StepId;

            var toStepId = UnknownStepId;
            if (toStep != null)
                toStepId = toStep.StepId;
            else if ((Drone != null) && Drone.HasFlightSteps)
                // Default to the full flight
                toStepId = Drone.FlightSteps.MaxStepId;

            if ((Drone != null) && Drone.HasFlightSteps)
                Drone.FlightSteps.Steps.CalculateSettings_Summarise(this, fromStepId, toStepId);
            else
                this.ResetSteps();

            ResetCurrImages();
        }


        public void ResetCurrImages()
        {
            CurrInputImage?.Dispose();
            CurrInputImage = null;
            CurrDisplayImage?.Dispose();
            CurrDisplayImage = null;
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
        public void CalculateInputScope(float inputVideoFromS, float inputVideoToS)
        {
            (PSM.FirstInputFrameId, PSM.LastInputFrameId, PSM.FirstVideoFrameMs, PSM.LastVideoFrameMs) =
                Drone.InputVideo.CalculateFromToS(inputVideoFromS, inputVideoToS);
        }


        public void ConfigureScope_SetFramePos( float inputVideoFromS, float inputVideoToS)
        {
            Drone.ResetCurrFrames();

            CalculateInputScope(inputVideoFromS, inputVideoToS);

            Drone.SetAndGetCurrFrames(PSM.FirstInputFrameId);

            if (Drone.HasFlightSteps)
            {
                SetCurrRunStepAndLeg(Drone.MsToNearestFlightStep(PSM.FirstVideoFrameMs));

                var toStep = Drone.MsToNearestFlightStep(PSM.LastVideoFrameMs);
                ResetScope(CurrRunFlightStep, toStep);
            }
            else
            {
                ResetScope(null, null);
                PSM.CurrRunLegId = 1;
            }

            Assert(Drone.InputVideo.CurrFrameId == PSM.FirstInputFrameId, "ProcessScope.ConfigureScope_SetFramePos: Bad FrameID");

            PSM.CurrBlockId = 1;
        }


        public void CalculateSettings()
        {
            SetCurrVideoFrameData(Drone.InputVideo, Drone.DisplayVideo);
            if (Drone.HasFlightSteps)
                SetCurrRunStepAndLeg(Drone.MsToNearestFlightStep(PSM.CurrInputFrameMs));
            else
                PSM.CurrRunLegId = 1;
        }


        // Return current input video frame and corresponding display video frame (if any)
        public void ConvertCurrImages()
        {
            ResetCurrImages();

            if (Drone.HaveFrames())
            {
                (Mat inputMat, Mat displayMat) = Drone.CurrFrames();

                CalculateSettings();

                CurrInputImage = inputMat.ToImage<Bgr, byte>();
                if (displayMat != null)
                    CurrDisplayImage = displayMat.ToImage<Bgr, byte>();
                else
                    CurrDisplayImage = inputMat.ToImage<Bgr, byte>();
            }
        }
    }
}
