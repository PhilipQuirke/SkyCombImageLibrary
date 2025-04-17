// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
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


        public void ResetScope(FlightStep? fromStep = null, FlightStep? toStep = null)
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

            ResetCurrImage();
        }


        public void ResetCurrImage()
        {
            CurrInputImage?.Dispose();
            CurrInputImage = null;
        }


        // Set the current flight step member data
        public void SetCurrRunStepAndLeg(FlightStep? step)
        {
            if (step == null)
                PSM.CurrRunLegId = 1;
            else
            {
                CurrRunFlightStep = step;
                PSM.CurrRunStepId = (step != null ? step.FlightSection.TardisId : UnknownValue);
                PSM.CurrRunLegId = (step != null ? step.FlightLegId : UnknownValue);
            }
        }


        // Use the exact positions from the Blocks
        public void SetInputScope(ProcessBlock firstBlock, ProcessBlock lastBlock)
        {
            PSM.FirstInputFrameId = firstBlock.InputFrameId;
            PSM.FirstVideoFrameMs = firstBlock.InputFrameMs;

            PSM.LastInputFrameId = lastBlock.InputFrameId;
            PSM.LastVideoFrameMs = lastBlock.InputFrameMs;

            BaseConstants.Assert(PSM.FirstInputFrameId <= PSM.LastInputFrameId, "SetInputScope");
            BaseConstants.Assert(PSM.FirstVideoFrameMs <= PSM.LastVideoFrameMs, "SetInputScope");
        }


        // Given Config.RunVideoFromS and Config.RunVideoToS, which input video frames will we process?
        public void CalculateInputScope(float inputVideoFromS, float inputVideoToS)
        {
            if (Drone.InputIsVideo)
                (PSM.FirstInputFrameId, PSM.LastInputFrameId, PSM.FirstVideoFrameMs, PSM.LastVideoFrameMs) =
                    Drone.InputVideo.CalculateFromToS(inputVideoFromS, inputVideoToS);
            else
            {
                var fromSection = Drone.FlightSections.SecondToFlightSection(inputVideoFromS);
                var toSection = Drone.FlightSections.SecondToFlightSection(inputVideoToS);
                PSM.FirstInputFrameId = fromSection.SectionId;
                PSM.LastInputFrameId = toSection.SectionId;
                PSM.FirstVideoFrameMs = fromSection.SumTimeMs;
                PSM.LastVideoFrameMs = toSection.SumTimeMs;
            }
        }


        public void ConfigureScope_SetFramePos(float inputVideoFromS, float inputVideoToS)
        {
            Drone.ResetCurrFrame();

            CalculateInputScope(inputVideoFromS, inputVideoToS);

            if(Drone.InputIsVideo)
                Drone.SetAndGetCurrFrame(PSM.FirstInputFrameId);
            else
            {
                Drone.InputVideo.ResetCurrFrame();
                Drone.InputVideo.CurrFrameId = PSM.FirstInputFrameId;
                Drone.InputVideo.CurrFrameMs = PSM.FirstVideoFrameMs;
            }

            FlightStep? firstStep = null;
            FlightStep? lastStep = null;
            if (Drone.HasFlightSteps)
            {
                if (Drone.InputIsVideo)
                {
                    firstStep = Drone.MsToNearestFlightStep(PSM.FirstVideoFrameMs);
                    lastStep = Drone.MsToNearestFlightStep(PSM.LastVideoFrameMs);
                }
                else
                {
                    firstStep = Drone.FlightSteps?.Steps[PSM.FirstInputFrameId];
                    lastStep = Drone.FlightSteps?.Steps[PSM.LastInputFrameId];
                }
            }
            SetCurrRunStepAndLeg(firstStep);
            ResetScope(firstStep, lastStep);

            Assert(Drone.InputVideo.CurrFrameId == PSM.FirstInputFrameId, "ProcessScope.ConfigureScope_SetFramePos: Bad FrameID");

            PSM.CurrBlockId = 1;
        }


        public void CalculateSettings()
        {
            PSM.CurrInputFrameId = Drone.InputVideo.CurrFrameId;
            PSM.CurrInputFrameMs = Drone.InputVideo.CurrFrameMs;

            FlightStep step = null;
            if (Drone.InputIsVideo)
                step = Drone?.MsToNearestFlightStep(PSM.CurrInputFrameMs);
            else
                step = Drone?.FlightSteps?.Steps[PSM.FirstInputFrameId];
            SetCurrRunStepAndLeg(step);
        }


        // Return current input video frame and corresponding display video frame (if any)
        public void ConvertCurrImage_InputIsVideo()
        {
            ResetCurrImage();

            if (Drone.HaveFrame())
            {
                Mat inputMat = Drone.CurrFrame();

                CalculateSettings();

                CurrInputImage = inputMat.ToImage<Bgr, byte>();
            }
        }
    }
}
