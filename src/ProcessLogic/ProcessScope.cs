// Copyright SkyComb Limited 2025. All rights reserved. 
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
        // All drone input data: video/images(definitely), flight(maybe) and ground(maybe) data
        public Drone? Drone { get; set; }

        public ProcessScopeModel PSM { get; }

        // Current flight step to process
        public FlightStep? CurrRunFlightStep { get; set; } = null;
        // First step of flight data to process 
        public int FirstRunStepId { get { return MinStepId; } }
        // Last step of flight data to process
        public int LastRunStepId { get { return MaxStepId; } }

        // Current (transient) thermal image data storage used while processing a Block and Objects.
        public Image<Gray, byte>? OriginalThermalImage = null; // Original is "best" raw thermal image from input video/image for human viewing.
        public Image<Gray, byte>? InputThermalImage = null; // Original after lower / higher cutoffs, etc for improved hotspot detection
        public Image<Bgr, byte>? OutputThermalImage = null; // Original overlaid with colored hotspots, etc for human viewing.
        // Current (transient) optical image data (if any) storage used while processing a Block and Objects.
        public string InputOpticalImagePath = "";
        public Image<Bgr, byte>? InputOpticalImage = null;
        public Image<Bgr, byte>? OutputOpticalImage = null; // Input optical overlaid with colored hotspots, etc for human viewing.
        // Transient values used while processing a Block and Objects.
        public ProcessBlock? CurrBlock = null;

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
            return Drone?.FlightSteps?.GetTardisModel(index);
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

            ResetInputThermal();
        }


        public void ResetInputThermal()
        {
            InputOpticalImagePath = "";
            
            OriginalThermalImage?.Dispose();
            OriginalThermalImage = null;

            InputThermalImage?.Dispose();
            InputThermalImage = null;
        }
        public void ResetOutputThermal()
        {
            OutputThermalImage?.Dispose();
            OutputThermalImage = null;
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

            Assert(PSM.FirstInputFrameId <= PSM.LastInputFrameId, "SetInputScope");
            Assert(PSM.FirstVideoFrameMs <= PSM.LastVideoFrameMs, "SetInputScope");
        }


        // Given Config.RunVideoFromS and Config.RunVideoToS, which input video frames will we process?
        public void CalculateInputScope(DroneIntervalModel interval)
        {
            if (Drone.InputIsVideo)
            {
                (PSM.FirstInputFrameId, PSM.LastInputFrameId, PSM.FirstVideoFrameMs, PSM.LastVideoFrameMs) =
                    Drone.InputVideo.CalculateFromToS(interval.RunVideoFromS, interval.RunVideoToS);
                PSM.FirstInputFrameId = Math.Max(1, PSM.FirstInputFrameId);
                PSM.LastInputFrameId = Math.Max(1, PSM.LastInputFrameId);
            }
            else
            {
                PSM.FirstInputFrameId = (interval.RunImagesFromStepId >= 0 ? interval.RunImagesFromStepId : 0);
                PSM.LastInputFrameId = (interval.RunImagesFromStepId >= 0 ? interval.RunImagesToStepId : Drone.FlightSections.Sections.Count - 1);
                PSM.FirstVideoFrameMs = Drone.FlightSections.Sections[PSM.FirstInputFrameId].SumTimeMs;
                PSM.LastVideoFrameMs = Drone.FlightSections.Sections[PSM.LastInputFrameId].SumTimeMs;
            }
        }


        public void ConfigureScope_SetFramePos(DroneIntervalModel interval)
        {
            Drone.ResetCurrFrame();

            CalculateInputScope(interval);

            Drone.SetAndGetCurrFrame(PSM.FirstInputFrameId);

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
            ResetInputThermal();

            if (Drone.HaveFrame())
            {
                Mat inputMat = Drone.CurrFrame();

                CalculateSettings();

                OriginalThermalImage = inputMat.ToImage<Gray, byte>();
                InputThermalImage = OriginalThermalImage.Clone();
            }
        }
    }
}
