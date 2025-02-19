// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    public class ProcessBlock : ProcessBlockModel
    {
        public FlightStep? FlightStep { get; set; } = null;


        public ProcessBlock(ProcessScope scope) : base(scope.PSM)
        {
        }


        // Constructor used when loading objects from the datastore
        public ProcessBlock(int blockId, List<string> settings, Drone drone) : base(blockId, settings)
        {
            if ((FlightStepId != UnknownValue) && (drone != null) && drone.HasFlightSteps)
                FlightStep = drone.FlightSteps.Steps[FlightStepId];
        }


        public void CalculateSettings(ProcessScope scope, Drone drone, ProcessBlock? prevBlock)
        {
            if ((drone != null) && drone.HasFlightSteps && (scope.PSM.CurrInputFrameId > 0))
            {
                CalculateSettings_FromSteps(drone, prevBlock, scope.PSM.CurrInputFrameMs);
                AssertGood();
            }
        }


        // Blocks often fall between two successive FlightSteps.
        // If so, interpolate Block attributes from the surrounding two steps.
        public void CalculateSettings_FromSteps(Drone drone, ProcessBlock? prevBlock, int blockMs)
        {
            var thisBlockId = this.BlockId;

            Assert((prevBlock == null) || (prevBlock.BlockId < thisBlockId), "CalculateSettings_FromSteps: Logic 1");

            // On rare occassions, at the start of a leg, successive blocks move backwards in time!
            // Cause is a deliberate reset of the video position to eliminate cumulative time errors.
            bool badBlockMs = ((prevBlock != null) && (prevBlock.InputFrameMs >= this.InputFrameMs));

            // Get the FlightStep AT or BEFORE blockMs            
            FlightStep? beforeStep;
            if ((prevBlock != null) && (prevBlock.FlightStep != null))
                beforeStep = drone.FlightSteps.FlightStepAtOrBeforeFlightMs(prevBlock.FlightStep, blockMs);
            else if (blockMs < drone.SectionIdToVideoMs(drone.FlightSteps.MinStepId))
                beforeStep = null;
            else
                beforeStep = drone.FlightSteps.FlightStepAtOrBeforeFlightMs(blockMs);
            bool haveBeforeStep = (beforeStep != null);
            int beforeStepMs = (haveBeforeStep ? beforeStep.SumTimeMs : 0);
            Assert((!haveBeforeStep) || (beforeStepMs <= blockMs), "CalculateSettings_FromSteps: Logic 2");

            // Get the FlightStep AFTER blockMs            
            FlightStep? afterStep = null;
            if (haveBeforeStep)
                drone.FlightSteps.Steps.TryGetValue(beforeStep.StepId + 1, out afterStep);
            bool haveAfterStep = (afterStep != null);
            int afterStepMs = (haveAfterStep ? afterStep.SumTimeMs : 0);
            Assert((!haveAfterStep) || (afterStepMs >= blockMs), "CalculateSettings_FromSteps: Logic 3");

            // With multiple Blocks per Step, this Block may be closer to nextStep than prevStep.
            // Use the closest Step values as defaults (including TimeMs & SumTimeMs).
            int rangeMs = afterStepMs - beforeStepMs;
            Assert((!haveAfterStep) || (rangeMs >= 0), "CalculateSettings_FromSteps: Logic 4");
            float nextWeighting = 0;
            if (haveBeforeStep && haveAfterStep && (rangeMs > 0))
            {
                nextWeighting = (float)(1.0 * (blockMs - beforeStepMs) / rangeMs);
                Assert(nextWeighting >= 0.0, "CalculateSettings_FromSteps: Logic 5");
                Assert(nextWeighting <= 1.0, "CalculateSettings_FromSteps: Logic 6");

                if (nextWeighting <= 0.5)
                    FlightStep = beforeStep;
                else
                    FlightStep = afterStep;
            }
            else if (haveBeforeStep)
                FlightStep = beforeStep;
            else if (haveAfterStep)
                FlightStep = afterStep;

            if (FlightStep != null)
            {
                FlightStepId = FlightStep.StepId;

                // Use many defaults from the closest Step values but override some values.
                CopyTardis(FlightStep);
                TardisId = thisBlockId;
                if (haveBeforeStep)
                {
                    StartTime = beforeStep.StartTime;
                    TimeMs = beforeStep.TimeMs;
                }

                // A single FlightStep may be shared by a few blocks.
                // Unless this BlockMs is exactly prevStepMs, 
                // use linear interpolation between prevStep and the nextStep.
                if (haveBeforeStep && haveAfterStep && !badBlockMs)
                {
                    float prevWeighting = 1 - nextWeighting;

                    DroneLocnM.NorthingM = (float)(beforeStep.DroneLocnM.NorthingM * prevWeighting + afterStep.DroneLocnM.NorthingM * nextWeighting);
                    DroneLocnM.EastingM = (float)(beforeStep.DroneLocnM.EastingM * prevWeighting + afterStep.DroneLocnM.EastingM * nextWeighting);

                    if ((beforeStep.YawDeg != UnknownValue) && (afterStep.YawDeg != UnknownValue))
                        // PQR TODO Does not handle a shift from yaw -176 to +179
                        YawDeg = (float)(beforeStep.YawDeg * prevWeighting + afterStep.YawDeg * nextWeighting);
                    else if (afterStep.YawDeg != UnknownValue)
                        YawDeg = afterStep.YawDeg;
                    else
                        YawDeg = beforeStep.YawDeg;

                    if ((beforeStep.PitchDeg != UnknownValue) && (afterStep.PitchDeg != UnknownValue))
                        PitchDeg = (float)(beforeStep.PitchDeg * prevWeighting + afterStep.PitchDeg * nextWeighting);
                    else if (afterStep.PitchDeg != UnknownValue)
                        PitchDeg = afterStep.PitchDeg;
                    else
                        PitchDeg = beforeStep.PitchDeg;

                    if ((beforeStep.RollDeg != UnknownValue) && (afterStep.RollDeg != UnknownValue))
                        RollDeg = (float)(beforeStep.RollDeg * prevWeighting + afterStep.RollDeg * nextWeighting);
                    else if (afterStep.RollDeg != UnknownValue)
                        RollDeg = afterStep.RollDeg;
                    else
                        RollDeg = beforeStep.RollDeg;

                    // Over the leg the drone will move up and down to stay ~60M above the terrain.
                    // In video ???2614 at 4m20s the drone altitude is decreasing 0.5m every 1/4s!! Or 2m/s!!
                    // Drone altitude can change by say 0.3m between FlightSteps. Smooth the block Altitude.
                    if ((beforeStep.AltitudeM != UnknownValue) && (afterStep.AltitudeM != UnknownValue))
                        AltitudeM = (float)(beforeStep.AltitudeM * prevWeighting + afterStep.AltitudeM * nextWeighting);
                    else if (afterStep.AltitudeM != UnknownValue)
                        AltitudeM = afterStep.AltitudeM;
                    else
                        AltitudeM = beforeStep.AltitudeM;
                }
            }

            RelativeLocation travelM;
            // Must override some FlightStep defaults as there are multiple blocks per FlightStep.
            if (badBlockMs)
            {
                travelM = RelativeLocation.TravelM(prevBlock.DroneLocnM, DroneLocnM);
                TimeMs = (int)(1000.0 / drone.InputVideo.Fps); // Default frame duration.
            }
            else if (prevBlock != null)
            {
                travelM = RelativeLocation.TravelM(prevBlock.DroneLocnM, DroneLocnM);
                TimeMs = InputFrameMs - prevBlock.InputFrameMs;
                int advanceMS = blockMs - beforeStepMs;
                Assert(advanceMS >= 0, "CalculateSettings_FromSteps: Logic 10");
                StartTime += TimeSpan.FromMilliseconds(advanceMS);
            }
            else
            {
                travelM = new();
                TimeMs = (int)(1000.0 / drone.InputVideo.Fps); // Default frame duration.
                int advanceMS = blockMs - beforeStepMs;
                Assert(advanceMS >= 0, "CalculateSettings_FromSteps: Logic 11");
                StartTime += TimeSpan.FromMilliseconds(advanceMS);
            }
            LinealM = travelM.DiagonalM;

            if (prevBlock != null)
                SumLinealM = prevBlock.SumLinealM + LinealM;
            else
                SumLinealM = LinealM;

            if (prevBlock != null)
                Assert(SumLinealM >= prevBlock.SumLinealM, "CalculateSettings_FromSteps: Logic 8");

            if ((prevBlock != null) && (!badBlockMs))
                Assert(prevBlock.StartTime < this.StartTime, "CalculateSettings_FromSteps: Logic 9");
            if (this.FlightStep != null)
                Assert(Math.Abs(this.FlightStep.SumTimeMs - this.SumTimeMs) < 3000, "CalculateSettings_FromSteps: Logic 10");

            // Check that this block is inside the summary envelope
            // generated from the previous and next flight steps.
            if ((afterStep != null) && (beforeStep != null))
            {
                TardisSummaryModel stepSummary = new("Step");
                stepSummary.SummariseTardis(beforeStep);
                stepSummary.SummariseTardis(afterStep);

                TardisSummaryModel blockSummary = new("Block");
                blockSummary.SummariseTardis(this);

                blockSummary.AssertGoodSubset(stepSummary, false);
            }
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore).
        public override DataPairList GetSettings()
        {
            var answer = base.GetSettings();

            if (FlightStep != null)
                FlightStep.AppendStepToBlockSettings(ref answer);
            else
            {
                answer.Add("DsmM", 0, HeightNdp);
                answer.Add("DemM", 0, HeightNdp);
            }

            answer.Add("HasLeg", (FlightLegId <= 0 ? 0 : 1));

            AssertGood();

            return answer;
        }
    };


    public class ProcessBlockList : SortedList<int, ProcessBlock>
    {
        public void AddBlock(ProcessBlock newBlock, ProcessScope? scope, Drone drone)
        {
            ProcessBlock? prevBlock = LastBlock;

            BaseConstants.Assert(newBlock.BlockId > 0, "ProcessBlockList.AddBlock: No Id");
            Add(newBlock.BlockId, newBlock);

            if (scope != null)
            {
                newBlock.CalculateSettings(scope, drone, prevBlock);

                BaseConstants.Assert(newBlock.BlockId == scope.PSM.CurrBlockId, "AddBlock: Bad scope");
                BaseConstants.Assert(this.Count == scope.PSM.CurrBlockId, "AddBlock: Bad Blocks count");
            }
        }


        public ProcessBlock? LastBlock
        {
            get
            {
                if (Count > 0)
                    return this.Last().Value;

                return null;
            }
        }


        // Returns the distance in meters between the fromBlock and the toBlock locations
        public double DistanceM(ProcessBlock? fromBlock, ProcessBlock? toBlock)
        {
            if ((fromBlock == null) || (toBlock == null) || (fromBlock.BlockId == toBlock.BlockId))
                return 0;

            return RelativeLocation.DistanceM(fromBlock.DroneLocnM, toBlock.DroneLocnM);
        }


        // Convert from a specific time in the video to the closest Block ID of a child block
        public int TimeMsToBlockId(double timeMs)
        {
            int theBlockId = BaseConstants.UnknownValue;

            if (Count > 0)
            {
                double minDiff = double.MaxValue;
                foreach (var block in this)
                {
                    double diff = Math.Abs(timeMs - block.Value.InputFrameMs);
                    if (diff < minDiff)
                    {
                        minDiff = diff;
                        theBlockId = block.Value.BlockId;
                    }
                }
            }

            return theBlockId;
        }
    };

}
