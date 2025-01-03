// Copyright SkyComb Limited 2025. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Diagnostics;


// Q1: Is zoom not zero?
// Q2: Is there some factor in image I am mising related to elevation? Why does image appear 70 CDDown when it is 90 CDDown?
// Q3: Save the object DEM in the Imagedata datastore tab.
// Q4: Draw DEM values on output image.


namespace SkyCombImage.ProcessLogic
{
    // ProcessSpan relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpan analyses ProcessObjects to refine/correct the flight altitude data using FlightStep.FixAltM/FixYawDeg/FixPitchDeg.
    public class ProcessSpan : ProcessSpanModel
    {
        // Parent process
        private ProcessAll Process { get; }


        public ProcessSpan(ProcessAll process, int spanId, List<string>? settings = null) : base(settings)
        {
            Process = process;
            ProcessSpanId = spanId;

            if (settings != null)
                LoadSettings(settings);
        }


        public void AssertGood()
        {
            Assert(ProcessSpanId > 0, "ProcessSpan.AssertGood: Bad SpanId");
            Assert(MinBlockId > 0, "ProcessSpan.AssertGood: Bad MinBlockId");
            Assert(MaxBlockId > 0, "ProcessSpan.AssertGood: Bad MaxBlockId");
        }


        // Apply FixValues to theSteps and on to the ProcessObjects and their ProcessFeatures
        public void CalculateSettings_ApplyFixValues_Core(int hFOVDeg, float fixAltM, float fixYawDeg, float fixPitchDeg, FlightStepList theSteps, ProcessObjList objs)
        {
            // The image area of the camera now covers a slightly different area (m2) of the ground
            Process.Drone.InputVideo.HFOVDeg = hFOVDeg;
            Assert(Process.VideoData.HFOVDeg == hFOVDeg, "CalculateSettings_ApplyFixValues_Core");

            // The image associated with each leg step now covers a slightly different area
            // Recalculate InputImageCenter Dem and Dsm based on FixAltM/FixYawDeg/FixPitchDeg
            theSteps.CalculateSettings_FixValues(fixAltM, fixYawDeg, fixPitchDeg, Process.VideoData, Process.Drone.GroundData);

            foreach (var theObject in objs)
            {
                var theObj = theObject.Value;

                // Clone the list of features (not the features themselves) claimed by the object 
                var objectFeatures = theObj.ProcessFeatures.Clone();

                // Eliminate all object summary data.
                theObj.ResetCalcedMemberData();

                // Recalc each feature - which will have a slightly different location
                foreach (var theFeature in objectFeatures)
                {
                    var theFeat = theFeature.Value;
                    theFeat.ResetCalcedMemberData();

                    // This code depends on ProcessAll.VideoData.HFOVDeg, FlightStep.FixAltM, FixYawDeg and FixPitchDeg  
                    // This code does NOT depend on FlightStep.InputImageCenter/InputImageSize/InputImageUnitVector/InputImageDemM/InputImageDsmM
                    theFeat.CalculateSettings_LocationM_HeightM_LOS(Process.GroundData);

                    theObj.ClaimFeature(theFeat);
                }
            }

            // Calculate the revised object-list location and height errors.
            objs.CalculateSettings();
        }


        // Apply FixValues to theSteps and on to the ProcessObjects and their ProcessFeatures
        public bool CalculateSettings_ApplyFixValues(int hFOVDeg, float fixAltM, float fixYawDeg, float fixPitchDeg, FlightStepList theSteps, ProcessObjList objs)
        {
            CalculateSettings_ApplyFixValues_Core(hFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, theSteps, objs);

            // Do we see a drop in location error sum?
            // The location error assumes the objects are mostly stationary over the time they are observed.
            var improvementM = BestSumLocnErrM - objs.SumLocationErrM;
            if (improvementM >= 0.1f ) // at least a 10cm improvement
            {
                SetBest(hFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, objs);
                return true;
            }

            return false;
        }


        // Analyse ProcessObjects in the selected steps by assuming the drone altitude is inaccurate.
        // Apply various "Fix*" trial values to the FlightSteps, ProcessFeatures & ProcessObjects.
        // For each trial, measure the sum of the object location errors.
        // Lock in the legSteps.Fix* value that reduces the error most.
        public void CalculateSettings_FixValues(FlightStepList theSteps, ProcessObjList theObjs)
        {
            try
            {
                Debug.Print("CalculateSettings_FixAltM_Start");
                NumSignificantObjects = theObjs.Count;

                if ((theSteps.Count == 0) || (NumSignificantObjects == 0))
                    return;

                //NQ Optimise
                float altRangeM = 25;
                float yawRangeDeg = 10;
                float pitchRangeDeg = 20;

                int theHFOVDeg = Process.Drone.InputVideo.HFOVDeg;

                ResetBest();
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, theSteps, theObjs);
                OrgSumLocnErrM = BestSumLocnErrM;
                OrgSumHeightErrM = BestSumHeightErrM;

                if (true)
                {
                    // DIMENSION 1: Video HFOVDeg
                    // The drone.InputVideo.HFOVDeg value is guessed from attributes of the SRT file. This is weak.
                    // Known hardware values for HFOVDeg are 38, 42 & 57. Test reasonable values around these.
                    CalculateSettings_ApplyFixValues(36, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(38, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(40, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(42, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(44, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(57, 0, 0, 0, theSteps, theObjs);
                    theHFOVDeg = BestHFOVDeg;

                    // DIMENSION 2: Drone altitude 
                    ResetBest();
                    for (float fixAltM = -altRangeM; fixAltM <= altRangeM; fixAltM += 1)
                        CalculateSettings_ApplyFixValues(theHFOVDeg, fixAltM, 0, 0, theSteps, theObjs);
                    var bestAltM = BestFixAltM;
                    var bestAltErr = BestSumLocnErrM;

                    // DIMENSION 3: Drone yaw 
                    ResetBest();
                    for (float fixYawDeg = -yawRangeDeg; fixYawDeg <= yawRangeDeg; fixYawDeg += 1)
                        CalculateSettings_ApplyFixValues(theHFOVDeg, 0, fixYawDeg, 0, theSteps, theObjs);
                    var bestYawDeg = BestFixYawDeg;
                    var bestYawErr = BestSumLocnErrM;

                    // DIMENSION 4: Drone pitch 
                    ResetBest();
                    for (float fixPitchDeg = -pitchRangeDeg; fixPitchDeg <= pitchRangeDeg; fixPitchDeg += 1)
                        CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, fixPitchDeg, theSteps, theObjs);
                    var bestPitchDeg = BestFixPitchDeg;
                    var bestPitchErr = BestSumLocnErrM;

                    // DIMENSION 2..4: Fine tune
                    // Vary dimensions 2 to 4 around the above rough values
                    ResetBest();
                    CalculateSettings_ApplyFixValues(theHFOVDeg, bestAltM, bestYawDeg, bestPitchDeg, theSteps, theObjs);
                    for (float fixAltM = bestAltM - 1.25f; fixAltM <= bestAltM + 1.25f; fixAltM += 0.25f)
                        for (float fixYawDeg = bestYawDeg - 1.25f; fixYawDeg <= bestYawDeg + 1.25f; fixYawDeg += 0.25f)
                            for (float fixPitchDeg = bestPitchDeg - 1.25f; fixPitchDeg <= bestPitchDeg + 1.25f; fixPitchDeg += 0.25f)
                                CalculateSettings_ApplyFixValues(theHFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, theSteps, theObjs);
                }


                // Lock in the best single value across the leg steps
                CalculateSettings_ApplyFixValues_Core(theHFOVDeg, BestFixAltM, BestFixYawDeg, BestFixPitchDeg, theSteps, theObjs);
                BestSumLocnErrM = theObjs.SumLocationErrM;
                BestSumHeightErrM = theObjs.SumHeightErrM;
                Process.ProcessObjects.CalculateSettings();

                Debug.Print("CalculateSettings_FixValues: theHFOVDeg=" + theHFOVDeg.ToString() + "BestFixAltM=\" + BestFixAltM.ToString() + \"BestFixYawDeg=" + BestFixYawDeg.ToString() + " BestFixPitchDeg=" + BestFixPitchDeg.ToString());
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessSpan.CalculateSettings_FixValues", ex);
            }
        }


        // Summarise the blocks in this leg
        private void SummariseSteps(FlightStepList steps)
        {
            if (steps.Count > 0)
            {
                MinStepId = steps.First().Value.StepId;
                MaxStepId = steps.Last().Value.StepId;
            }

            ResetTardis();
            foreach (var step in steps)
                SummariseTardis(step.Value);
        }


        // Analyse ProcessObjects in the FlightLeg, assuming various inaccuracies.
        // Lock in the FlightSteps.FixAltM/FixYawDeg/FixPitchDeg values that reduces the location wobble most.
        public void CalculateSettings_from_FlightLeg()
        {
            ResetBest();
            ResetTardis();

            if (ProcessSpanId == UnknownValue)
                return;

            var legSteps = Process.Drone.FlightSteps.Steps.GetLegSteps(ProcessSpanId);
            var theObjs = Process.ProcessObjects.FilterByLeg(ProcessSpanId);

            CalculateSettings_FixValues(legSteps, theObjs);
            SummariseSteps(legSteps);
        }


        // Analyse ProcessObjects in the Block series - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM/FixYawDeg/FixPitchDeg values that reduces the location most.
        public void CalculateSettings_from_FlightSteps(int minStepId, int maxStepId)
        {
            try
            {
                ResetBest();
                ResetTardis();

                if ((Process.Drone.FlightSteps != null) && (Process.ProcessObjects.Count > 0))
                {
                    // Get the FlightSteps corresponding to the block range
                    FlightStepList theSteps = new();
                    for (int stepId = minStepId; stepId <= maxStepId; stepId++)
                    {
                        FlightStep theStep;
                        if (Process.Drone.FlightSteps.Steps.TryGetValue(stepId, out theStep))
                            theSteps.AddStep(theStep);
                    }
                    if (theSteps.Count >= 4)
                    {
                        // Get the Objects that exist inside the block range (not overlapping the block range).
                        // We may be analyzing 30 minutes of video.
                        // Recall that objects are ordered by the BlockId of the first feature in the object.
                        // For speed, scan backwards through ProcessObjList
                        // until we find an object that ends before minBlockId.
                        var allObjs = Process.ProcessObjects;
                        ProcessObjList theObjs = new();
                        for (int objectId = allObjs.Last().Key; objectId >= 0; objectId--)
                        {
                            ProcessObject theObject;
                            if (allObjs.TryGetValue(objectId, out theObject))
                            {
                                var firstFeat = theObject.FirstFeature;
                                var lastFeat = theObject.LastRealFeature;
                                if ((firstFeat != null) && (lastFeat != null))
                                {
                                    var firstStepId = firstFeat.Block.FlightStepId;
                                    var lastRealStepId = lastFeat.Block.FlightStepId;

                                    if ((firstStepId >= minStepId) && (lastRealStepId <= maxStepId))
                                        // Object lies fully within the specified block range
                                        theObjs.AddObject(theObject);

                                    if (lastRealStepId < minStepId)
                                        break; // We've gone back far enough
                                }
                            }

                        }


                        CalculateSettings_FixValues(theSteps, theObjs);
                        SummariseSteps(theSteps);
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessSpan.CalculateSettings_from_FlightSteps", ex);
            }
        }


        public int PercentOverlapWithRunFromTo(Drone drone)
        {
            return FlightLeg.PercentOverlapWithRunFromTo(drone, MinStepId, MaxStepId);
        }
        public bool OverlapsRunFromTo(Drone drone)
        {
            return PercentOverlapWithRunFromTo(drone) >= FlightLegModel.MinOverlapPercent;
        }
    }


    // A list of ProcessSpan objects
    public class ProcessSpanList : SortedList<int, ProcessSpan>
    {
        public ProcessSpanList()
        {
        }


        public void AddSpan(ProcessSpan processSpan)
        {
            BaseConstants.Assert(processSpan.ProcessSpanId > 0, "ProcessSpanList.AddLeg: No Id");
            Add(processSpan.ProcessSpanId, processSpan);
        }


        // Return the index of the first and last ProcessSpan overlap of this leg with the RunVideoFromS / RunVideoToS range
        public (int, int) OverlappingSpansRange(Drone drone)
        {
            int firstSpanId = BaseConstants.UnknownValue;
            int lastSpanId = BaseConstants.UnknownValue;

            foreach (var span in this)
            {
                if (span.Value.OverlapsRunFromTo(drone))
                {
                    if (firstSpanId == BaseConstants.UnknownValue)
                        firstSpanId = span.Value.ProcessSpanId;
                    lastSpanId = span.Value.ProcessSpanId;
                }
                else
                {
                    if (firstSpanId != BaseConstants.UnknownValue)
                        break;
                }
            }

            return (firstSpanId, lastSpanId);
        }


        public void SetFixValuesAfterLoad(VideoModel videoData, Drone drone)
        {
            if (drone.FlightSteps == null)
                return;

            var steps = drone.FlightSteps.Steps;

            foreach (var theSpan in this)
                if (Math.Abs(theSpan.Value.BestFixAltM) + Math.Abs(theSpan.Value.BestFixYawDeg) + Math.Abs(theSpan.Value.BestFixPitchDeg) > 0)
                    // PQR TODO Should we use BestFixHFOV to set videoData.HFOVDeg?
                    for (int stepId = theSpan.Value.MinStepId; stepId <= theSpan.Value.MaxStepId; stepId++)
                    {
                        if (steps.TryGetValue(stepId, out var step))
                        {
                            step.FixAltM = theSpan.Value.BestFixAltM;
                            step.FixYawDeg = theSpan.Value.BestFixYawDeg;
                            step.FixPitchDeg = theSpan.Value.BestFixPitchDeg;
                            step.CalculateSettings_InputImageCenterDemDsm(videoData, drone.GroundData);
                        }
                    }
        }
    }
}
