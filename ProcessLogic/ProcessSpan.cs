// Copyright SkyComb Limited 2024. All rights reserved. 
using MathNet.Numerics.LinearRegression;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // ProcessSpan relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpan analyses ProcessObjects to refine/correct the flight altitude data using FlightStep.FixAltM.
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


        // Apply FixAltM to theSteps and on to the ProcessObjects and their ProcessFeatures
        public bool CalculateSettings_ApplyFixAltM(float fixAltM, FlightStepList theSteps, ProcessObjList objs)
        {
            // The image associated with each leg step now covers a slightly different area
            // Recalculate InputImageCenter Dem and Dsm based on fixAltM
            theSteps.CalculateSettings_FixAltM(fixAltM, Process.VideoData, Process.Drone.GroundData);

            foreach (var theObject in objs)
            {
                var theObj = theObject.Value;
                // Copy the list of features claimed by the object
                var objectFeatures = theObj.ProcessFeatures.Clone();

                // Eliminate all object summary data.
                theObj.ResetMemberData();

                // Recalc each feature - which will have a slightly different location
                foreach (var theFeature in objectFeatures)
                {
                    var theFeat = theFeature.Value;
                    theFeat.ResetMemberData();
                    theFeat.CalculateSettings_LocationM_FlatGround(theObj.LastRealFeature);
                    theFeat.CalculateSettings_LocationM_HeightM_LineofSight(Process.GroundData);

                    theObj.ClaimFeature(theFeat);
                }
            }

            // Calculate the revised object-list location and height errors.
            objs.CalculateSettings(objs);

            // Do we see a drop in location error sum?
            // The location error assumes the objects are mostly stationary over the time they are observed.
            if (objs.SumLocationErrM + 0.02f < BestSumLocnErrM) // at least a 2cm improvement
            {
                SetBest(fixAltM, objs);
                return true;
            }

            return false;
        }


        // Analyse ProcessObjects in the selected steps by assuming the drone altitude is inaccurate.
        // Apply various "FixAltM" trial values to the FlightSteps, ProcessFeatures & ProcessObjects.
        // For each trial, measure the sum of the object location errors.
        // Lock in the legSteps.FixAltM value that reduces the error most.
        public void CalculateSettings_FixAltM(FlightStepList theSteps, ProcessObjList theObjs)
        {
            try
            {
                NumSignificantObjects = theObjs.Count;

                if((theSteps.Count == 0) || (NumSignificantObjects == 0))
                    return;

                int maxTestAbsM = 8;
                // If the OnGroundAt setting was not available then test a wider range.
                if (!Process.Drone.FlightSteps.HasOnGroundAtFix)
                    maxTestAbsM = 15;

                if (true)
                {
                    // Proven method

                    // Calculate the initial object error location and height errors with no fix.
                    var fixAltM = 0.0f;
                    CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs);
                    OrgSumLocnErrM = BestSumLocnErrM;
                    OrgSumHeightErrM = BestSumHeightErrM;

                    // Search upwards at +0.2m intervals. If maxTestAbsM == 5, do <= 24 calculations 
                    for (fixAltM = 0.2f; fixAltM <= maxTestAbsM; fixAltM += 0.2f)
                        if (!CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs))
                            break;

                    // Search downwards at -0.2m intervals. If maxTestAbsM == 5, do <= 24 calculations 
                    for (fixAltM = -0.2f; fixAltM >= -maxTestAbsM; fixAltM -= 0.2f)
                        if (!CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs))
                            break;

                    // Fine tune at 0.1m intervals. Costs 1 or 2 calculations.
                    fixAltM = BestFixAltM + 0.1f;
                    if (!CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs))
                    {
                        fixAltM = BestFixAltM - 0.1f;
                        CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs);
                    }

                    // Lock in the best single value across the leg steps
                    fixAltM = BestFixAltM;
                    CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs);
                    Process.ProcessObjects.CalculateSettings(Process.ProcessObjects);
                }
                else
                {
                    // Trial method. Works but not well enough to replace above code. Needs refining.

                    var fixAltM = 0.0f;
                    OrgSumLocnErrM = 9999;
                    OrgSumHeightErrM = 9999;

                    // Pass 1: Build up sample points using the
                    // (computationally expensive) CalculateSettings_ApplyFixAltM
                    const int numPoints = 16;
                    var x = new double[numPoints];
                    var y = new double[numPoints];
                    for (int i = 0; i < numPoints; i++)
                    {
                        fixAltM = i * 0.5f - 4; // Evaluate from -4 to +4
                        CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs);

                        x[i] = fixAltM;
                        y[i] = theObjs.SumLocationErrM;

                        if (fixAltM == 0)
                            OrgSumLocnErrM = theObjs.SumLocationErrM;

                        if (Math.Abs(y[i]) < BestSumLocnErrM)
                        {
                            BestSumLocnErrM = theObjs.SumLocationErrM;
                            BestFixAltM = fixAltM;
                        }
                    }


                    // Pass 2: Try more values from -9m to +9m in 0.1m increments
                    // using the (computationally cheap) polynomial
                    var rslt = MathNet.Numerics.Fit.Polynomial(x, y, 5, DirectRegressionMethod.QR);
                    var poly = new MathNet.Numerics.Polynomial(rslt);
                    for (fixAltM = -9; fixAltM <= +9; fixAltM += 0.1f)
                    {
                        var value = (float)Math.Abs(poly.Evaluate(fixAltM));
                        if (value < BestSumLocnErrM)
                        {
                            BestSumLocnErrM = value;
                            BestFixAltM = fixAltM;
                        }
                    }

                    // Lock in the best single value across the full leg
                    fixAltM = BestFixAltM;
                    CalculateSettings_ApplyFixAltM(fixAltM, theSteps, theObjs);
                    SetBest(fixAltM, theObjs);
                    Process.ProcessObjects.CalculateSettings(Process.ProcessObjects);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessSpan.CalculateSettings_Core", ex);
            }
        }


        // Summarise the blocks in this leg
        private void SummariseSteps(FlightStepList steps)
        {
            if(steps.Count > 0)
            {
                MinStepId = steps.First().Value.StepId;
                MaxStepId = steps.Last().Value.StepId;
            }

            ResetTardis();
            foreach(var step in steps)
                SummariseTardis(step.Value);
        }


        // Analyse ProcessObjects in the FlightLeg - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM value that reduces the location most.
        public void CalculateSettings_from_FlightLeg()
        {
            ResetBest();
            ResetTardis();

            if (ProcessSpanId == UnknownValue)
                return;

            var legSteps = Process.Drone.FlightSteps.Steps.GetLegSteps(ProcessSpanId);
            var theObjs = Process.ProcessObjects.FilterByLeg(ProcessSpanId);

            CalculateSettings_FixAltM(legSteps, theObjs);
            SummariseSteps(legSteps);
        }


        // Analyse ProcessObjects in the Block series - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM value that reduces the location most.
        public void CalculateSettings_from_FlightSteps(int minStepId, int maxStepId)
        {
            try { 
                ResetBest();
                ResetTardis();

                if((Process.Drone.FlightSteps != null) && (Process.ProcessObjects.Count > 0))
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


                        CalculateSettings_FixAltM(theSteps, theObjs);
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


        public void SetFixAltMAfterLoad(VideoModel videoData, Drone drone)
        {
            if( drone.FlightSteps == null )
                return;

            var steps = drone.FlightSteps.Steps;

            foreach (var theSpan in this)
                if( theSpan.Value.BestFixAltM != 0 )
                    for(int stepId = theSpan.Value.MinStepId; stepId <= theSpan.Value.MaxStepId; stepId++)
                    {
                        if( steps.TryGetValue(stepId, out var step) )
                        {
                            step.FixAltM = theSpan.Value.BestFixAltM;
                            step.CalculateSettings_InputImageCenterDemDsm(videoData, drone.GroundData);
                        }
                    }
        }
    }
}
