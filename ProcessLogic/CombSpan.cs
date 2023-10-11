// Copyright SkyComb Limited 2023. All rights reserved. 
using MathNet.Numerics.LinearRegression;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // CombSpan relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, CombSpan analyses CombObjects to refine/correct the flight altitude data using FlightStep.FixAltM.
    public class CombSpan : CombSpanModel
    {
        // Parent process
        private CombProcessAll Process { get; }


        public CombSpan(CombProcessAll process, int spanId, List<string>? settings = null) : base(settings)
        {
            Process = process;
            CombSpanId = spanId;

            if (settings != null)
                LoadSettings(settings);
        }


        public void AssertGood()
        {
            Assert(CombSpanId > 0, "CombSpan.AssertGood: Bad SpanId");
            Assert(MinBlockId > 0, "CombSpan.AssertGood: Bad MinBlockId");
            Assert(MaxBlockId > 0, "CombSpan.AssertGood: Bad MaxBlockId");
        }


        // Apply FixAltM to the specified FlightSteps and CombObjects and their CombFeatures
        public bool CalculateSettings_ApplyFixAltM(float fixAltM, FlightStepList legSteps, CombObjList combObjs)
        {
            // The image associated with each leg step now covers a slightly different area
            legSteps.CalculateSettings_FixAltM(fixAltM, Process.VideoData, Process.Drone.GroundData);

            foreach (var theObject in combObjs)
            {
                // Copy the list of features claimed by the object
                var objectFeatures = theObject.Value.Features.Clone();

                theObject.Value.ResetMemberData();

                // Each feature now has a slightly different location
                foreach (var theFeature in objectFeatures)
                {
                    theFeature.Value.ResetMemberData();
                    theFeature.Value.CalculateSettings_LocationM_FlatGround(theObject.Value.LastRealFeature(), false);
                    theFeature.Value.CalculateSettings_LocationM_HeightM_LineofSight();
                    theObject.Value.ClaimFeature(theFeature.Value, false);
                }
            }

            // Calculate the revised object-list location and height errors.
            combObjs.CalculateSettings(combObjs);

            // Do we see a significant drop in location error sum?
            // The location error assumes the objects are mostly stationary over the time they are observed.
            if (combObjs.SumLocationErrM + 0.02f < BestSumLocnErrM) // at least a 2cm improvement
            {
                SetBest(fixAltM, combObjs);
                return true;
            }

            return false;
        }


        // Analyse CombObjects in the selected steps by assuming the drone altitude is inaccurate.
        // Apply various "FixAltM" trial values to the FlightSteps, CombFeatures & CombObjects.
        // For each trial, measure the sum of the object location errors.
        // Lock in the legSteps.FixAltM value that reduces the error most.
        public void CalculateSettings_FixAltM(FlightStepList legSteps, CombObjList combObjs)
        {
            try
            {
                NumSignificantObjects = combObjs.Count;

                // Testing
                // return;


                if((legSteps.Count == 0) || (NumSignificantObjects == 0))
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
                    CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs);
                    OrgSumLocnErrM = BestSumLocnErrM;
                    OrgSumHeightErrM = BestSumHeightErrM;

                    // Search upwards at +0.2m intervals. If maxTestAbsM == 5, do 24 evaluations 
                    for (fixAltM = 0.2f; fixAltM <= maxTestAbsM; fixAltM += 0.2f)
                        if (!CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs))
                            break;

                    // Search downwards at -0.2m intervals. If maxTestAbsM == 5, do 24 evaluations 
                    for (fixAltM = -0.2f; fixAltM >= -maxTestAbsM; fixAltM -= 0.2f)
                        if (!CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs))
                            break;

                    // Fine tune at 0.1m intervals. Costs 1 or 2 evaluations.
                    fixAltM = BestFixAltM + 0.1f;
                    if (!CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs))
                    {
                        fixAltM = BestFixAltM - 0.1f;
                        CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs);
                    }

                    // Lock in the best single value across the leg steps
                    fixAltM = BestFixAltM;
                    CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs);
                    Process.CombObjs.CombObjList.CalculateSettings(Process.CombObjs.CombObjList);
                }
                else
                {
                    // Trail method. Works but not well enough to replace above code. Needs refining.

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
                        CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs);

                        x[i] = fixAltM;
                        y[i] = combObjs.SumLocationErrM;

                        if (fixAltM == 0)
                            OrgSumLocnErrM = combObjs.SumLocationErrM;

                        if (Math.Abs(y[i]) < BestSumLocnErrM)
                        {
                            BestSumLocnErrM = combObjs.SumLocationErrM;
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
                    CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs);
                    SetBest(fixAltM, combObjs);
                    Process.CombObjs.CombObjList.CalculateSettings(Process.CombObjs.CombObjList);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("CombSpan.CalculateSettings_Core", ex);
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


        // Analyse CombObjects in the FlightLeg - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM value that reduces the location most.
        public void CalculateSettings_from_FlightLeg()
        {
            ResetBest();
            ResetTardis();

            if (CombSpanId == UnknownValue)
                return;

            var legSteps = Process.Drone.FlightSteps.Steps.GetLegSteps(CombSpanId);
            var combObjs = Process.CombObjs.CombObjList.FilterByLeg(CombSpanId);

            CalculateSettings_FixAltM(legSteps, combObjs);
            SummariseSteps(legSteps);
        }


        // Analyse CombObjects in the Block series - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM value that reduces the location most.
        public void CalculateSettings_from_FlightSteps(int minStepId, int maxStepId)
        {
            try { 
                ResetBest();
                ResetTardis();

                if((Process.Drone.FlightSteps != null) && (Process.CombObjs.CombObjList.Count > 0))
                {
                    // Get the FlightSteps corresponding to the block range
                    FlightStepList legSteps = new();
                    for (int stepId = minStepId; stepId <= maxStepId; stepId++)
                    {
                        FlightStep theStep;
                        if (Process.Drone.FlightSteps.Steps.TryGetValue(stepId, out theStep))
                            legSteps.AddStep(theStep);
                    }
                    if (legSteps.Count >= 4)
                    {
                        // Get the Objects that exist inside the block range (not overlapping the block range).
                        // We may be analyzing 30 minutes of video.
                        // Recall that objects are ordered by the BlockId of the first feature in the object.
                        // For speed, scan backwards through CombObjList
                        // until we find an object that ends before minBlockId.
                        var allObjs = Process.CombObjs.CombObjList;
                        CombObjList combObjs = new();
                        for (int objectId = allObjs.Last().Key; objectId >= 0; objectId--)
                        {
                            CombObject theObject;
                            if (allObjs.TryGetValue(objectId, out theObject))
                            {
                                var firstFeat = theObject.FirstFeature();
                                var lastFeat = theObject.LastRealFeature();
                                if ((firstFeat != null) && (lastFeat != null))
                                {
                                    var firstStepId = firstFeat.Block.FlightStepId;
                                    var lastRealStepId = lastFeat.Block.FlightStepId;

                                    if ((firstStepId >= minStepId) && (lastRealStepId <= maxStepId))
                                        // Object lies fully within the specified block range
                                        combObjs.AddObject(theObject);

                                    if (lastRealStepId < minStepId)
                                        break; // We've gone back far enough
                                }
                            }

                        }


                        CalculateSettings_FixAltM(legSteps, combObjs);
                        SummariseSteps(legSteps);
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("CombSpan.CalculateSettings_from_FlightSteps", ex);
            }
        }
    }


    // A list of CombSpan objects
    public class CombSpanList : SortedList<int, CombSpan>
    {
        public CombSpanList()
        {
        }


        public void AddSpan(CombSpan combSpan)
        {
            BaseConstants.Assert(combSpan.CombSpanId > 0, "CombSpanList.AddLeg: No Id");
            Add(combSpan.CombSpanId, combSpan);
        }


        public void SetFixAltMAfterLoad(VideoModel videoData, Drone drone)
        {
            if( drone.FlightSteps == null )
                return;

            var steps = drone.FlightSteps.Steps;

            foreach (var combSpan in this)
                if( combSpan.Value.BestFixAltM != 0 )
                    for(int stepId = combSpan.Value.MinStepId; stepId <= combSpan.Value.MaxStepId; stepId++)
                    {
                        if( steps.TryGetValue(stepId, out var step) )
                        {
                            step.FixAltM = combSpan.Value.BestFixAltM;
                            step.CalculateSettings_InputImageCenterDemDsm(videoData, drone.GroundData);
                        }
                    }
        }
    }
}
