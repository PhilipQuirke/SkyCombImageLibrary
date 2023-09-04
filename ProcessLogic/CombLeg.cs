// Copyright SkyComb Limited 2023. All rights reserved. 
using MathNet.Numerics.LinearRegression;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // CombLeg analyses CombObjects in a leg (either FLightLeg or transatory) 
    // to refine/correct the flight altitude data using FlightStep.FixAltM.
    public class CombLeg : CombLegModel
    {
        // Parent process
        private CombProcessAll Process { get; }

        public FlightLeg? FlightLeg { get; set; }


        public CombLeg(CombProcessAll process, FlightLeg? flightLeg = null, List<string>? settings = null) : base(settings)
        {
            Process = process;
            FlightLeg = flightLeg;
            LegId = (flightLeg != null ? flightLeg.LegId : UnknownValue);

            if (settings != null)
                LoadSettings(settings);
        }


        public void AssertGood()
        {
            Assert(LegId > 0, "CombLeg.AssertGood: Bad legId");
            Assert(MinBlockId > 0, "CombLeg.AssertGood: Bad MinBlockId");
            Assert(MaxBlockId > 0, "CombLeg.AssertGood: Bad MaxBlockId");
        }


        private void ResetBest()
        {
            BestFixAltM = 0;
            BestSumLocnErrM = 9999;
            BestSumHeightErrM = 9999;
            OrgSumLocnErrM = 9999;
            OrgSumHeightErrM = 9999;
        }


        private void SetBest(float fixAltM, CombObjList combObjs)
        {
            BestFixAltM = fixAltM;
            BestSumLocnErrM = combObjs.SumLocationErrM;
            BestSumHeightErrM = combObjs.SumHeightErrM;
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
                    theFeature.Value.CalculateSettings_LocationM(false);
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

                // Summarise the blocks in this leg
                ResetTardis();
                foreach (var theBlock in Process.Blocks)
                    if (theBlock.Value.LegId == LegId)
                    {
                        if (theBlock.Value.FlightStep != null)
                        {
                            if (MinStepId == UnknownValue)
                                MinStepId = theBlock.Value.FlightStep.StepId;
                            MaxStepId = Math.Max(MaxStepId, theBlock.Value.FlightStep.StepId);
                        }
                        SummariseTardis(theBlock.Value);
                    }
            }
            catch (Exception ex)
            {
                throw ThrowException("CombLeg.CalculateSettings_Core", ex);
            }
        }


        // Summarise the blocks in this leg
        private void SummariseBlocks()
        {
            ResetTardis();
            foreach (var theBlock in Process.Blocks)
                if (theBlock.Value.LegId == LegId)
                {
                    if (theBlock.Value.FlightStep != null)
                    {
                        if (MinStepId == UnknownValue)
                            MinStepId = theBlock.Value.FlightStep.StepId;
                        MaxStepId = Math.Max(MaxStepId, theBlock.Value.FlightStep.StepId);
                    }
                    SummariseTardis(theBlock.Value);
                }
        }


        // Analyse CombObjects in the FlightLeg - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM value that reduces the location most.
        public void CalculateSettings_from_FlightLeg()
        {
            ResetBest();

            if (LegId == UnknownValue)
                return;

            var legSteps = Process.Drone.FlightSteps.Steps.GetLegSteps(LegId);
            var combObjs = Process.CombObjs.CombObjList.GetSignificantLegObjects(LegId);

            CalculateSettings_FixAltM(legSteps, combObjs);
            SummariseBlocks();
        }


        // Analyse CombObjects in the Block series - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM value that reduces the location most.
        public void CalculateSettings_from_Blocks(int minBlockId, int maxBlockId)
        {
            ResetBest();

            if (Process.CombObjs.CombObjList.Count > 0)
            {
                // Get the FlightSteps corresponding to the block range
                FlightStepList legSteps = new();
                for (int blockId = minBlockId; blockId <= maxBlockId; blockId++)
                {
                    ProcessBlock theBlock;
                    if (Process.Blocks.TryGetValue(blockId, out theBlock) && (theBlock.FlightStep != null))
                        legSteps.Add(theBlock.FlightStep.StepId, theBlock.FlightStep);
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
                                var firstBlockId = firstFeat.Block.BlockId;
                                var lastRealBlockId = lastFeat.Block.BlockId;

                                if ((firstBlockId >= minBlockId) && (lastRealBlockId <= minBlockId))
                                    // Object lies fully within the specified block range
                                    combObjs.AddObject(theObject);

                                if (lastRealBlockId < minBlockId)
                                    break; // We've gone back far enough
                            }
                        }

                    }


                    CalculateSettings_FixAltM(legSteps, combObjs);
                }
            }

            SummariseBlocks();
        }
    }


    // A list of CombLeg objects
    public class CombLegList : SortedList<int, CombLeg>
    {
        public CombLegList()
        {
        }


        public void Add(CombLeg combLeg)
        {
            Add(combLeg.LegId, combLeg);
        }
    }
}
