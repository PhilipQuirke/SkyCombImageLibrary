// Copyright SkyComb Limited 2023. All rights reserved. 
using MathNet.Numerics.LinearRegression;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // FlightLeg contains the best estimate of a drone flight leg from drone data.
    // CombLeg analyses CombObjects in that FlightLeg to refine/correct
    // the flight altitude data using FlightStep.FixAltM.
    public class CombLeg : CombLegModel
    {
        // Parent process
        private CombProcessAll Process { get; }

        private FlightLeg FlightLeg { get; set; }


        public CombLeg(CombProcessAll process, FlightLeg flightLeg, List<string>? settings = null) : base(settings)
        {
            Process = process;
            LegId = flightLeg.LegId;
            FlightLeg = flightLeg;

            if (settings != null)
                LoadSettings(settings);
        }


        public void AssertGood()
        {
            Assert(LegId > 0, "CombLeg.AssertGood: Bad legId");
            Assert(MinBlockId > 0, "CombLeg.AssertGood: Bad MinBlockId");
            Assert(MaxBlockId > 0, "CombLeg.AssertGood: Bad MaxBlockId");
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
            // Save the new fixAltM to all the steps in scope.
            legSteps.SetFixAltM(fixAltM);

            // The image associated with each leg step now covers a slightly different area
            legSteps.CalculateSettings_ApplyFixAltM(Process.VideoData, Process.Drone.GroundData);

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
        public void CalculateSettings_Core(FlightStepList legSteps, CombObjList combObjs)
        {
            try
            {
                BestFixAltM = 0;
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
                    BestFixAltM = 0;
                    BestSumLocnErrM = 9999;
                    BestSumHeightErrM = 9999;
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

                    // Lock in the best single value across the full leg
                    fixAltM = BestFixAltM;
                    CalculateSettings_ApplyFixAltM(fixAltM, legSteps, combObjs);
                    Process.CombObjs.CombObjList.CalculateSettings(Process.CombObjs.CombObjList);
                }
                else
                {
                    // Trail method. Works but not well enough to replace above code. Needs refining.

                    var fixAltM = 0.0f;
                    BestFixAltM = 0;
                    BestSumLocnErrM = 9999;
                    BestSumHeightErrM = 9999;
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


        // Analyse CombObjects in the FlightLeg by assuming the drone altitude is inaccurate.
        // Apply various "FixAltM" trial values to the FlightSteps, CombFeatures & CombObjects in the leg.
        // For each trial, measure the sum of the object location errors.
        // Lock in the FlightSteps.FixAltM value that reduces the error most.
        public void CalculateSettings_from_FlightLeg()
        {
            var legSteps = Process.Drone.FlightSteps.Steps.GetLegSteps(LegId);
            var combObjs = Process.CombObjs.CombObjList.GetSignificantLegObjects(LegId);

            CalculateSettings_Core(legSteps, combObjs);
        }
    }


    // A list of Comb objects
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
