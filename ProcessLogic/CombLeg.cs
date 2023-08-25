// Copyright SkyComb Limited 2023. All rights reserved. 
using MathNet.Numerics.LinearRegression;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // FlightLeg contains the best estimate of a drone flight leg from drone data.
    // CombLeg analyses CombObjects in that FlightLeg to refine/correct
    // the flight altitude data using FlightStep.FixAltitudeM.
    public class CombLeg : CombLegModel
    {
        // Parent process
        private CombProcessAll Process { get; }

        private FlightLeg FlightLeg { get; set; }


        public CombLeg(CombProcessAll process, FlightLeg flightLeg, List<string> settings = null) : base(settings)
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


        private void SetBest(CombObjList combObjs)
        {
            BestFixAltitudeM = FlightLeg.FixAltitudeM;
            BestSumLocnErrM = combObjs.SumLocationErrM;
            BestSumHeightErrM = combObjs.SumHeightErrM;
        }


        // Apply FixAltitudeM to the FlightSteps, CombFeatures & CombObjects in the leg
        public bool CalculateSettings_ApplyFixAltitudeM(VideoModel videoData, FlightStepList legSteps, CombObjList combObjs)
        {
            // The image associated with each leg step now covers a slightly different area
            legSteps.CalculateSettings_ApplyFixAltitudeM(videoData);

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
                SetBest(combObjs);
                return true;
            }

            return false;
        }


        // Analyse CombObjects in the FlightLeg by assuming the drone altitude is inaccurate.
        // Apply various "FixAltitudeM" trial values to the FlightSteps, CombFeatures & CombObjects in the leg.
        // For each trial, measure the sum of the object location errors.
        // Lock in the FlightLeg.FixAltitudeM value that reduces the error most.
        public void CalculateSettings(VideoModel videoData, Drone drone)
        {
            try
            {
                var legSteps = drone.FlightSteps.Steps.GetLegSteps(LegId);
                if (legSteps.Count == 0)
                    return;

                var combObjs = Process.CombObjs.CombObjList.GetSignificantLegObjects(LegId);
                NumSignificantObjects = combObjs.Count;
                if (NumSignificantObjects == 0)
                    BestFixAltitudeM = 0;
                else
                {
                    int maxTestAbsM = 8;
                    // If the OnGroundAt setting was not available then test a wider range.
                    if (!drone.FlightSteps.HasOnGroundAtFix)
                        maxTestAbsM = 15;

                    if (true)
                    {
                        // Proven method

                        // Calculate the initial object error location and height errors with no fix.
                        FlightLeg.FixAltitudeM = 0;
                        BestFixAltitudeM = 0;
                        BestSumLocnErrM = 9999;
                        BestSumHeightErrM = 9999;
                        CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs);
                        OrgSumLocnErrM = BestSumLocnErrM;
                        OrgSumHeightErrM = BestSumHeightErrM;

                        // Search upwards at +0.2m intervals. If maxTestAbsM == 5, do 24 evaluations 
                        for (FlightLeg.FixAltitudeM = 0.2f; FlightLeg.FixAltitudeM <= maxTestAbsM; FlightLeg.FixAltitudeM += 0.2f)
                            if (!CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs))
                                break;

                        // Search downwards at -0.2m intervals. If maxTestAbsM == 5, do 24 evaluations 
                        for (FlightLeg.FixAltitudeM = -0.2f; FlightLeg.FixAltitudeM >= -maxTestAbsM; FlightLeg.FixAltitudeM -= 0.2f)
                            if (!CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs))
                                break;

                        // Fine tune at 0.1m intervals. Costs 1 or 2 evaluations.
                        FlightLeg.FixAltitudeM = BestFixAltitudeM + 0.1f;
                        if (!CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs))
                        {
                            FlightLeg.FixAltitudeM = BestFixAltitudeM - 0.1f;
                            CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs);
                        }

                        // Lock in the best single value across the full leg
                        FlightLeg.FixAltitudeM = BestFixAltitudeM;
                        CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs);
                        Process.CombObjs.CombObjList.CalculateSettings(Process.CombObjs.CombObjList);
                    }
                    else
                    {
                        // Trail method. Works but not well enough to replace above code. Needs refining.

                        BestFixAltitudeM = 0;
                        BestSumLocnErrM = 9999;
                        BestSumHeightErrM = 9999;
                        OrgSumLocnErrM = 9999;
                        OrgSumHeightErrM = 9999;

                        // Pass 1: Build up sample points using the
                        // (computationally expensive) CalculateSettings_ApplyFixAltitudeM
                        const int numPoints = 16;
                        var x = new double[numPoints];
                        var y = new double[numPoints];
                        for (int i = 0; i < numPoints; i++)
                        {
                            FlightLeg.FixAltitudeM = i * 0.5f - 4; // Evaluate from -4 to +4
                            CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs);

                            x[i] = FlightLeg.FixAltitudeM;
                            y[i] = combObjs.SumLocationErrM;

                            if (FlightLeg.FixAltitudeM == 0)
                                OrgSumLocnErrM = combObjs.SumLocationErrM;

                            if (Math.Abs(y[i]) < BestSumLocnErrM)
                            {
                                BestSumLocnErrM = combObjs.SumLocationErrM;
                                BestFixAltitudeM = FlightLeg.FixAltitudeM;
                            }
                        }


                        // Pass 2: Try more values from -9m to +9m in 0.1m increments
                        // using the (computationally cheap) polynomial
                        var rslt = MathNet.Numerics.Fit.Polynomial(x, y, 5, DirectRegressionMethod.QR);
                        var poly = new MathNet.Numerics.Polynomial(rslt);
                        for (float fixAltM = -9; fixAltM <= +9; fixAltM += 0.1f)
                        {
                            var value = (float)Math.Abs(poly.Evaluate(fixAltM));
                            if (value < BestSumLocnErrM)
                            {
                                BestSumLocnErrM = value;
                                BestFixAltitudeM = fixAltM;
                            }
                        }

                        // Lock in the best single value across the full leg
                        FlightLeg.FixAltitudeM = BestFixAltitudeM;
                        CalculateSettings_ApplyFixAltitudeM(videoData, legSteps, combObjs);
                        SetBest(combObjs);
                        Process.CombObjs.CombObjList.CalculateSettings(Process.CombObjs.CombObjList);
                    }
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
                throw ThrowException("CombLeg.CalculateSettings", ex);
            }
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
