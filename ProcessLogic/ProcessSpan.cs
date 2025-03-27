// Copyright SkyComb Limited 2025. All rights reserved. 
using OpenCvSharp;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Diagnostics;
using System.Drawing;


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


        public void DebugPrintBest(string prefix = "")
        {
            var answer = "";

            if (BestFixAltM != 0) answer += " BestFixAltM=" + BestFixAltM.ToString();
            if (BestFixYawDeg != 0) answer += " BestFixYawDeg=" + BestFixYawDeg.ToString();
            if (BestFixPitchDeg != 0) answer += " BestFixPitchDeg=" + BestFixPitchDeg.ToString();

            if (answer != "")
                Debug.Print(
                    "CalculateSettings_FixValues:" + prefix + " " + answer + 
                    " BestSumLocnErrM=" + BestSumLocnErrM.ToString() +
                    " BestSumHeightErrM=" + BestSumHeightErrM.ToString());
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


        // Apply FixValues to theSteps and on to the ProcessObjects and their ProcessFeatures //PQ??
        public bool CalculateSettings_ApplyFixValues(int hFOVDeg, float fixAltM, float fixYawDeg, float fixPitchDeg, FlightStepList theSteps, ProcessObjList objs)
        {
            CalculateSettings_ApplyFixValues_Core(hFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, theSteps, objs);

            // Do we see a drop in location error sum?
            // The location error assumes the objects are mostly stationary over the time they are observed.
            var improvementM = BestSumLocnErrM - objs.SumLocationErrM;
            if (improvementM >= 0.1f) // at least a 10cm improvement
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
            bool optimiseHFOVDeg = false;
            bool optimiseFixAltM = true;
            bool optimiseFixYawDeg = false;
            bool optimiseFixPitchDeg = false;

            try
            {
                Debug.Print("CalculateSettings_FixAltM_Start");
                NumSignificantObjects = theObjs.Count;

                if ((theSteps.Count == 0) || (NumSignificantObjects == 0))
                    return;

                float altRangeM = 90;
                float yawRangeDeg = 10;
                float pitchRangeDeg = 20;

                int theHFOVDeg = Process.Drone.InputVideo.HFOVDeg;
                float bestAltM = 0;
                float bestAltErr = 0;
                float bestYawDeg = 0;
                float bestYawErr = 0;
                float bestPitchDeg = 0;
                float bestPitchErr = 0;

                ResetBest();
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, theSteps, theObjs);
                OrgSumLocnErrM = BestSumLocnErrM;
                OrgSumHeightErrM = BestSumHeightErrM;

                if (true)
                {
                    // DIMENSION 1: Video HFOVDeg
                    // The drone.InputVideo.HFOVDeg value is guessed from attributes of the SRT file. This is weak.
                    // Known hardware values for HFOVDeg are 38, 42 & 57. Test reasonable values around these.
                    if (optimiseHFOVDeg)
                    {
                        CalculateSettings_ApplyFixValues(36, 0, 0, 0, theSteps, theObjs);
                        CalculateSettings_ApplyFixValues(38, 0, 0, 0, theSteps, theObjs);
                        CalculateSettings_ApplyFixValues(40, 0, 0, 0, theSteps, theObjs);
                        CalculateSettings_ApplyFixValues(42, 0, 0, 0, theSteps, theObjs);
                        CalculateSettings_ApplyFixValues(44, 0, 0, 0, theSteps, theObjs);
                        CalculateSettings_ApplyFixValues(57, 0, 0, 0, theSteps, theObjs);
                        theHFOVDeg = BestHFOVDeg;
                    }

                    // DIMENSION 2: Drone altitude 
                    if (optimiseFixAltM)
                    {
                        ResetBest();
                        for (float fixAltM = altRangeM; fixAltM >= -altRangeM; fixAltM -= 1)
                            if( CalculateSettings_ApplyFixValues(theHFOVDeg, fixAltM, 0, 0, theSteps, theObjs) )
                                DebugPrintBest();
                        bestAltM = BestFixAltM;
                        bestAltErr = BestSumLocnErrM;
                    }

                    // DIMENSION 3: Drone yaw 
                    if (optimiseFixYawDeg)
                    {
                        ResetBest();
                        for (float fixYawDeg = -yawRangeDeg; fixYawDeg <= yawRangeDeg; fixYawDeg += 1)
                            CalculateSettings_ApplyFixValues(theHFOVDeg, 0, fixYawDeg, 0, theSteps, theObjs);
                        bestYawDeg = BestFixYawDeg;
                        bestYawErr = BestSumLocnErrM;
                    }

                    // DIMENSION 4: Drone pitch 
                    if (optimiseFixPitchDeg)
                    {
                        ResetBest();
                        for (float fixPitchDeg = -pitchRangeDeg; fixPitchDeg <= pitchRangeDeg; fixPitchDeg += 1)
                            CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, fixPitchDeg, theSteps, theObjs);
                        bestPitchDeg = BestFixPitchDeg;
                        bestPitchErr = BestSumLocnErrM;
                    }

                    // DIMENSION 2..4: Fine tune
                    // Vary dimensions 2 to 4 around the above rough values
                    if (optimiseFixAltM && optimiseFixYawDeg && optimiseFixPitchDeg)
                    {
                        ResetBest();
                        CalculateSettings_ApplyFixValues(theHFOVDeg, bestAltM, bestYawDeg, bestPitchDeg, theSteps, theObjs);
                        for (float fixAltM = bestAltM - 1.25f; fixAltM <= bestAltM + 1.25f; fixAltM += 0.25f)
                            for (float fixYawDeg = bestYawDeg - 1.25f; fixYawDeg <= bestYawDeg + 1.25f; fixYawDeg += 0.25f)
                                for (float fixPitchDeg = bestPitchDeg - 1.25f; fixPitchDeg <= bestPitchDeg + 1.25f; fixPitchDeg += 0.25f)
                                    CalculateSettings_ApplyFixValues(theHFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, theSteps, theObjs);
                    }
                }

                // Lock in the best single value across the leg steps
                CalculateSettings_ApplyFixValues_Core(theHFOVDeg, BestFixAltM, BestFixYawDeg, BestFixPitchDeg, theSteps, theObjs);
                BestSumLocnErrM = theObjs.SumLocationErrM;
                BestSumHeightErrM = theObjs.SumHeightErrM;
                Process.ProcessObjects.CalculateSettings();

                DebugPrintBest();
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

            if (true)
            {
                // nq new method
                TriangulateSpanObjectsFeaturesLocationAndHeight(theObjs, Process);
                foreach (var theObj in theObjs)
                    theObj.Value.Calculate_RealObject_SimpleMemberData();
                theObjs.CalculateSettings();
            }
            else
            {
                // Old pq optimization method
                // For 2164 video on Leg A a Best Fix Alt of 19m gave a 13cm reduction in avg locn err. Not worth it.
                //CalculateSettings_FixValues(legSteps, theObjs);

                // No optimization method
                int theHFOVDeg = Process.Drone.InputVideo.HFOVDeg;
                ResetBest();
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs);
                OrgSumLocnErrM = BestSumLocnErrM;
                OrgSumHeightErrM = BestSumHeightErrM;

                /*
                // Test if DroneTargetCalculator.DroneK contains bad data.
                // Default DroneK is Intrinsic(9.1, 640, 512, 7.68, 6.144);
                Debug.Print("Default DroneK: OrgSumLocnErrM=" + OrgSumLocnErrM + " OrgSumHeightErrM=" + OrgSumHeightErrM );

                //Modifying x and y only has 3% impact
                //for (int x = -20; x <= +20; x += 5)
                //    for (int y = -20; y <= +20; y += 5)
                //    {
                //        DroneTargetCalculator.DroneK = DroneTargetCalculator.Intrinsic(9.1, 640 + x, 512 + y, 7.68, 6.144);
                //        if (CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs))
                //            Debug.Print("x=" + x + ", y=" + y + ", BestSumLocnErrM=" + BestSumLocnErrM  + " BestSumHeightErrM=" + BestSumHeightErrM );
                //    }

                // At f = 4.95, has 50% reduction (OrgSumLocnErrM=286, BestSumLocnErrM=149) but tightens objects to the flight line. BAD?
                //for (float f = 0.9f; f <= 5; f += 0.05f)
                //{
                //    DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1 * f, 640, 512, 7.68, 6.144);
                //    if (CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs))
                //        Debug.Print("f=" + f + ", BestSumLocnErrM=" + BestSumLocnErrM + " BestSumHeightErrM=" + BestSumHeightErrM );
                //}

                // Modifying w has no impact
                //for (float w = 0.2f; w >= 0.01f; w -= 0.01f)
                //{
                //    DroneTargetCalculator.DroneK = DroneTargetCalculator.Intrinsic(9.1, 640, 512, 7.68 * w, 6.144);
                //    if (CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs))
                //        Debug.Print("w=" + w + " BestSumLocnErrM=" + BestSumLocnErrM  + " BestSumHeightErrM=" + BestSumHeightErrM );
                //}

                // At h = 0.02 (98% reduction), has 60% reduction in location inaccuracy: OrgSumLocnErrM=286, BestSumLocnErrM=119
                // Does not appear to shift objects towards flightpath. GOOD
                for (float h = 0.2f; h >= 0.01f; h -= 0.01f)
                {
                    DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * h);
                    if (CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs))
                        Debug.Print("h=" + h + " BestSumLocnErrM=" + BestSumLocnErrM + " BestSumHeightErrM=" + BestSumHeightErrM);
                }

                //Debug.Print("Default DroneK: OrgSumLocnErrM=" + OrgSumLocnErrM + " OrgSumHeightErrM=" + OrgSumHeightErrM);
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1 * 4.95, 640, 512, 7.68, 6.144);  // BAD. At f = 4.95, has 50% reduction. tightens objects to flight pathight line
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1 * 4.95, 640, 512, 7.68 * 0.02, 6.144 * 0.02); // BAD. Pinned to line
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68 * 0.02, 6.144 * 0.02); // BAD. Pinned to line

                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 1.0); // h=1.0 => 286/286
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.8); // h=0.8 => 251/286
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.4); // h=0.4 => 182/286 
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.3); // h=0.3 => 166/286  
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.2); // h=0.2 => 149/286 
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.15); // h=0.15 => 140/286 
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.1); // h=0.1 => 132/286  
                DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.05); // h=0.05 => 124/286  
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.02); // h=0.02 => 119/286 
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.01); // h=0.01 => 117/286  
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.001); // h=0.001 => 116/286  
                //DroneTargetCalculatorV2.DroneK = DroneTargetCalculatorV2.Intrinsic(9.1, 640, 512, 7.68, 6.144 * 0.0001); // h=0.0001 => 116/286  
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs);
                Debug.Print("BestSumLocnErrM=" + BestSumLocnErrM + " BestSumHeightErrM=" + BestSumHeightErrM);
                */
            }

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

        // utility
        public void printMat(Mat mat, string desc)
        {
            Debug.WriteLine(desc);
            for (int i = 0; i < mat.Rows; i++)
            {
                for (int j = 0; j < mat.Cols; j++)
                {
                    Debug.Write(mat.At<double>(i, j).ToString() + ",");
                }
                Debug.WriteLine("");
            }
        }
        
        //VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        // Copy the static Lennard Spark drone camera intrinsic matrix
        public static Mat K = ProcessConfigModel.LennardsDroneK;
        public Mat K = Intrinsic(9.1, 640, 512, 7.68, 6.144);


        //           Fix and test with non zero animal
        // Recalculate the Span.Objects.Features.LocationM and HeightM using triangulation.
        public void TriangulateSpanObjectsFeaturesLocationAndHeight(ProcessObjList theObjs, ProcessAll processAll)
        {
            var compareInterval = 5; // Frame pair intervals constant, 3 frames is 1/20th of second. 5 frames is 1/6th of a second.

            var totBlocks = Process.Blocks.Count;
            foreach (var block in Process.Blocks.Values)
            {
                var blockId = block.BlockId;
                var compareBlockId = blockId + compareInterval; // NOTE this depends on blockId as being sequential !!!!!!!
                if (compareBlockId > totBlocks) break; // not enough compare interval left
                var compareBlock = Process.Blocks[compareBlockId];
                if (block.MinFeatureId < 1 || compareBlock.MinFeatureId < 1) continue; // no features in either this or compare
                if (block.DroneLocnM.EastingM == compareBlock.DroneLocnM.EastingM
                    && block.DroneLocnM.NorthingM == compareBlock.DroneLocnM.NorthingM) continue; // not enough drone location difference for the compare
                var BlocksInfo = new BlockInfo();

                for (var id = block.MinFeatureId; id <= block.MaxFeatureId; id++)
                {
                    var feature = Process.ProcessFeatures[id];
                    if (!feature.Significant) continue;
                    if ((feature.PixelBox.X <= 1) || (feature.PixelBox.X + feature.PixelBox.Width >= 1280) || (feature.PixelBox.Y <= 1) || (feature.PixelBox.Y + feature.PixelBox.Height >= 1024)) continue; //too close to edge
                    var objid = feature.ObjectId;
                    for (var idC = compareBlock.MinFeatureId; idC <= compareBlock.MaxFeatureId; idC++)
                    {
                        var featureC = Process.ProcessFeatures[idC];
                        if ((featureC.PixelBox.X <= 1) || (featureC.PixelBox.X + featureC.PixelBox.Width >= 1280) || (featureC.PixelBox.Y <= 1) || (featureC.PixelBox.Y + featureC.PixelBox.Height >= 1024)) continue; //too close to edge
                        var objidC = featureC.ObjectId;
                        if (objid != objidC) continue;
                        BlocksInfo.fromObs.Add(objid);
                        BlocksInfo.fromFeatures.Add(objid, feature);
                        BlocksInfo.toFeatures.Add(objid, featureC);

                        // Updating for new stuff 12/3
                        var result = BlocksInfo.GetPoint(feature, featureC, K);
                        if (result is not null)
                        {
                            feature.LocationM = new DroneLocation((float)result[1], (float)result[0]); // Northing is stored first in location
                            // Change from altitude to height based on ground altitude at location
                            var obsDEM = processAll.GroundData.DemModel.GetElevationByDroneLocn(feature.LocationM);
                            Assert(obsDEM != UnknownValue, objid.ToString() + " " + feature.FeatureId.ToString() + " object/feature location out of bounds");
                            feature.HeightM = (float)result[2]- obsDEM;
                        }
                    }
                }
                /*if (BlocksInfo.fromObs.Count == 0) continue;
                Debug.WriteLine("+++++++++++++++++++++++++");
                Debug.WriteLine(blockId.ToString());
                Debug.WriteLine(block.DroneLocnM.EastingM.ToString() + "," + block.DroneLocnM.NorthingM.ToString() + "," + block.AltitudeM.ToString() + "," + block.RollDeg.ToString() + "," + block.PitchDeg.ToString() + "," + block.YawDeg);
*/                
                using var Points1 = ToTriangulationFormat(BlocksInfo.CreatePoints(true));
                using var Points2 = ToTriangulationFormat(BlocksInfo.CreatePoints(false));
                using var Projection1 = BlocksInfo.CreateProjectionMatrix(block.DroneLocnM.EastingM, block.DroneLocnM.NorthingM, block.AltitudeM, block.RollDeg, block.PitchDeg, block.YawDeg, K);
                using var Projection2 = BlocksInfo.CreateProjectionMatrix(compareBlock.DroneLocnM.EastingM, compareBlock.DroneLocnM.NorthingM, compareBlock.AltitudeM, compareBlock.RollDeg, compareBlock.PitchDeg, compareBlock.YawDeg, K);
                using Mat homogeneousPoints = new Mat();
/*
                printMat(Projection1, "Proj1");
                printMat(Points1, "Points1");
                printMat(Projection2, "Proj2");
                printMat(Points2, "Points2"); 
*/
                Cv2.TriangulatePoints(Projection1, Projection2, Points1, Points2, homogeneousPoints);
                // Convert homogeneous coordinates to 3D and update the locations into YoloProcessFeature
                int counter = 0;
                foreach (var obs in BlocksInfo.fromObs)
                {
                    var thisfeature = BlocksInfo.fromFeatures[obs];
                    thisfeature.realLocation = [(float)(homogeneousPoints.At<double>(0, counter) / homogeneousPoints.At<double>(3, counter)),
                            (float)(homogeneousPoints.At<double>(1, counter) / homogeneousPoints.At<double>(3, counter)),
                            (float)(homogeneousPoints.At<double>(2, counter) / homogeneousPoints.At<double>(3, counter))];
                    counter++;
                }*/
                
            }

            /* Overwriting existing feature LocationM and HeightM
            double[] lastlocation = [0,0,0];
            foreach (var obj in theObjs)
                foreach (ProcessFeature thisfeature in obj.Value.ProcessFeatures.Values)
                {
                    if (thisfeature.realLocation[0] == 0 && thisfeature.realLocation[1] == 0) //this happens because of the compare interval, the location is written to the first feature of the pair.
                    {
                        thisfeature.LocationM = new DroneLocation((float)lastlocation[1], (float)lastlocation[0]);
                        thisfeature.HeightM = (float)lastlocation[2];
                    }
                    else
                    {
                        thisfeature.LocationM = new DroneLocation((float)thisfeature.realLocation[1], (float)thisfeature.realLocation[0]);
                        thisfeature.HeightM = (float)thisfeature.realLocation[2];
                        lastlocation = thisfeature.realLocation;
                    }
                        Debug.WriteLine(thisfeature.BlockId.ToString() + "," + obj.Key.ToString() + "," + thisfeature.LocationM.EastingM.ToString() + "," + thisfeature.LocationM.NorthingM.ToString() + ",");
                     
                }
            */
        }

/*                    // Overwriting existing feature LocationM
                    thisfeature.LocationM = new DroneLocation((float)thisfeature.realLocation[1], (float)thisfeature.realLocation[0]);

                    // Change from altitude to height based on ground altitude at location
                    var droneDEM = processAll.GroundData.DemModel.GetElevationByDroneLocn(thisfeature.Block.DroneLocnM);
                    Assert(droneDEM != UnknownValue, thisfeature.FeatureId.ToString() + " drone location out of bounds");
                    var obsDEM = processAll.GroundData.DemModel.GetElevationByDroneLocn(thisfeature.LocationM);
                    Assert(obsDEM != UnknownValue,obs.ToString() + " " + thisfeature.FeatureId.ToString() + " object/feature location out of bounds");
                    thisfeature.HeightM = (float)thisfeature.realLocation[2] - obsDEM;  
                    var droneHeight = thisfeature.Block.AltitudeM - droneDEM;
                    Debug.WriteLine(thisfeature.BlockId.ToString() + "," + obs.ToString() 
                        + "," + thisfeature.LocationM.EastingM.ToString() + "," + thisfeature.LocationM.NorthingM.ToString() 
                        + "," + thisfeature.HeightM.ToString() + "," + thisfeature.realLocation[2].ToString()
                        + "," + droneHeight.ToString()+ "," + thisfeature.Block.AltitudeM.ToString()
                        + "," + thisfeature.Block.DroneLocnM.EastingM.ToString()+ "," + thisfeature.Block.DroneLocnM.NorthingM.ToString()
                        + "," + block.RollDeg.ToString() + "," + block.PitchDeg.ToString() + "," + block.YawDeg
                        );

                }
            }
        }
*/

        // Formulation for the intrinsic matrix.
        private static Mat Intrinsic(double focalLength, double imageWidth, double imageHeight, double sensorWidth, double sensorHeight)
        {
            var Cx = imageWidth / 2; var Cy = imageHeight / 2;
            var Fx = focalLength * imageWidth / sensorWidth; // F * pixels per mm = focal length in mm x image width px / sensor width mm
            var Fy = focalLength * imageHeight / sensorHeight;
            Mat K = new Mat(3, 3, MatType.CV_64F);
            K.At<double>(0, 0) = Fx;
            K.At<double>(0, 1) = 0;
            K.At<double>(0, 2) = Cx;
            K.At<double>(1, 0) = 0;
            K.At<double>(1, 1) = Fy;
            K.At<double>(1, 2) = Cy;
            K.At<double>(2, 0) = 0;
            K.At<double>(2, 1) = 0;
            K.At<double>(2, 2) = 1;
            return K;
        }

        // Convert collected points to OpenCV Mat format for triangulation
        public Mat ToTriangulationFormat(List<Point2d> points1)
        {
            var points1Mat = new Mat(2, points1.Count, MatType.CV_64F);
            for (int i = 0; i < points1.Count; i++)
            {
                points1Mat.Set<double>(0, i, points1[i].X);
                points1Mat.Set<double>(1, i, points1[i].Y);
            }
            return points1Mat;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------

    public class BlockInfo
    {
        private bool disposed = false;
        public SortedList<int, ProcessFeature> fromFeatures = new SortedList<int, ProcessFeature>();
        public SortedList<int, ProcessFeature> toFeatures = new SortedList<int, ProcessFeature>();
        public List<int> fromObs = new List<int>();
        // Create the points list
        public List<Point2d> CreatePoints(bool from)
        {
            var pointlist = new List<Point2d>();
            var feats = from ? fromFeatures : toFeatures;
            foreach (var obj in fromObs)
            {
                Debug.Write((from? "From obj,":"To obj,") +obj.ToString()+","+ "Feature " + feats[obj].FeatureId.ToString() + "," + "Drone altitude " + feats[obj].Block.AltitudeM.ToString() + ",");
                pointlist.Add(centroid(feats[obj].PixelBox));
                // pointlist.Add(new Point2d((feats[obj].PixelBox.X + feats[obj].PixelBox.Width/2 )/ 2, (feats[obj].PixelBox.Y + feats[obj].PixelBox.Height/2) / 2));
            }
            return pointlist;
        }

        // Using camera drone information, determine camera heading/position
        private static Mat CreateRotationMatrix(double rollDegrees, double pitchDegrees, double yawDegrees)
        {
            // Convert angles to radians
            double roll = rollDegrees * Math.PI / 180.0; //x
            double pitch = pitchDegrees * Math.PI / 180.0; //y
            double yaw = yawDegrees * Math.PI / 180.0; //z

            // Create rotation matrices
            using Mat Rx = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(Rx);
            Rx.At<double>(1, 1) = Math.Cos(roll);
            Rx.At<double>(1, 2) = -Math.Sin(roll);
            Rx.At<double>(2, 1) = Math.Sin(roll);
            Rx.At<double>(2, 2) = Math.Cos(roll);

            using Mat Ry = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(Ry);
            Ry.At<double>(0, 0) = Math.Cos(pitch);
            Ry.At<double>(0, 2) = Math.Sin(pitch);
            Ry.At<double>(2, 0) = -Math.Sin(pitch);
            Ry.At<double>(2, 2) = Math.Cos(pitch);

            using Mat Rz = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(Rz);
            Rz.At<double>(0, 0) = Math.Cos(yaw);
            Rz.At<double>(0, 1) = -Math.Sin(yaw);
            Rz.At<double>(1, 0) = Math.Sin(yaw);
            Rz.At<double>(1, 1) = Math.Cos(yaw);

            // Combine rotations
            return Rz * Ry * Rx;
        }

        /* Calulate the projection matrices from K, using the camera intrinsic matrix, and the drone camera's rotation matrix.
             The general form of the projection matrix P is P=K⋅[R∣t], where [R∣t] is the camera's extrinsic matrix, composed of the rotation matrix R and the translation vector t. 
             The translation vector t is derived from the camera's position C in the world as follows: t =  − R⋅C.   */
        public Mat CreateProjectionMatrix(
            double easting, double northing, double altitude,
            double rollDegrees, double pitchDegrees, double yawDegrees, Mat K)
        {
            // Create rotation matrix
            using Mat R = CreateRotationMatrix(rollDegrees, pitchDegrees, yawDegrees);

            // Create translation vector
            using Mat C = new Mat(3, 1, MatType.CV_64F);
            C.At<double>(0, 0) = easting;
            C.At<double>(1, 0) = northing;
            C.At<double>(2, 0) = altitude;

            // Create [R|t] matrix
            using Mat t = -R * C;
            using Mat Rt = new Mat(3, 4, MatType.CV_64F);
            for (int i = 0; i < 3; i++) // Copy rotation matrix
                for (int j = 0; j < 3; j++)
                    Rt.At<double>(i, j) = R.At<double>(i, j);
            
            for (int i = 0; i < 3; i++) // Copy negative translation
                Rt.At<double>(i, 3) = t.At<double>(i, 0);

            // Calculate P = K[R|t]
            return K * Rt;
        }
        public Mat PointDirection(ProcessFeature feat, Mat K)
        {
            using Mat R = CreateRotationMatrix(feat.Block.RollDeg, feat.Block.PitchDeg, feat.Block.YawDeg);

            var featpoint = centroid(feat.PixelBox);
            using Mat PixelPoint = new Mat(3, 1, MatType.CV_64F);
            PixelPoint.At<double>(0, 0) = featpoint.X;
            PixelPoint.At<double>(1, 0) = featpoint.Y;
            PixelPoint.At<double>(2, 0) = 1;
            return R * (K.Inv() * PixelPoint);
        }
        public List<double>? GetPoint(ProcessFeature fromF, ProcessFeature toF, Mat K)
        {
            Debug.WriteLine("=========  From object: " + fromF.ObjectId.ToString() + "=========  block: " + fromF.Block.BlockId.ToString() + "=========  feature: " + fromF.FeatureId.ToString());
            Debug.WriteLine("=========  To object: " + toF.ObjectId.ToString() + "=========  block: " + toF.Block.BlockId.ToString() + "=========  feature: " + toF.FeatureId.ToString());
            Debug.WriteLine("");
            
            using var RayFromF = PointDirection(fromF, K);
            using var RayToF = PointDirection(toF, K);

            using Mat C = new Mat(5, 1, MatType.CV_64F);
            C.At<double>(0, 0) = fromF.Block.DroneLocnM.EastingM;
            C.At<double>(1, 0) = fromF.Block.DroneLocnM.NorthingM;
            C.At<double>(2, 0) = fromF.Block.AltitudeM;
            C.At<double>(3, 0) = toF.Block.DroneLocnM.EastingM;
            C.At<double>(4, 0) = toF.Block.DroneLocnM.NorthingM;

            using Mat LHS = new Mat(5, 5, MatType.CV_64F, Scalar.All(0));
            LHS.At<double>(0, 0) = 1;
            LHS.At<double>(1, 1) = 1;
            LHS.At<double>(2, 2) = 1;

            LHS.At<double>(3, 0) = 1;
            LHS.At<double>(4, 1) = 1;

            LHS.At<double>(0, 3) = -RayFromF.At<double>(0, 0);
            LHS.At<double>(1, 3) = -RayFromF.At<double>(1, 0);
            LHS.At<double>(2, 3) = -RayFromF.At<double>(2, 0);

            LHS.At<double>(3, 4) = -RayToF.At<double>(0, 0);
            LHS.At<double>(4, 4) = -RayToF.At<double>(1, 0);


            Debug.WriteLine("Camera Position 1");
            Debug.Write(Math.Round(fromF.Block.DroneLocnM.EastingM, 2) + ", ");
            Debug.Write(Math.Round(fromF.Block.DroneLocnM.NorthingM, 2) + ", ");
            Debug.Write(Math.Round(fromF.Block.AltitudeM, 2) + ", ");

            Debug.WriteLine("");
            Debug.WriteLine("Camera Position 2");
            Debug.Write(Math.Round(toF.Block.DroneLocnM.EastingM, 2) + ", ");
            Debug.Write(Math.Round(toF.Block.DroneLocnM.NorthingM, 2) + ", ");
            Debug.Write(Math.Round(toF.Block.AltitudeM, 2) + ", ");

            Debug.WriteLine("");
            Debug.WriteLine("Ray 1");
            for (int j = 0; j < 3; j++)
            {
                Debug.Write(Math.Round(RayFromF.At<double>(j, 0), 2) + ", ");
            }
            Debug.WriteLine("");
            Debug.WriteLine("Ray 2");
            for (int j = 0; j < 3; j++)
            {
                Debug.Write(Math.Round(RayToF.At<double>(j, 0), 2) + ", ");
            }
            Debug.WriteLine("");
            
            
            var test = LHS.Determinant();
            Debug.WriteLine("Det");
            Debug.WriteLine(test.ToString());
            Debug.WriteLine("---------");


            if (!(test != 0 && test < 1000)) return null;
            using Mat Ans = LHS.Inv() * C;
            List<double> result = new();

            Debug.WriteLine("Params");
            for (int j = 3; j < 5; j++)
            {
                Debug.Write(Math.Round(Ans.At<double>(j, 0), 2) + ", ");
            }
            Debug.WriteLine("");

            Debug.WriteLine("Object calc 1");
            for (int j = 0; j < 3; j++)
            {
                Debug.Write(Math.Round(Ans.At<double>(j, 0), 2) + ", ");
                result.Add(Ans.At<double>(j, 0));
            }
            Debug.WriteLine("");
            Debug.WriteLine("---------");
            return result;
        }

    }

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
