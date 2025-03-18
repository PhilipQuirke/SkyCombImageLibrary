// Copyright SkyComb Limited 2025. All rights reserved. 
using Accord.Math;
using Accord.Statistics.Kernels;
using Accord;
using MathNet.Numerics.LinearAlgebra.Factorization;
using Microsoft.VisualBasic.ApplicationServices;
using Microsoft.VisualBasic;
using OpenCvSharp;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Data;
using System.Diagnostics;
using System.DirectoryServices.ActiveDirectory;
using System.Drawing;
using static Emgu.CV.XImgproc.SupperpixelSLIC;
using static MS.WindowsAPICodePack.Internal.CoreNativeMethods;
using static OfficeOpenXml.ExcelErrorValue;
using static System.Windows.Forms.AxHost;
using static System.Windows.Forms.DataFormats;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.ToolTip;
using System.Dynamic;
using System.Net;
using System.Threading;
using YoloDotNet.Models;
using Emgu.CV.CvEnum;
//using alglib;



// Q1: Is zoom not zero? zoom is none or 1-1 or 100: "dzoom_ratio: 1.00"
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

            if (true)
            {
                // nq new method
                TriangulateSpanObjectsFeaturesLocationAndHeight(Process);
                foreach (var theObj in theObjs)
                    theObj.Value.Calculate_RealObject_SimpleMemberData();
                theObjs.CalculateSettings();
            }
            else
            {
                // Old method
                //CalculateSettings_FixValues(legSteps, theObjs);

                // No method
                int theHFOVDeg = Process.Drone.InputVideo.HFOVDeg;
                ResetBest();
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs);
                OrgSumLocnErrM = BestSumLocnErrM;
                OrgSumHeightErrM = BestSumHeightErrM;
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


        //VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        // Copy the static Lennard Spark drone camera intrinsic matrix
        public static Mat K = ProcessConfigModel.LennardsDroneK;
        public static Point2d ImageDim = ProcessConfigModel.LennardsDroneImageDimensions;
        //           Fix and test with non zero animal
        // Recalculate the Span.Objects.Features.LocationM and HeightM using triangulation.
        public void TriangulateSpanObjectsFeaturesLocationAndHeight(ProcessAll processAll)
        {
            var compareInterval = 3; // Frame pair intervals constant, 3 frames is 1/20th of second. 5 frames is 1/6th of a second.
            var totBlocks = Process.Blocks.Count;
            foreach (var block in Process.Blocks.Values)
            {
                var blockId = block.BlockId;
                var compareBlockId = blockId + compareInterval;
                if (compareBlockId > totBlocks) break; // not enough compare interval left
                var compareBlock = Process.Blocks[compareBlockId];
                if (block.MinFeatureId < 1 || compareBlock.MinFeatureId < 1) continue; // no features in either this or compare
                if (block.DroneLocnM.EastingM == compareBlock.DroneLocnM.EastingM
                    && block.DroneLocnM.NorthingM == compareBlock.DroneLocnM.NorthingM) continue; // not enough drone location difference for the compare
                var BlocksInfo = new BlockInfo();

                for (var id = block.MinFeatureId; id <= block.MaxFeatureId; id++)
                {
                    var feature = Process.ProcessFeatures[id];
                    if (!BlockInfo.DistinctFeature(feature, ImageDim)) continue;
                    var objid = feature.ObjectId;
                    for (var idC = compareBlock.MinFeatureId; idC <= compareBlock.MaxFeatureId; idC++)
                    {
                        var featureC = Process.ProcessFeatures[idC];
                        var objidC = featureC.ObjectId;
                        if (objid != objidC) continue;
                        if (!BlockInfo.DistinctFeature(featureC, ImageDim)) continue;

                        var result = BlocksInfo.GetPoint(feature, featureC, K, processAll, ImageDim, compareInterval);
                        if (result is not null)
                        {
                            feature.LocationM = new DroneLocation((float)result[1], (float)result[0]); // Northing is stored first in location
                            // Change from altitude to height based on ground altitude at location
                            var obsDEM = processAll.GroundData.DemModel.GetElevationByDroneLocn(feature.LocationM);
                            Assert(obsDEM != UnknownValue, objid.ToString() + " " + feature.FeatureId.ToString() + " object/feature location out of bounds");
                            feature.HeightM = (float)result[2] - obsDEM;
                        }
                    }
                }
            }
        }

    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------

    public class BlockInfo
    {
        public static bool DistinctFeature(ProcessFeature feature, Point2d imageDim)
        {
            if (!feature.Significant) return false;
            // we have to divide pixel box dimensions by 2 because the DJI camera is returning double intensity. 
            if ((feature.PixelBox.X / 2 <= 1) || (feature.PixelBox.X / 2 + feature.PixelBox.Width / 2 >= imageDim.X) || (feature.PixelBox.Y / 2 <= 1) || (feature.PixelBox.Y / 2 + feature.PixelBox.Height / 2 >= imageDim.Y)) return false; //too close to edge
            return true;
        }
        private static List<Point2d>? AdjustedPoints(ProcessFeature fromf, ProcessFeature tof, ProcessAll processAll, Point2d imageDim, int interval)
        {
            // we want to minimize: Sum over features f in selected interval I with feature centroid (cxf, cyf) feature width and height (wf, hf)
            // of F(xf, yf) = (xf - cxf + yf - cyf) => min F(xf, yf) = xf + yf
            //  where for every f: yf = a.yx + k
            //      => if drone is going in a straight line at constant speed, for every f and the adjacent (f+1): xf - x(f+1) - q1 = 0, yf - y(f+1) - q2 = 0
            //  and q1, q2 unconstrained
            //  and for every fx: cxf - wf <= xf <= cxf + wf
            //  and for every fy: cyf - hf <= yf <= cyf + hf
            //Therefore we have (2*I + 2) variables xf, yf, q1, q2; I = number of features in the interval

            /* Cost term: C array N. state=algorithm state
              c = [Sum over f(xf + yf)]
              minlpsetcost(minlpstate state, double[] c)

             * General linear constraints are specified as AL<=A*x<=AU; AL,AU vectors, A matrix; K is the number of equality/inequality constraints
              AL = [[cxf - wf/2], [cyf - hf/2], [0], [0]]
              AU = [[cxf + wf/2], [cyf + hf/2], [0], [0]]
              A = [[xf], [yf], [xf - x(f+1) - q1], [yf - y(f+1) - q2]]
              K = (I-1) + (I-1) + I + I
              minlpsetlc2(minlpstate state, sparsematrix a, double[] al, double[] au, int k)

             * ALGLIB optimizers use scaling matrices to test stopping  conditions and as preconditioner. S=array[N], non-zero scaling coefficients
                Scale of the I-th variable is a translation invariant measure of:
                a) "how large" the variable is
                b) how large the step should be to make significant changes in the function.
                s = [1/2 image width, 1/2 image height
                minlpsetscale(minlpstate state, double[] s)
            */

            /* If infeasible, we could add into the objective function quadruple weighted variables vxf and vyf (which would reveal where a feature was useless)
             * such that:
             *  0 <= xf - x(f+1) - q1 + vxf - vx(f+1) <= 0
             *  0 <= yf - y(f+1) - q1 + vyf - vy(f+1) <= 0
             *  vxf >= 0, vyf >= 0, scale 10?
             *  The weight could be I, so cost function => min F(xf, yf, vxf, vyf) = xf + yf + I.vxf + I.vyf
             *  
             *  Conversely, if too many feasible points, could make xf double weighted and add half weighted variables constrained closer to the centroid.
             */
            var obj = fromf.ObjectId;
            var firstFeat = fromf.Block.BlockId;
            var lastfeat = tof.Block.BlockId;
            var featlist = processAll.ProcessObjects[obj].ProcessFeatures.Values;
            int M = (interval + 1) * 2 + 2*interval + (interval + 1) * 2 + 2; // xf and yf rectangle bounds, q fitting, vxf and vyf >= 0, q1 and q2 unconstrained
            int N = (interval + 1) * 4 + 2; // xf, yf, vxf, vyf, q1, q2
            double[] AL = new double[M]; 
            double[] AU = new double[M];
            double[,] A = new double[M,N];
            double[] C = new double[N]; // including q1 & q2 in the cost function with multiplied by 0
            double[] s = new double[N]; //scale
            int featcount = 0;
            foreach (var feature in featlist)            
            {
                if ((feature.BlockId > lastfeat) || (feature.BlockId < firstFeat)) continue;
                //xf
                AL[featcount * 6] = feature.PixelBox.X / 2; 
                AU[featcount * 6] = (feature.PixelBox.X + feature.PixelBox.Width) / 2;
                A[featcount * 6, featcount * 4] = 1; 
                //yf
                AL[featcount * 6 + 1] = feature.PixelBox.Y / 2; 
                AU[featcount * 6 + 1]=(feature.PixelBox.Y + feature.PixelBox.Height) / 2;
                A[featcount * 6 + 1, featcount * 4 + 1] = 1; 
                //vxf
                AL[featcount * 6 + 2] = 0;
                AU[featcount * 6 + 2] = System.Double.PositiveInfinity;
                A[featcount * 6 + 2, featcount * 4 + 2] = 1; 
                //vyf
                AL[featcount * 6 + 3] = 0;
                AU[featcount * 6 + 3] = System.Double.PositiveInfinity;
                A[featcount * 6 + 3, featcount * 4 + 3] = 1; 

                // cost function
                C[featcount * 4] = 1; //xf
                C[featcount * 4 + 1] = 1; //yf
                C[featcount * 4 + 2] = interval; //vxf
                C[featcount * 4 + 3] = interval; //vyf
                // scaling for xf and yf
                s[featcount * 4] = imageDim.X/interval;
                s[featcount * 4 + 1] = imageDim.Y/interval;
                s[featcount * 4 + 2] = 10;
                s[featcount * 4 + 3] = 10;

                if (feature.BlockId < lastfeat) 
                {
                    // [xf  +vxf - x(f+1) -vx(f+1)- q1 ]
                    A[featcount * 6 + 4, featcount * 4 ] = 1;
                    A[featcount * 6 + 4, featcount * 4 + 2] = 1;
                    A[featcount * 6 + 4, (featcount + 1) * 4] = -1;
                    A[featcount * 6 + 4, (featcount + 1) * 4 + 2] = -1;
                    A[featcount * 6 + 4, interval * 4 + 4] = -1;
                    // [yf  +vyf- y(f+1) -vy(f+1)- q2]
                    A[featcount * 6 + 5, featcount * 4 + 1] = 1;
                    A[featcount * 6 + 5, featcount * 4 + 3] = 1;
                    A[featcount * 6 + 5, (featcount + 1) * 4 + 1] = -1;
                    A[featcount * 6 + 5, (featcount + 1) * 4 + 3] = -1;
                    A[featcount * 6 + 5, interval * 4 + 5] = -1;
                    // AL and AU are zero by default
                }
                else
                {
                    // final scaling for q
                    s[featcount * 4 + 4] = imageDim.X / interval;
                    s[featcount * 4 + 5] = imageDim.Y / interval;
                    // unconstrained q1 and q2
                    A[featcount * 6 + 4, interval * 4 + 4] = 1;
                    A[featcount * 6 + 5, interval * 4 + 5] = 1;
                    AL[featcount * 6 + 4] = -System.Double.PositiveInfinity;
                    AU[featcount * 6 + 4] = System.Double.PositiveInfinity;
                    AL[featcount * 6 + 5] = -System.Double.PositiveInfinity;
                    AU[featcount * 6 + 5] = System.Double.PositiveInfinity;
                    // cost function for q1 & q2 auto set to zero because we don't want to minimize them
                }
                // Assuming at present that I don't have to put in bounds for q1 and q2, otherwise they would be set at -System.Double.PositiveInfinity and System.Double.PositiveInfinity
                featcount++;
            }
            Debug.WriteLine("{0}", alglib.ap.format(A, 2));
            Debug.WriteLine("{0}", alglib.ap.format(AL, 2));
            Debug.WriteLine("{0}", alglib.ap.format(AU, 2));
            Debug.WriteLine(alglib.ap.format(C, 2));

            double[] ans;
            alglib.minlpstate state;
            alglib.minlpreport rep;

            alglib.minlpcreate(N, out state);
            alglib.minlpsetcost(state, C);
            //alglib.minlpsetbc(state, bndl, bndu);
            alglib.minlpsetlc2dense(state, A, AL, AU, M);
            alglib.minlpsetscale(state, s);
            alglib.minlpsetalgodss(state,0);
            alglib.minlpoptimize(state);
            alglib.minlpresults(state, out ans, out rep);
            // Debug.WriteLine("{0}", alglib.ap.format(ans, 3));
            Debug.WriteLine("{0}", rep.terminationtype);

            return null;
            /* The subroutine creates LP  solver.  After  initial  creation  it  contains
            default optimization problem with zero cost vector and all variables being
fixed to zero values and no constraints.

In order to actually solve something you should:
*set cost vector with minlpsetcost()
*set variable bounds with minlpsetbc(), or minlpsetbcall() if constraints for all variables are same
*
*Following types of constraints are supported:

    DESCRIPTION         CONSTRAINT              HOW TO SPECIFY
    fixed variable      x[i]=Bnd[i]             BndL[i]=BndU[i]
    lower bound         BndL[i]<=x[i]           BndU[i]=+INF
    upper bound         x[i]<=BndU[i]           BndL[i]=-INF
    range               BndL[i]<=x[i]<=BndU[i]  ...
    free variable       -                       BndL[I]=-INF, BndU[I]+INF

INPUT PARAMETERS:
    State   -   structure stores algorithm state
    BndL    -   lower bounds, array[N].
    BndU    -   upper bounds, array[N].

NOTE: infinite values can be specified by means of Double.PositiveInfinity
      and  Double.NegativeInfinity  (in  C#)  and  alglib::fp_posinf   and
      alglib::fp_neginf (in C++).
*
*
*
*specify constraint matrix with one of the following functions:
            [*] minlpsetlc()        for dense one-sided constraints
            [*] minlpsetlc2dense()  for dense two-sided constraints
            [*] minlpsetlc2()       for sparse two-sided constraints
                    [*] minlpaddlc2dense()  to add one dense row to constraint matrix
                    [*] minlpaddlc2()       to add one row to constraint matrix(compressed format)
* call minlpoptimize() to run the solver and  minlpresults()  to  get  the
  solution vector and additional information.

By  default, LP  solver uses best algorithm available.As of ALGLIB 3.17,
sparse interior point(barrier) solver is used.Future releases of  ALGLIB
may introduce other solvers.

User may choose specific LP algorithm by calling:
*minlpsetalgodss() for revised dual simplex method with DSE  pricing  and
  bounds flipping ratio test(aka long dual step).Large - scale  sparse LU
  solverwith  Forest - Tomlin update is used internally as linear  algebra
  driver.
* minlpsetalgoipm() for sparse interior point method

INPUT PARAMETERS:
    N - problem size

OUTPUT PARAMETERS:
    State - optimizer in the default state

REPORT
TerminationType field contains completion code, which can be:
  -8    internal integrity control detected  infinite  or  NAN  values  in
        function/gradient. Abnormal termination signalled.
  -3    inconsistent constraints. Feasible point is
        either nonexistent or too hard to find. Try to
        restart optimizer with better initial approximation
   1    relative function improvement is no more than EpsF.
   2    relative step is no more than EpsX.
   4    gradient norm is no more than EpsG
   5    MaxIts steps was taken
   7    stopping conditions are too stringent,
        further improvement is impossible,
        X contains best point found so far.
   8    terminated by user who called minbleicrequesttermination(). X contains
        point which was "current accepted" when  termination  request  was
        submitted.

*/
        }

        // Calculate the centroid of the pixelbox and convert location to real pixels, half that given, because the image has been double sized by DGI.
        private Point2d centroid(Rectangle rectangle)
        {
            Point2d centroid = new Point2d();
            centroid.X = (rectangle.X + rectangle.Width / 2)/2;
            centroid.Y = (rectangle.Y + rectangle.Height / 2)/2;
            Debug.WriteLine(centroid.X.ToString() + "," + centroid.Y.ToString() + ",");

            return centroid;

        }
        // Using camera drone information, determine camera heading/position
        private static Mat CreateRotationMatrix(double rollDegrees, double pitchDegrees, double yawDegrees)
        {
            // Convert angles to radians
            double roll = rollDegrees * Math.PI / 180.0;
            double pitch = pitchDegrees * Math.PI / 180.0;
            double yaw = yawDegrees * Math.PI / 180.0;

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

        public Mat PointDirection(ProcessFeature feat, Mat t, Mat K)
        {
            using Mat R = CreateRotationMatrix(feat.Block.RollDeg, feat.Block.PitchDeg, feat.Block.YawDeg);

            var featpoint = centroid(feat.PixelBox);
            using Mat PixelPoint = new Mat(3, 1, MatType.CV_64F);
            PixelPoint.At<double>(0, 0) = featpoint.X;
            PixelPoint.At<double>(1, 0) = featpoint.Y;
            PixelPoint.At<double>(2, 0) = 1;
            return R.Inv() * (K.Inv() * PixelPoint - t);
        }
        public List<double>? GetPoint(ProcessFeature fromF, ProcessFeature toF, Mat K, ProcessAll process, Point2d imageDim, int interval)
        {
            var adjustedPoints = AdjustedPoints(fromF, toF, process, imageDim, interval);
                            Debug.WriteLine("=========  From object: " + fromF.ObjectId.ToString() + "=========  block: " + fromF.Block.BlockId.ToString() + "=========  feature: " + fromF.FeatureId.ToString());
                            Debug.WriteLine("=========  To object: " + toF.ObjectId.ToString() + "=========  block: " + toF.Block.BlockId.ToString() + "=========  feature: " + toF.FeatureId.ToString());
                            Debug.WriteLine("");
            
            using Mat t1 = new Mat(3, 1, MatType.CV_64F);
            t1.At<double>(0, 0) = fromF.Block.DroneLocnM.EastingM;
            t1.At<double>(1, 0) = fromF.Block.DroneLocnM.NorthingM;
            t1.At<double>(2, 0) = fromF.Block.AltitudeM;

            using Mat t2 = new Mat(3, 1, MatType.CV_64F);
            t1.At<double>(0, 0) = toF.Block.DroneLocnM.EastingM;
            t1.At<double>(1, 0) = toF.Block.DroneLocnM.NorthingM;
            t1.At<double>(2, 0) = toF.Block.AltitudeM;

            using var RayFromF = PointDirection(fromF, t1, K);
            using var RayToF = PointDirection(toF, t2, K);

            using Mat C = t1 - t2;

            using Mat LHS = new Mat(3, 2, MatType.CV_64F, Scalar.All(0));
            LHS.At<double>(0, 0) = -RayFromF.At<double>(0, 0);
            LHS.At<double>(1, 0) = -RayFromF.At<double>(1, 0);
            LHS.At<double>(2, 0) = -RayFromF.At<double>(2, 0);

            LHS.At<double>(0, 1) = RayToF.At<double>(0, 0); 
            LHS.At<double>(1, 1) = RayToF.At<double>(1, 0);
            LHS.At<double>(2, 1) = RayToF.At<double>(2, 0);  


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

            //=====================================================================
            //                        
            using Mat Moore_Penrose_LHS_Inv = (LHS.Transpose() * LHS).Inv() * LHS.Transpose(); //https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
            using Mat test0 = (LHS.Transpose() * LHS);
            Debug.WriteLine(test0.Determinant().ToString()); //https://chatgpt.com/c/67d226cb-e424-800e-a424-2157f1b28340
            using Mat Params = Moore_Penrose_LHS_Inv * C;
            using Mat test1 = Moore_Penrose_LHS_Inv * LHS;
            using Mat test2 = LHS * Params - C;
            using Mat Ans1 = t1 + RayFromF * Params.At<double>(0, 0);
            using Mat Ans2 = t2 + RayToF * Params.At<double>(1, 0);
            //
            //=====================================================================
            List<double> result = new();

                            Debug.WriteLine("Params");
                            for (int j = 0; j < 2; j++)
                            {
                                Debug.Write(Math.Round(Params.At<double>(j, 0), 2) + ", ");
                            }
                            Debug.WriteLine("");

                            Debug.WriteLine("Object calc 1");
                            for (int j = 0; j < 3; j++)
                            {
                                result.Add(Ans1.At<double>(j, 0));
                                Debug.Write(Math.Round(Ans1.At<double>(j, 0), 2) + ", ");
                            }
                            Debug.WriteLine("");

                            Debug.WriteLine("Object calc 2");
                            for (int j = 0; j < 3; j++)
                            {
                                Debug.Write(Math.Round(Ans2.At<double>(j, 0), 2) + ", ");
                            }
                            Debug.WriteLine("");
                            Debug.WriteLine("Zero test");
                            for (int j = 0; j < 3; j++)
                            {
                                Debug.Write(Math.Round(test2.At<double>(j, 0), 2) + ", ");
                            }
                            Debug.WriteLine("");

                            Debug.WriteLine("Identity test");
                            for (int j = 0; j < 2; j++)
                            {
                                for (int k = 0; k < 2; k++)
                                {
                                    Debug.Write(Math.Round(test1.At<double>(j, k), 2) + ", ");
                                }
                                Debug.WriteLine("");
                            }
                            Debug.WriteLine("");
                            Debug.WriteLine("---------");
            if (test2.At<double>(0, 0) < 0.1 && test2.At<double>(1, 0) < 0.1 && test2.At<double>(2, 0) < 0.1 
                && test2.At<double>(0, 0) > -0.1 && test2.At<double>(1, 0) > -0.1 && test2.At<double>(2, 0) > -0.1)
            {
                return result;
            }
            else return null;
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
