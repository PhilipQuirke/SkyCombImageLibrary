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
using System.Drawing;
using System.Security.Cryptography.Xml;
using System.Net.Mail; // temporary for unit test


namespace SkyCombImage.ProcessLogic
{
    // ProcessSpanOptimize relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpanOptimize analyses ProcessObjects to find their location based on a triangulation of the drone's position and the object's image location in two images.

    public class SpanOptimize
    {
        // Parent process
        private ProcessAll Process { get; }
        private int CompareInterval { get; }
        public static CameraIntrinsic intrinsic = ProcessConfigModel.intrinsic; // Copy the static Lennard Spark drone camera information
        
        // Recalculate the Span.Objects.Features.LocationM and HeightM using triangulation.
        // Frame pair intervals constant, 3 frames is 1/20th of second. 5 frames is 1/6th of a second.
        public SpanOptimize(ProcessAll process, int compareInterval)
        {
            Process = process;
            CompareInterval = compareInterval;
            var totBlocks = Process.Blocks.Count;
            if (false)
            { // for debugging. Ensure that the time chosen includes features in the blocks.
                CompareInterval = 1;
                totBlocks = 2;
                process.Blocks[1].DroneLocnM = new DroneLocation(2, 0);
                process.Blocks[1].AltitudeM = 20;
                process.Blocks[1].RollDeg = 0;
                process.Blocks[1].PitchDeg = -89.91f;
                process.Blocks[1].YawDeg = -123.61888f;
                process.Blocks[2].DroneLocnM = new DroneLocation(3, 5); // Reminder, drone location does northing then easting
                process.Blocks[2].AltitudeM = 4;
                process.Blocks[2].RollDeg = 0;
                process.Blocks[2].PitchDeg = -89.915146f;
                process.Blocks[2].YawDeg = -123.63545f;
                var firstfeature = process.Blocks[1].MinFeatureId;
                process.Blocks[1].MaxFeatureId = firstfeature;
                var secondfeature = process.Blocks[2].MinFeatureId;
                process.Blocks[2].MaxFeatureId = secondfeature;
                var objid = process.ProcessFeatures[firstfeature].ObjectId;
                process.ProcessFeatures[secondfeature].ObjectId = objid;
                process.ProcessFeatures[firstfeature].Significant = true;
                process.ProcessFeatures[secondfeature].Significant = true;

            }
            foreach (var block in Process.Blocks.Values)
            {
                var blockId = block.BlockId;
                var compareBlockId = blockId + CompareInterval;
                if (compareBlockId > totBlocks) break; // not enough compare interval left
                var compareBlock = Process.Blocks[compareBlockId];
                if (block.MinFeatureId < 1 || compareBlock.MinFeatureId < 1) continue; // no features in either this or compare
                if (block.DroneLocnM.EastingM == compareBlock.DroneLocnM.EastingM
                    && block.DroneLocnM.NorthingM == compareBlock.DroneLocnM.NorthingM) continue; // not enough drone location difference for the compare

                for (var id = block.MinFeatureId; id <= block.MaxFeatureId; id++)
                {
                    Process.ProcessFeatures.TryGetValue(id, out var feature);
                    if (feature == null)
                        continue;

                    if (!DistinctFeature(feature)) continue;
                    var objid = feature.ObjectId;
                    for (var idC = compareBlock.MinFeatureId; idC <= compareBlock.MaxFeatureId; idC++)
                    {
                        Process.ProcessFeatures.TryGetValue(idC, out var featureC);
                        if (featureC == null)
                            continue;

                        var objidC = featureC.ObjectId;
                        if (objid != objidC) continue;
                        if (!DistinctFeature(featureC)) continue;

                        var result = GetPoint(feature, featureC);
                        if (result is not null)
                        {
                            feature.LocationM = new DroneLocation((float)result[2], (float)result[0]); // Northing is stored first in location
                            // Change from altitude to height based on ground altitude at location
                            var obsDEM = process.GroundData.DemModel.GetElevationByDroneLocn(feature.LocationM);
                            Debug.Assert(obsDEM != -999, objid.ToString() + " " + feature.FeatureId.ToString() + " object/feature location out of bounds");
                            feature.HeightM = (float)result[1] - obsDEM;
                        }
                    }
                }
            }
        }
        public bool DistinctFeature(ProcessFeature feature)
        {
            if (!feature.Significant) return false;
            // we have to divide pixel box dimensions by 2 because the DJI camera is returning double intensity. 
            if ((feature.PixelBox.X / 2 <= 1) || (feature.PixelBox.X / 2 + feature.PixelBox.Width / 2 >= intrinsic.ImageWidth) || (feature.PixelBox.Y / 2 <= 1) || (feature.PixelBox.Y / 2 + feature.PixelBox.Height / 2 >= intrinsic.ImageHeight)) return false; //too close to edge
            return true;
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
        private static void FillMat(Mat mat, double[] values)
        {
            for (int i = 0; i < values.Length; i++)
            {
                    mat.At<double>(i) = values[i];
            }
        }
        public Mat PointDirection(Point2d featpoint, Mat R)
        {
            using Mat PixelPoint = new Mat(3, 1, MatType.CV_64F);
            FillMat(PixelPoint, new double[] { featpoint.X, featpoint.Y, 1 });
            return R.Inv() * intrinsic.K.Inv() * PixelPoint;
        }
        public List<double>? GetPoint(ProcessFeature fromF, ProcessFeature toF)
        {
            List<Point2d> adjustedPoints;
            //adjustedPoints = AdjustedPoints(fromF, toF);
            adjustedPoints = new List<Point2d> { new Point2d(fromF.PixelBox.X/2 + fromF.PixelBox.Width/4, fromF.PixelBox.Y/2 + fromF.PixelBox.Height / 4), new Point2d(toF.PixelBox.X/2 + toF.PixelBox.Width / 4, toF.PixelBox.Y/2 + toF.PixelBox.Height / 4) };
            if (false)
            {// for debugging
                adjustedPoints = new List<Point2d> { new Point2d(694.8387729, 791.3252464), new Point2d(433.7235783, 351.4702146) };
            }
            using Mat c1 = new Mat(3, 1, MatType.CV_64F);
            FillMat(c1, new double[] { fromF.Block.DroneLocnM.EastingM, fromF.Block.AltitudeM, fromF.Block.DroneLocnM.NorthingM });
            using Mat c2 = new Mat(3, 1, MatType.CV_64F);
            FillMat(c2, new double[] { toF.Block.DroneLocnM.EastingM, toF.Block.AltitudeM, toF.Block.DroneLocnM.NorthingM });
            using Mat R1 = CreateRotationMatrix(fromF.Block.RollDeg, fromF.Block.PitchDeg, fromF.Block.YawDeg);
            using var RayFromF = PointDirection(adjustedPoints[0], R1);
            using Mat t1 = R1 * c1;
            using Mat R2 = CreateRotationMatrix(toF.Block.RollDeg, toF.Block.PitchDeg, toF.Block.YawDeg);
            using var RayToF = PointDirection(adjustedPoints[1], R2);
            using Mat t2 = R2 * c2;

            using Mat C = c1 - c2;

            using Mat LHS = new Mat(3, 2, MatType.CV_64F, Scalar.All(0));
            LHS.At<double>(0,0)= -RayFromF.At<double>(0, 0);
            LHS.At<double>(1,0)= -RayFromF.At<double>(1, 0);
            LHS.At<double>(2,0)= -RayFromF.At<double>(2, 0);
            LHS.At<double>(0,1)= RayToF.At<double>(0, 0);
            LHS.At<double>(1,1)= RayToF.At<double>(1, 0);
            LHS.At<double>(2,1)= RayToF.At<double>(2, 0);

            //=====================================================================
            //                        
            using Mat Moore_Penrose_LHS_Inv = (LHS.Transpose() * LHS).Inv() * LHS.Transpose(); //https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
            using Mat Params = Moore_Penrose_LHS_Inv * C;
            using Mat zerotest = LHS * Params - C;
            using Mat Ans1 = c1 + RayFromF * Params.At<double>(0, 0);
            using Mat Ans2 = c2 + RayToF * Params.At<double>(1, 0);
            //
            //=====================================================================
            
            Debug.WriteLine("==  From obj: " + fromF.ObjectId.ToString() + "===  block: " + fromF.Block.BlockId.ToString() + "===  feature: " + fromF.FeatureId.ToString());
            Debug.WriteLine("==  To obj: " + toF.ObjectId.ToString() + "===  block: " + toF.Block.BlockId.ToString() + "===  feature: " + toF.FeatureId.ToString());
            Debug.WriteLine("");
            Debug.WriteLine("Cam diff");
            for (int j = 0; j < 3; j++)
            {
                Debug.Write(Math.Round(C.At<double>(j, 0), 3) + ", ");
            }
            Debug.WriteLine("");
            Debug.WriteLine("Points");
           
            Debug.WriteLine(adjustedPoints[0].X.ToString() + ", " + adjustedPoints[0].Y.ToString() + ", " + adjustedPoints[1].X.ToString() + ", " + adjustedPoints[1].Y.ToString() );
            Debug.WriteLine("");

            Debug.WriteLine("Camera Position 1");
            Debug.WriteLine(c1.At<double>(0, 0).ToString() + ", " + c1.At<double>(1, 0).ToString() + ", " + c1.At<double>(2, 0).ToString() + ", ");
            Debug.WriteLine(fromF.Block.RollDeg.ToString() + ", " + fromF.Block.PitchDeg + ", " + fromF.Block.YawDeg);
            Debug.WriteLine("");
            Debug.WriteLine("Camera Position 2");
            Debug.WriteLine(c2.At<double>(0, 0).ToString() + ", " + c2.At<double>(1, 0).ToString() + ", " + c2.At<double>(2, 0).ToString() + ", ");
            Debug.WriteLine(toF.Block.RollDeg.ToString() + ", " + toF.Block.PitchDeg + ", " + toF.Block.YawDeg);

            Debug.WriteLine("");
            Debug.WriteLine("LHS = -Ray 1 | Ray 2");
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    Debug.Write(Math.Round(LHS.At<double>(i, j), 3) + "  |  ");
                }
                Debug.WriteLine("");
            }
            Debug.WriteLine("");
            Debug.WriteLine("Params");
            for (int j = 0; j < 2; j++)
            {
                Debug.Write(Math.Round(Params.At<double>(j, 0), 2) + ", ");
            }
            Debug.WriteLine("");

            Debug.WriteLine("Zero test");
            for (int j = 0; j < 3; j++)
            {
                Debug.Write(Math.Round(zerotest.At<double>(j, 0), 2) + ", ");
            }
            Debug.WriteLine("");
            Debug.WriteLine("Object calc 1 | 2:");
            
            for (int j = 0; j < 3; j++)
            {
                Debug.WriteLine(Math.Round(Ans1.At<double>(j, 0), 3) + "   |   " + Math.Round(Ans2.At<double>(j, 0), 3));
            }
            Debug.WriteLine("");
            Debug.WriteLine("---------");

//            Debug.WriteLine(Ans1.At<double>(0, 0) + ", " + Ans1.At<double>(1, 0) + ", " + Ans1.At<double>(2, 0) + ", " + Ans2.At<double>(0, 0) + ", " + Ans2.At<double>(1, 0) + ", " + Ans2.At<double>(2, 0));
 
            //=====================================================================

            if (Ans1.At<double>(0, 0) + Ans1.At<double>(2, 0) > Ans2.At<double>(0, 0) + Ans2.At<double>(2, 0)) // northing or easting is negative
            {
                return [ Ans1.At<double>(0, 0), Ans1.At<double>(1, 0), Ans1.At<double>(2, 0) ];
            }
            else return [Ans2.At<double>(0, 0), Ans2.At<double>(1, 0), Ans2.At<double>(2, 0)];
        }

    
        private List<Point2d>? AdjustedPoints(ProcessFeature fromf, ProcessFeature tof)
        {
            var obj = fromf.ObjectId;
            var firstFeat = fromf.Block.BlockId;
            var lastfeat = tof.Block.BlockId;
            var featlist = Process.ProcessObjects[obj].ProcessFeatures.Values;
            int M = 2 * CompareInterval; // q fitting
            int N = (CompareInterval + 1) * 4 + 2; // xf, yf, vxf, vyf, q1, q2
            double[] bndl = new double[N]; // xf and yf rectangle bounds, vxf and vyf >= 0, q1 and q2 unconstrained
            double[] bndu = new double[N];
            double[] AL = new double[M];
            double[] AU = new double[M];
            double[,] A = new double[M, N];
            double[] C = new double[N]; // including q1 & q2 in the cost function with multiplied by 0
            double[] s = new double[N]; //scale
            int featcount = 0;
            foreach (var feature in featlist)
            {
                if ((feature.BlockId > lastfeat) || (feature.BlockId < firstFeat)) continue;
                //xf
                bndl[featcount * 4] = feature.PixelBox.X / 2;
                bndu[featcount * 4] = (feature.PixelBox.X + feature.PixelBox.Width) / 2;
                C[featcount * 4] = 1; //xf
                s[featcount * 4] = 100;
                //yf
                bndl[featcount * 4 + 1] = feature.PixelBox.Y / 2;
                bndu[featcount * 4 + 1] = (feature.PixelBox.Y + feature.PixelBox.Height) / 2;
                C[featcount * 4 + 1] = 1; //yf
                s[featcount * 4 + 1] = 100;
                //vxf
                bndl[featcount * 4 + 2] = 0;
                bndu[featcount * 4 + 2] = System.Double.PositiveInfinity;
                C[featcount * 4 + 2] = CompareInterval; //vxf
                s[featcount * 4 + 2] = 70;
                //vyf
                bndl[featcount * 4 + 3] = 0;
                bndu[featcount * 4 + 3] = System.Double.PositiveInfinity;
                C[featcount * 4 + 3] = CompareInterval; //vyf
                s[featcount * 4 + 3] = 70;

                if (feature.BlockId < lastfeat)
                {
                    // [xf  +vxf - x(f+1) -vx(f+1)- q1 ]
                    A[featcount * 2 + 0, featcount * 4] = 1;
                    A[featcount * 2 + 0, featcount * 4 + 2] = 1;
                    A[featcount * 2 + 0, (featcount + 1) * 4] = -1;
                    A[featcount * 2 + 0, (featcount + 1) * 4 + 2] = -1;
                    A[featcount * 2 + 0, CompareInterval * 4 + 4] = -1;
                    // [yf  +vyf- y(f+1) -vy(f+1)- q2]
                    A[featcount * 2 + 1, featcount * 4 + 1] = 1;
                    A[featcount * 2 + 1, featcount * 4 + 3] = 1;
                    A[featcount * 2 + 1, (featcount + 1) * 4 + 1] = -1;
                    A[featcount * 2 + 1, (featcount + 1) * 4 + 3] = -1;
                    A[featcount * 2 + 1, CompareInterval * 4 + 5] = -1;
                    // AL and AU are zero by default
                }
                else
                {
                    // final scaling for q
                    //q1
                    bndl[featcount * 4 + 4] = -System.Double.PositiveInfinity;
                    bndu[featcount * 4 + 4] = System.Double.PositiveInfinity;
                    s[featcount * 4 + 4] = 200;
                    //q2
                    bndl[featcount * 4 + 5] = -System.Double.PositiveInfinity;
                    bndu[featcount * 4 + 5] = System.Double.PositiveInfinity;
                    s[featcount * 4 + 5] = 200;
                    // cost function for q1 & q2 auto set to zero because we don't want to minimize them
                }
                featcount++;
            }

            double[] ans;
            alglib.minlpstate state;
            alglib.minlpreport rep;

            alglib.minlpcreate(N, out state);
            alglib.minlpsetcost(state, C);
            alglib.minlpsetbc(state, bndl, bndu);
            alglib.minlpsetlc2dense(state, A, AL, AU, M);
            alglib.minlpsetscale(state, s);
            alglib.minlpsetalgodss(state, 0);
            alglib.minlpoptimize(state);
            alglib.minlpresults(state, out ans, out rep);
            //Debug.WriteLine("{0}", alglib.ap.format(ans, 3));
            //Debug.WriteLine("{0}", rep.terminationtype);

            if (rep.terminationtype != 1) return null;
            List<Point2d> points = new();
            points.Add(new Point2d(ans[0], ans[1]));
            points.Add(new Point2d(ans[CompareInterval * 4], ans[CompareInterval * 4 + 1]));

            return points;

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


        }

    }
}
