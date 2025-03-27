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
        public double[] GetPoint(ProcessFeature fromF, ProcessFeature toF)
        {
            List<Point2d> adjustedPoints;
            //adjustedPoints = AdjustedPoints(fromF, toF);
            adjustedPoints = new List<Point2d> { new Point2d(fromF.PixelBox.X / 2 + fromF.PixelBox.Width / 4, fromF.PixelBox.Y / 2 + fromF.PixelBox.Height / 4), new Point2d(toF.PixelBox.X / 2 + toF.PixelBox.Width / 4, toF.PixelBox.Y / 2 + toF.PixelBox.Height / 4) };
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
            using Mat R2 = CreateRotationMatrix(toF.Block.RollDeg, toF.Block.PitchDeg, toF.Block.YawDeg);
            using var RayToF = PointDirection(adjustedPoints[1], R2);

            using Mat RHS = new Mat(6, 1, MatType.CV_64F);
            Cv2.VConcat(c1, c2, RHS);
            using Mat I = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(I);
            using Mat StackI = new Mat(6, 3, MatType.CV_64F);
            Cv2.VConcat(I, I, StackI);
            using Mat StackD = new Mat(6, 1, MatType.CV_64F);
            Cv2.VConcat(-RayFromF, -RayToF, StackD);
            using Mat LHS = new Mat(6, 8, MatType.CV_64F);
            Cv2.HConcat(StackI, StackD, LHS);

            //=====================================================================
            //                        
            //using Mat Moore_Penrose_LHS_Inv = (LHS.Transpose() * LHS).Inv() * LHS.Transpose(); //https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
            //using Mat Ans = Moore_Penrose_LHS_Inv * RHS;
            var optimal = new SetOptimize(LHS, RHS);
            var Ans = optimal.Ans;
            //
            //=====================================================================

            Debug.WriteLine("==  From obj: " + fromF.ObjectId.ToString() + "===  block: " + fromF.Block.BlockId.ToString() + "===  feature: " + fromF.FeatureId.ToString());
            Debug.WriteLine("==  To obj: " + toF.ObjectId.ToString() + "===  block: " + toF.Block.BlockId.ToString() + "===  feature: " + toF.FeatureId.ToString());
            Debug.WriteLine("");
            Debug.WriteLine("Points");
            Debug.WriteLine(adjustedPoints[0].X.ToString() + ", " + adjustedPoints[0].Y.ToString() + ", " + adjustedPoints[1].X.ToString() + ", " + adjustedPoints[1].Y.ToString());
            Debug.WriteLine("");

            Debug.WriteLine("Camera Position 1");
            Debug.WriteLine(c1.At<double>(0, 0).ToString() + ", " + c1.At<double>(1, 0).ToString() + ", " + c1.At<double>(2, 0).ToString() + ", ");
            Debug.WriteLine(fromF.Block.RollDeg.ToString() + ", " + fromF.Block.PitchDeg + ", " + fromF.Block.YawDeg);
            Debug.WriteLine("");
            Debug.WriteLine("Camera Position 2");
            Debug.WriteLine(c2.At<double>(0, 0).ToString() + ", " + c2.At<double>(1, 0).ToString() + ", " + c2.At<double>(2, 0).ToString() + ", ");
            Debug.WriteLine(toF.Block.RollDeg.ToString() + ", " + toF.Block.PitchDeg + ", " + toF.Block.YawDeg);

            Debug.WriteLine("");
            Debug.WriteLine("Ray 1 & Ray 2");
            for (int j = 0; j < 6; j++)
            {
                Debug.Write(Math.Round(-StackD.At<double>(j, 0), 6) + ", ");
            }
            Debug.WriteLine("");
            Debug.WriteLine("Params");
            for (int j = 3; j < 5; j++)
            {
                Debug.Write(Math.Round(Ans[j], 6) + ", ");
            }
            Debug.WriteLine("");

            Debug.WriteLine("");
            Debug.WriteLine("Object calc:");

            for (int j = 0; j < 3; j++)
            {
                Debug.WriteLine(Math.Round(Ans[j], 3));
            }
            Debug.WriteLine("");
            Debug.WriteLine("---------");

            //            Debug.WriteLine(Ans1.At<double>(0, 0) + ", " + Ans1.At<double>(1, 0) + ", " + Ans1.At<double>(2, 0) + ", " + Ans2.At<double>(0, 0) + ", " + Ans2.At<double>(1, 0) + ", " + Ans2.At<double>(2, 0));

            //=====================================================================
            if (optimal.terminationtype == 2 || optimal.terminationtype == 7)
            {
                return Ans;
            }
            else
            {
                return null;
            }
        }
    }
    public class SetOptimize
    {
        private Mat A;
        private Mat b;
        public double[] Ans;
        public int terminationtype;
        public SetOptimize(Mat first, Mat second)
        {
            A = first;
            b = second;
            int M = 6; // 3*2 = number of dimensions*number of images
            int N = 5; // x, y, z, l1, l2
            double[] bndl = new double[5] { -50, 0, -50, -System.Double.PositiveInfinity, -System.Double.PositiveInfinity };
            double[] bndu = new double[5] { 1200, b.At<double>(1,0)-60, 600, System.Double.PositiveInfinity, System.Double.PositiveInfinity };
            double[] s = new double[5] { 60, 60, 60, 30, 30 }; //scale
            double[] x = new double[5] { b.At<double>(0, 0), b.At<double>(1, 0)-90, b.At<double>(2, 0), 1, 1 };
            double EpsX = 0; // 0.0001;
            int MaxIts = 500; // 0 means no limit
            alglib.nlsstate state;
            alglib.nlsreport rep;
            alglib.nlscreatedfo(N,M,x, out state);
            alglib.nlssetcond(state, EpsX, MaxIts);
            alglib.nlssetscale(state, s);
            alglib.nlssetbc(state, bndl, bndu);
            alglib.nlssetalgodfolsa(state,3);
            alglib.nlsoptimize(state, function1_fvec, null, null);
            alglib.nlsresults(state, out x, out rep);
            Debug.WriteLine("{0}", alglib.ap.format(x, 3));
            Debug.WriteLine("{0}", rep.terminationtype);
            Ans = x;
            terminationtype = rep.terminationtype;
        }
        public void function1_fvec(double[] x, double[] fi, object obj)
        {
            //
            // this callback calculates
            // sum over x of ((Ax-b)^2)
            // where each f_i corresponds to one element of the sum
            //
            for (int i = 0; i < fi.Length; i++)
            {
                fi[i] = 0;
                for (int j = 0; j < x.Length; j++)
                {
                    fi[i] = fi[i] + A.At<double>(i, j) * x[j];
                }
                fi[i] = fi[i] - b.At<double>(i);
                fi[i] = fi[i] * fi[i];
            }
        }
    } 
}

    

