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
using System.Net.Mail;
using System.Windows.Forms; // temporary for unit test


namespace SkyCombImage.ProcessLogic
{
    // ProcessSpanOptimize relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpanOptimize analyses ProcessObjects to find their location based on a triangulation of the drone's position and the object's image location in two images.

    public class SpanOptimize
    {
        // Parent process
        private ProcessAll Process { get; }
        private int CompareInterval { get; }
        private double[][] LHS;
        private double[] RHS;

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
            BuildMatrices(GetObjectPoints(fromF, toF));
            var optimal = new SetOptimize(LHS, RHS);
            if (optimal.terminationtype == 2 || optimal.terminationtype == 7)
            {
                return optimal.Ans;
            }
            else
            {
                return null;
            }
        }
        public List<ProcessFeature> GetObjectPoints(ProcessFeature fromf, ProcessFeature tof) {
            var featurelist = new List<ProcessFeature>();
            var obj = fromf.ObjectId;
            var firstFeat = fromf.Block.BlockId;
            var lastfeat = tof.Block.BlockId;
            var featlist = Process.ProcessObjects[obj].ProcessFeatures.Values;
            int featcount = 0;
            Point2d adjustedPoint;
            foreach (var feature in featlist)
            {
                if ((feature.BlockId > lastfeat) || (feature.BlockId < firstFeat)) continue;
                if (!DistinctFeature(feature)) continue;
                featurelist.Add(feature);
                featcount++;
            }
            return featurelist;
        }
        public void BuildMatrices(List<ProcessFeature> featurelist)
        {
            int M = featurelist.Count;
            RHS = new double[M*3];           
            LHS = new double[M * 3][];
            for (int i = 0; i < M; i++)
            {
                var feature = featurelist[i];
                RHS[i * 3] = feature.Block.DroneLocnM.EastingM;
                RHS[i * 3 + 1] = feature.Block.AltitudeM;
                RHS[i * 3 + 2] = feature.Block.DroneLocnM.NorthingM;
                using Mat R = CreateRotationMatrix(feature.Block.RollDeg, feature.Block.PitchDeg, feature.Block.YawDeg);
                var adjustedPoint = new Point2d(feature.PixelBox.X / 2 + feature.PixelBox.Width / 4, feature.PixelBox.Y / 2 + feature.PixelBox.Height / 4);
                using Mat Ray = PointDirection(adjustedPoint, R);
                for (int j = 0; j < 3; j++)
                {
                    LHS[i * 3 + j] = new double[3+M];
                    LHS[i * 3 + j][j] = 1;
                    LHS[i * 3 + j][3 + i] = -Ray.At<double>(j,0);
                }
            }
            
        }
        public class SetOptimize
        {
            private double[][] A;
            private double[] b;
            public double[] Ans;
            public int terminationtype;
            public SetOptimize(double[][] first, double[] second)
            {
                A = first;
                b = second;
                int M = 6; // 3*2 = number of dimensions*number of images
                int N = 5; // x, y, z, l1, l2
                double[] bndl = new double[5] { -50, 0, -50, -System.Double.PositiveInfinity, -System.Double.PositiveInfinity };
                double[] bndu = new double[5] { 1800, b[1] - 60, 1200, System.Double.PositiveInfinity, System.Double.PositiveInfinity };
                double[] s = new double[5] { 80, 80, 80, 30, 30 }; //scale
                double[] x = new double[5] { b[0], b[1] - 90, b[2], 1, 1 };
                double EpsX = 0.0001;
                int MaxIts = 500; // 0 means no limit
                alglib.nlsstate state;
                alglib.nlsreport rep;
                alglib.nlscreatedfo(N, M, x, out state);
                alglib.nlssetcond(state, EpsX, MaxIts);
                alglib.nlssetscale(state, s);
                alglib.nlssetbc(state, bndl, bndu);
                alglib.nlssetalgodfolsa(state, 3);
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
                        fi[i] = fi[i] + A[i][j] * x[j];
                    }
                    fi[i] = fi[i] - b[i];
                    fi[i] = fi[i] * fi[i];
                }
            }
        }
    }
}

    

