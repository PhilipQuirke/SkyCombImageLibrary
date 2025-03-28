// Copyright SkyComb Limited 2025. All rights reserved. 

using Accord.Math;
using OpenCvSharp;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Diagnostics;
using System.Security.Cryptography.X509Certificates;



namespace SkyCombImage.ProcessLogic
{
    // ProcessSpanOptimize relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpanOptimize analyses ProcessObjects to find their location based on a triangulation of the drone's position and the object's image location in two images.

    public class SpanOptimize
    {
        // Parent process
        private ProcessAll Process { get; }
        private int CompareInterval { get; }
        // Camera information
        public static double camViewAngle = 15;
        public static CameraIntrinsic intrinsic = ProcessConfigModel.intrinsic; // Copy the static Lennard Spark drone camera information
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
                    int objectFeatures = Process.ProcessObjects[objid].ProcessFeatures.Count;
                    if (objectFeatures < 2) continue; // not enough features to triangulate
                    int centralFeatureid = Process.ProcessObjects[objid].ProcessFeatures.GetKeyAtIndex(objectFeatures/2);
                    var centralFeature = Process.ProcessObjects[objid].ProcessFeatures[centralFeatureid];

                    for (var idC = compareBlock.MinFeatureId; idC <= compareBlock.MaxFeatureId; idC++)
                    {
                        Process.ProcessFeatures.TryGetValue(idC, out var featureC);
                        if (featureC == null)
                            continue;

                        var objidC = featureC.ObjectId;
                        if (objid != objidC) continue;
                        if (!DistinctFeature(featureC)) continue;

                        var result = GetPoint(feature, featureC, centralFeature);
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
        private static Mat CreateRotationMatrix(double rollDegrees, double pitchDegrees, double yawDegrees)
        {
            // Using camera drone information, determine camera heading/position
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
        public double[] GetPoint(ProcessFeature fromF, ProcessFeature toF, ProcessFeature midF)
        {
            var features = GetObjectPoints(fromF, toF);
            var optimal = new SetOptimize(features, midF);
            if (optimal.terminationtype == 2 || optimal.terminationtype == 7 || optimal.terminationtype == 5)
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

        public class SetOptimize
        {
            private double[][] LHS;
            private double[] RHS;
            public double[] Ans;
            public int terminationtype;
            public double camHeight = 100; // Camera height in meters
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
            public Mat PointDirection(Point2d featpoint, Mat R)
            {
                using Mat PixelPoint = new Mat(3, 1, MatType.CV_64F);
                FillMat(PixelPoint, new double[] { featpoint.X, featpoint.Y, 1 });
                return R.Inv() * intrinsic.K.Inv() * PixelPoint;
            }
            public bool RightOfCamera(ProcessFeature feature) => feature.PixelBox.X > intrinsic.ImageWidth; // Because of the halving, imagewidth would normally be divided by 2;
            public SetOptimize(List<ProcessFeature> featurelist, ProcessFeature midF)
            {
                BuildMatrices(featurelist);
                var frames = featurelist.Count;
                int M = frames * 3; // 3*2 = number of dimensions*number of features
                int N = frames + 3; // x, y, z, plus one for each feature
                double[] bndl = new double[N];
                double[] bndu = new double[N];
                double[] s = new double[N];
                double[] x = new double[N];
                var camRadius = Math.Tan((90 - Math.Abs(midF.Block.PitchDeg) + (camViewAngle / 2)) * (Math.PI / 180.0)) * camHeight;
                bndl[0] = midF.Block.DroneLocnM.EastingM - camRadius;
                bndl[1] = 0;
                bndl[2] = midF.Block.DroneLocnM.NorthingM - camRadius;
                bndu[0] = midF.Block.DroneLocnM.EastingM + camRadius;
                bndu[1] = midF.Block.AltitudeM - 60;
                bndu[2] = midF.Block.DroneLocnM.NorthingM + camRadius;
                s[0] = 80;
                s[1] = 80;
                s[2] = 80;
                x[0] = midF.Block.DroneLocnM.EastingM;
                x[1] = midF.Block.AltitudeM-camHeight;
                x[2] = midF.Block.DroneLocnM.NorthingM;
                for (int i = 3; i < N; i++)
                {
                    bndl[i] = -System.Double.PositiveInfinity;
                    bndu[i] = System.Double.PositiveInfinity;
                    s[i] = 60;
                    x[i] = 1;
                }
                double EpsX = 0.0001;
                int MaxIts = 2000; // 0 means no limit
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
                        fi[i] = fi[i] + LHS[i][j] * x[j];
                    }
                    fi[i] = fi[i] - RHS[i];
                    fi[i] = fi[i] * fi[i];
                }
            }
        }
    }
}

    

