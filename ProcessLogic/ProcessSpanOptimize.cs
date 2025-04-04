// Copyright SkyComb Limited 2025. All rights reserved. 

using Accord.Math;
using OpenCvSharp;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System;
using System.Diagnostics;
using System.Diagnostics.Eventing.Reader;
using System.Security.Cryptography.X509Certificates;
using System.Windows.Forms;
using static alglib;
using static System.Runtime.InteropServices.JavaScript.JSType;



namespace SkyCombImage.ProcessLogic
{
    // ProcessSpanOptimize relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpanOptimize analyses ProcessObjects to find their location based on a triangulation of the drone's position and the object's image location in two images.

    public class SpanOptimize
    {
        // Parent process
        private ProcessAll Process { get; }
        // Camera information
        public static double camViewAngle = 15;
        public static CameraIntrinsic intrinsic = ProcessConfigModel.intrinsic; // Copy the static Lennard Spark drone camera information
        public static int FramesBehind = 0; // Number of frames behind the current block to use for drone location
        public SpanOptimize(ProcessAll process,TextBox? output = null)
        {
            Process = process;
            output?.ResetText();
            var ObjList = Process.ProcessObjects;
            var totBlocks = Process.Blocks.Count;
            if (false)
            { // for debugging. Ensure that the time chosen includes features in the blocks.
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
            foreach (var obj in ObjList.Values)
            {
                if (obj.FirstFeature is null) continue;
                var objectFeatures = obj.ProcessFeatures;
                if (objectFeatures.Count < 2) continue; // not enough features to triangulate
                int centralFeatureid = objectFeatures.GetKeyAtIndex(objectFeatures.Count / 2);
                var centralFeature = objectFeatures[centralFeatureid];
                BaseConstants.Assert(centralFeature.ObjectId == obj.ObjectId, "Logic 1"); 

                var featurelist = new List<ProcessFeature>();
                foreach (var feature in objectFeatures.Values)
                {
                    if (!DistinctFeature(feature)) continue;
                    BaseConstants.Assert(feature.ObjectId == obj.ObjectId, "Logic 2");
                    featurelist.Add(feature);
                }
                if (featurelist.Count < 2) continue;
                if (featurelist[0].BlockId - FramesBehind <= 0 || featurelist[^1].BlockId - FramesBehind > totBlocks)
                {
                    Debug.WriteLine($"Start analysis earlier/later for object {obj.Name}");
                    output?.AppendText($"\nStart analysis {((FramesBehind < 0)? "later":"earlier")} for object {obj.Name}\r\n");
                    continue;
                }
                Debug.WriteLine($"Outcome for object {obj.Name} with {FramesBehind} frames behind");
                if (output != null) output.AppendText($"Outcome for object {obj.Name} with {FramesBehind} frames behind\r\n");

                var result = new SetOptimize(obj.ObjectId, featurelist, centralFeature, Process, output);
                if (result.Ans == null) continue;
                if (result.terminationtype > 0)
                {
                    obj.LocationM = new DroneLocation((float)result.Ans[1], (float)result.Ans[0]); // Northing is stored first in location
                    obj.HeightM = (float)result.Ans[2] - Process.GroundData.DemModel.GetElevationByDroneLocn(obj.LocationM); 

                    foreach (var feature in obj.ProcessFeatures.Values)
                    // These have been filled in fillFeatureResults, we are now changing the values to be relative to the ground
                    {
                        BaseConstants.Assert(feature.ObjectId == obj.ObjectId, "Logic 3");

                        if (featurelist.Contains(feature))
                            feature.HeightM = feature.HeightM - process.GroundData.DemModel.GetElevationByDroneLocn(feature.LocationM);
                        else
                        {
                            feature.HeightM = -999;
                            feature.LocationM = null;
                        }
                    }
                }
            }
        }
        public bool DistinctFeature(ProcessFeature feature)
        {
            if (!feature.Significant) return false;
            if ((feature.PixelBox.X <= 1) || (feature.PixelBox.X + feature.PixelBox.Width >= intrinsic.ImageWidth) || (feature.PixelBox.Y <= 1) || (feature.PixelBox.Y + feature.PixelBox.Height >= intrinsic.ImageHeight)) return false; //too close to edge
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
            return Ry * Rz * Rx;
        }
        private static void FillMat(Mat mat, double[] values)
        {
            for (int i = 0; i < values.Length; i++)
            {
                mat.At<double>(i) = values[i];
            }
        }
        public class SetOptimize
        {
            private double[][]? Direction;
            private double[]? CamPosn;
            private ProcessAll Process;
            public double[]? Ans = null;
            public int terminationtype;
            public double camHeight = 90; // Camera height in meters
            public double[]? GetDroneInfFromFeature(ProcessFeature feature)
            { // No need for checking because this is done in the parent class
                var beforeBlock = Process.Blocks[(int) feature.BlockId - FramesBehind];

                return new double[] { beforeBlock.DroneLocnM.EastingM, beforeBlock.DroneLocnM.NorthingM, beforeBlock.AltitudeM
                    , beforeBlock.RollDeg, beforeBlock.PitchDeg, beforeBlock.YawDeg};
            }
            public void BuildMatrices(List<ProcessFeature> featurelist)
            {
                // This builds the matrices for the optimization of minimize:
                // sum over all points i of:
                // best final location - (best point location) 
                // = best final location - (camera location at point(i) + lambda(i)*direction(i))
                // RHS = camera location at point i, with three dimensions at each point
                // LHS = identity - direction(i)
                int M = featurelist.Count;
                CamPosn = new double[M*3];           
                Direction = new double[M * 3][];
                for (int i = 0; i < M; i++) // for each point
                {
                    var feature = featurelist[i];
                    var dronePosture = GetDroneInfFromFeature(feature);
                    if (dronePosture == null) continue;
                    CamPosn[i * 3] = dronePosture[0]; 
                    CamPosn[i * 3 + 1] = dronePosture[1]; 
                    CamPosn[i * 3 + 2] = dronePosture[2]; 
                    using Mat R = CreateRotationMatrix(dronePosture[3], dronePosture[4], dronePosture[5]);
                    var adjustedPoint = new Point2d(feature.PixelBox.X  + feature.PixelBox.Width / 2, feature.PixelBox.Y + feature.PixelBox.Height / 2);
                    using Mat Ray = PointDirection(adjustedPoint, R);
                    for (int j = 0; j < 3; j++) // for each of easting, height, northing
                    {
                        Direction[i * 3 + j] = new double[3+M];
                        // Identity matrix for the x0, x1, x2 final result:
                        Direction[i * 3 + j][j] = 1;
                        // the ray d(ij) for the lambda. The first 3 cols are reserved for the identity, then each subsequent column for each point:
                        Direction[i * 3 + j][3 + i] = Ray.At<double>(j,0); 
                    }
                }    
            }
            public static Mat PointDirection(Point2d featpoint, Mat R)
            {
                using Mat PixelPoint = new Mat(3, 1, MatType.CV_64F);
                FillMat(PixelPoint, new double[] { featpoint.X, featpoint.Y, 1 });
                return R.Inv() * intrinsic.K.Inv() * PixelPoint;
            }
            public static bool RightOfCamera(ProcessFeature feature) => feature.PixelBox.X > intrinsic.ImageWidth; // Because of the halving, imagewidth would normally be divided by 2;
            public SetOptimize(int objectId, List<ProcessFeature> featurelist, ProcessFeature midF, ProcessAll process, TextBox? output = null)
            {
                Process = process;
                BuildMatrices(featurelist);
                var frames = featurelist.Count;
                int M = frames * 3; // 3*2 = number of dimensions*number of features
                int N = frames + 3; // x, y, z, plus one for each feature
                double[] bndl = new double[N];
                double[] bndu = new double[N];
                double[] s = new double[N];
                double[] x = new double[N];
                var camRadius = Math.Tan((90 - Math.Abs(midF.Block.PitchDeg) + (camViewAngle / 2)) * (Math.PI / 180.0)) * camHeight;
                var midFPosture = GetDroneInfFromFeature(midF);
                if (midFPosture == null) return;

                bndl[0] = midF.LocationM.EastingM - 15;
                bndl[2] = midF.HeightM - 30;
                bndl[1] = midF.LocationM.NorthingM - 15;
                bndu[0] = midF.LocationM.EastingM + 15;
                bndu[2] = midF.HeightM + 30;
                bndu[1] = midF.LocationM.NorthingM + 15;
                s[0] = 30;
                s[1] = 30;
                s[2] = 30;
                x[0] = midF.LocationM.EastingM; // PQ solution
                x[2] = 15;
                x[1] = midF.LocationM.NorthingM; // PQ solution
                for (int i = 3; i < N; i++)
                {
                    bndl[i] = -1;
                    bndu[i] = 1;
                    s[i] = 2;
                    x[i] = 0;
                }
                double EpsX = 0.0001;
                int MaxIts = 5000; // 0 means no limit
                alglib.nlsstate state;
                alglib.nlsreport rep;
                alglib.nlscreatedfo(N, M, x, out state);
                alglib.nlssetcond(state, EpsX, MaxIts);
                alglib.nlssetscale(state, s);
                //alglib.nlssetbc(state, bndl, bndu);
                alglib.nlssetalgodfolsa(state, 3);
                alglib.nlsoptimize(state, function1_fvec, null, null);
                alglib.nlsresults(state, out x, out rep);
                Debug.WriteLine("{0}", alglib.ap.format(x, 3));
                switch (rep.terminationtype)
                {
                    case 1:
                        Debug.WriteLine("Termination: Relative function improvement is no more than EpsF.");
                        break;
                    case 2:
                        Debug.WriteLine($"Termination: Relative step is no more than {EpsX}.");
                        break;
                    case 4:
                        Debug.WriteLine("Termination: Gradient norm is no more than EpsG.");
                        break;
                    case 5:
                        Debug.WriteLine($"Termination: Maximum number of iterations reached ({MaxIts}).");
                        break;
                    case 7:
                        Debug.WriteLine("Termination: No further improvement possible with current stopping conditions. Best point found.");
                        break;
                    case 8:
                        Debug.WriteLine("Termination: Process terminated by user.");
                        break;
                    default:
                        Debug.WriteLine($"Termination: Unknown termination type ({rep.terminationtype}).");
                        break;
                }
                Ans = x;
                terminationtype = rep.terminationtype;
                fillFeatureResults(featurelist, objectId);
                terminationtype = rep.terminationtype;
                output?.AppendText($"Northing: {Math.Round(x[1], 3)}\r\nEasting: {Math.Round(x[0], 3)}\r\nAltitude: {Math.Round(x[2], 3)}\r\n");
                Debug.WriteLine($"Current objective ({SquareResult(M)}).");
                if (output != null) output.AppendText($"Current objective ({SquareResult(M)}).\r\n");
            }
            public void fillFeatureResults(List<ProcessFeature> featurelist, int objectId)
            {
                //
                // this callback calculates camera location - lambda * direction 
                // lambda = Ans[feature row], direction = direction[feature column and rows], camera location = CamPosn[feature rows]
                // where b is the camera location, and A is the negative direction of the feature
                //
                double[] vector;
                int i = 0;
                foreach (var feature in featurelist)
                {
                    BaseConstants.Assert(feature.ObjectId == objectId, "Logic 3");

                    vector = new double[3] { CamPosn[i*3] - Direction[i*3][3 + i]*Ans[3 + i]
                        , CamPosn[i*3 + 1] - Direction[i*3 + 1][3 + i]*Ans[3 + i]
                        , CamPosn[i*3 + 2] - Direction[i*3 + 2][3 + i]*Ans[3 + i] };
                    feature.LocationM = new DroneLocation((float)vector[1], (float)vector[0]);
                    feature.HeightM = (float)vector[2];
                    i++;
                }

            }
            public void function1_fvec(double[] x, double[] fi, object obj)
            {
                //
                // this callback calculates
                // f(i) = (sum over x:(I|CamPosn[i][x])-RHS[i]) ^ 2
                // where each f_i corresponds to one element of the sum
                //
                for (int i = 0; i < fi.Length; i++)
                {
                    fi[i] = 0;
                    for (int j = 0; j < x.Length; j++)
                    {
                        fi[i] = fi[i] + Direction[i][j] * x[j];
                    }
                    fi[i] = fi[i] - CamPosn[i];
                    fi[i] = fi[i] * fi[i];
                }
            }
            public long SquareResult(int M)
            {
                //
                // this callback calculates the sum of squares of elements of the vector 
                //
                double result = 0;
                double[] fi = new double[M];
                object obj = null;
                function1_fvec(Ans, fi, obj);
                for (int i = 0; i < M; i++)
                    result += fi[i];
                return (long) Math.Sqrt(result);
            }
        }
    }
}

    

