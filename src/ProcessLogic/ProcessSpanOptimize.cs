// Copyright SkyComb Limited 2025. All rights reserved. 

using Accord.Math;
using OpenCvSharp;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using System.Diagnostics;
using System.Windows.Forms;


namespace SkyCombImage.ProcessLogic
{
    // ProcessSpanOptimize relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpanOptimize analyses ProcessObjects to find their location based on a triangulation of the drone's position and the object's image location in two images.

    public class SpanOptimize
    {
        // Parent process
        // Camera information
        public static double camViewAngle = 15;
        public static CameraIntrinsic intrinsic = ProcessConfigModel.CameraIntrinsic; // Copy the static Lennard Spark drone camera information
        public static int FramesBehind = 0; // Number of frames behind the current block to use for drone location
        public SpanOptimize(ProcessObjList processObjs, GroundData groundInfo, TextBox? output = null)
        {
            output?.ResetText();
            foreach (var obj in processObjs.Values)
            {
                if (obj.FirstFeature is null) continue;
                var objectFeatures = obj.ProcessFeatures;
                if (objectFeatures.Count < 2) continue; // not enough features to triangulate
                int centralFeatureid = objectFeatures.GetKeyAtIndex(objectFeatures.Count / 2);
                var centralFeature = objectFeatures[centralFeatureid];
                BaseConstants.Assert(centralFeature.ObjectId == obj.ObjectId, "Logic 1");

                // Get feature list for this object that are away from the edge.
                var featurelist = new List<ProcessFeature>();
                ProcessFeature comparefeature = null;
                foreach (var feature in objectFeatures.Values)
                {
                    if (!DistinctFeature(feature, comparefeature)) continue;
                    featurelist.Add(feature);
                    comparefeature = feature;
                }
                if (featurelist.Count < 2)
                {
                    // not enough features to triangulate
                    output?.AppendText($"Not enough features for object {obj.Name}\r\n");
                    continue;
                }

                output?.AppendText($"Outcome for object {obj.Name}\r\n");

                var result = new SetSolve(obj, featurelist, centralFeature, output);
                if (result.Ans != null && result.terminationtype > 0)
                {
                    output?.AppendText($"PQ height: {Math.Round(obj.HeightM, 3)}\r\n");
                    obj.LocationM = new DroneLocation((float)result.Ans[1], (float)result.Ans[0]); // Northing is stored first in location
                    obj.HeightM = (float)result.Ans[2] - groundInfo.DemModel.GetElevationByDroneLocn(obj.LocationM);
                    output?.AppendText($"NQ height: {Math.Round(obj.HeightM, 3)}\r\n");
                    foreach (var feature in featurelist)
                        // These have been filled in fillFeatureResults, we are now changing the values to be relative to the ground
                        feature.Set_LocationM_HeightM(
                            feature.LocationM,
                            feature.HeightM - groundInfo.DemModel.GetElevationByDroneLocn(feature.LocationM));
                }
            }
        }
        public bool DistinctFeature(ProcessFeature feature, ProcessFeature? compare)
        // This is used to determine if the feature is distinct enough from the previous feature, and away from the edge.
        // In addition, it nullifies the PQ location data if it is not distinct enough.
        // Instead of converting the pixelbox values, we've multiplied the intrinsic dimensions by 2.
        {
            if (!feature.Significant) return false;
            else if ((compare == null) && !((feature.PixelBox.X <= 1) || (feature.PixelBox.X + feature.PixelBox.Width >= intrinsic.ImageWidth * 2) || (feature.PixelBox.Y <= 1) || (feature.PixelBox.Y + feature.PixelBox.Height >= intrinsic.ImageHeight * 2)))
                return true; // first feature that is far enough from the edge
            else if ((compare != null) && (Math.Abs(feature.PixelBox.X - compare.PixelBox.X + feature.PixelBox.Width / 2 - compare.PixelBox.Width / 2) >= 2) && (Math.Abs(feature.PixelBox.Y - compare.PixelBox.Y + feature.PixelBox.Height / 2 - compare.PixelBox.Height / 2) >= 2))
                return true; // different enough from compare

            // Remove dead-reckoning location and height
            feature.Set_LocationM_HeightM();
            return false;
        }
        private static Mat CreateRotationMatrix(double rollDegrees, double pitchDegrees, double yawDegrees)
        {
            // Using camera drone information, determine camera heading/position
            // Convert angles to radians
            double roll = rollDegrees * Math.PI / 180.0;
            double pitch = (90 + pitchDegrees) * Math.PI / 180.0;
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
        public class SetSolve
        {
            private double[][]? Direction;
            private double[]? CamPosn;
            public double[]? Ans = null;
            public int terminationtype = 1;
            public double camHeight = 90; // Camera height in meters
            public double[]? GetDroneInfFromFeature(ProcessFeature feature)
            { // No need for checking because this is done in the parent class
                var thisblock = feature.Block;

                return new double[] { thisblock.DroneLocnM.EastingM, thisblock.DroneLocnM.NorthingM, thisblock.AltitudeM
                    , thisblock.RollDeg, thisblock.PitchDeg, thisblock.YawDeg};
            }
            public static double middlePixel(double x, double width, double axisLength)
            {
                // Convert middle pixel to camera coordinate system
                // This is necessary because the origin for the image is in the top right corner ???!!!
                //return (axisLength - (x + width / 2) / 2);
                return ((x + width / 2) / 2);
            }
            public void BuildMatrices(List<ProcessFeature> featurelist)
            {
                // This builds the matrices for the optimization of minimize:
                // sum over all points i of:
                // best final location - (best point location) 
                // = best final location - (camera location at point(i) + lambda(i)*direction(i))
                // CamPosn = camera location at point i, with three dimensions at each point
                // Direction = (identity | for each i: ( 0(i-1) | ray(i) | 0(i+1) ))
                int M = featurelist.Count;
                CamPosn = new double[M * 3];
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
                    var adjustedPoint = new Point2d(middlePixel(feature.PixelBox.X, feature.PixelBox.Width, intrinsic.ImageWidth), middlePixel(feature.PixelBox.Y, feature.PixelBox.Height, intrinsic.ImageHeight));
                    using Mat Ray = PointDirection(adjustedPoint, R);
                    for (int j = 0; j < 3; j++) // for each of easting, height, northing
                    {
                        Direction[i * 3 + j] = new double[3 + M];
                        // Identity matrix for the x0, x1, x2 final result:
                        Direction[i * 3 + j][j] = 1;
                        // the ray d(ij) for the lambda. The first 3 cols are reserved for the identity, then each subsequent column for each point:
                        Direction[i * 3 + j][3 + i] = Ray.At<double>(j, 0);
                    }
                }
            }
            public static Mat PointDirection(Point2d featpoint, Mat R)
            {
                using Mat PixelPoint = new Mat(3, 1, MatType.CV_64F);
                FillMat(PixelPoint, new double[] { featpoint.X, featpoint.Y, 1 });
                return R.Inv() * intrinsic.K.Inv() * PixelPoint;
            }
            public SetSolve(ProcessObject po, List<ProcessFeature> featurelist, ProcessFeature midF, TextBox? output = null)
            {
                BuildMatrices(featurelist);
                var frames = featurelist.Count;
                int M = frames * 3; // 3*2 = number of dimensions*number of features
                int N = frames + 3; // x, y, z, plus one for each feature
                PseudoInverse(Direction, CamPosn);
                //Optimise(M,N,midF);
                fillFeatureResults(featurelist, po.ObjectId);
                output?.AppendText($"Northing: {Math.Round(Ans[1], 3)}\r\nEasting: {Math.Round(Ans[0], 3)}\r\nAltitude: {Math.Round(Ans[2], 3)}\r\n");
                output?.AppendText($"PQ Northing: {Math.Round(po.LocationM.NorthingM, 3)}\r\nEasting: {Math.Round(po.LocationM.EastingM, 3)}\r\n");
                output?.AppendText($"Current objective ({SquareResult(M)}).\r\n");
            }
            public void PseudoInverse(double[][] A, double[] C)
            {
                // This is a pseudo inverse of the matrix A, multiplied by C
                // Ans = A^T * (A * A^T)^-1 * C
                // Only stable for small matrices
                var At = A.Transpose();
                var AtA = MMultiply(At, A);
                var AtAInv = AtA.Inverse();
                var AtAInvAt = MMultiply(AtAInv, At);
                Ans = AtAInvAt.Dot(C);
            }
            public void Optimise(int M, int N, ProcessFeature midF)
            {
                double[] bndl = new double[N];
                double[] bndu = new double[N];
                double[] s = new double[N];
                double[] x = new double[N];
                var camRadius = Math.Tan((90 - Math.Abs(midF.Block.PitchDeg) + (camViewAngle / 2)) * (Math.PI / 180.0)) * camHeight;

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

                    feature.Set_LocationM_HeightM(
                        new DroneLocation((float)vector[1], (float)vector[0]),
                        (float)vector[2]);
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
                return (long)Math.Sqrt(result);
            }
            public static double[][] MMultiply(double[][] A, double[][] B)
            {
                int rowsA = A.Length;
                int colsA = A[0].Length;
                int colsB = B[0].Length;
                double[][] result = new double[rowsA][];
                for (int i = 0; i < rowsA; i++)
                {
                    result[i] = new double[colsB];
                    for (int j = 0; j < colsB; j++)
                    {
                        result[i][j] = 0;
                        for (int k = 0; k < colsA; k++)
                        {
                            result[i][j] += A[i][k] * B[k][j];
                        }
                    }
                }
                return result;
            }
        }
    }

}



