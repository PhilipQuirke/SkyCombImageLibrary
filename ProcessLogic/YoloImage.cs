// Copyright SkyComb Limited 2024. All rights reserved. 
using System;
using System.Collections.Generic;
using System.Data;
using System.Drawing;
using System.Linq;
using Accord.MachineLearning;
using Accord.MachineLearning.VectorMachines.Learning;
using Accord.MachineLearning.VectorMachines;
using Accord.Statistics.Kernels;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.ProcessLogic
{
    // An image feature detected by YOLO in a single frame
    public class YoloFeatureSeen
    {
        public int BlockId;
        public int FeatureId;
        public Rectangle Box;
    }


    public class YoloFeatureSeenList : List<YoloFeatureSeen>
    {
        public YoloFeatureSeenList() : base() { }


        // Clone this list
        public YoloFeatureSeenList Clone()
        {
            YoloFeatureSeenList clone = new();
            foreach (var feature in this)
                clone.Add(new YoloFeatureSeen { BlockId = feature.BlockId, FeatureId = feature.FeatureId, Box = feature.Box });

            return clone;
        }


        // Return a list of each feature's ID, BlockId (Time frame) and 2D location
        public DataTable ToDataTable()
        {
            DataTable dataTable = new DataTable();
            dataTable.Columns.Add("X", typeof(double));
            dataTable.Columns.Add("Y", typeof(double));
            dataTable.Columns.Add("Time", typeof(double));
            dataTable.Columns.Add("Cluster", typeof(int));
            dataTable.Columns.Add("Feature", typeof(int));

            foreach (var feature in this)
                dataTable.Rows.Add(feature.Box.X, feature.Box.Y, feature.BlockId, 0, feature.FeatureId);

            return dataTable;
        }
    }


    // An object is a collection of features over several frames
    public class YoloObjectSeen
    {
        // The features that make up the object. Implies the first and last frame the object was seen in
        public YoloFeatureSeenList Features;

        public YoloObjectSeen()
        {
            Features = new();
        }
    }


    public class YoloObjectSeenList : List<YoloObjectSeen>
    {
    }


    // A class to track all objects across all frames. 
    // Physical objects may be visible, then obscured, then visible resulting in two ObjectSeen objects. Try to merge them.
    public class YoloTracker
    {
        static public YoloObjectSeenList CalculateObjectsInLeg(double yMovePerTimeSlice, YoloFeatureSeenList features)
        {
            YoloObjectSeenList answer = new();

            try
            {
                if (features.Count > 0)
                {
                    var minFrameId = features[0].BlockId;
                    var maxFrameId = features[^1].BlockId;

                    var dataTable = features.ToDataTable();

                    foreach (DataRow row in dataTable.Rows)
                    {
                        row["Y"] = (double)row["Y"] - (double)row["Time"] * yMovePerTimeSlice;
                    }

                    // Estimate the optimal number of clusters
                    int nClusters = EstimateClustersSilhouette(dataTable, 50);

                    // Initial clustering
                    double[][] data = dataTable.AsEnumerable().Select(row => new double[] { (double)row["X"], (double)row["Y"], (double)row["Time"] }).ToArray();
                    KMeans kmeans = new(nClusters);
                    KMeansClusterCollection clusters = kmeans.Learn(data);
                    int[] clusterLabels = clusters.Decide(data);

                    // Assign cluster labels to each data point
                    for (int i = 0; i < dataTable.Rows.Count; i++)
                    {
                        dataTable.Rows[i]["Cluster"] = clusterLabels[i];
                    }

                    // Adjust clusters to meet the criteria
                    // PQR temp todo var adjustedTable = AdjustClusters(yMovePerTimeSlice, dataTable);
                    var adjustedTable = dataTable;

                    // Validate the clusters
                    //var validationErrors = ValidateClusters(yMovePerTimeSlice, adjustedTable);
                    //Console.WriteLine("Validation Errors:");
                    //foreach (var error in validationErrors)
                    //{
                    //    Console.WriteLine(error);
                    //}

                    // Identify features not contained in the adjusted clusters
                    //var adjustedFeatureIndices = adjustedTable.AsEnumerable().Select(row => dataTable.Rows.IndexOf(row)).ToList();
                    //var allFeatureIndices = Enumerable.Range(0, dataTable.Rows.Count).ToList();
                    //var unclusteredFeatures = allFeatureIndices.Except(adjustedFeatureIndices).ToList();


                    foreach (var cluster in adjustedTable.AsEnumerable().Select(row => row.Field<int>("Cluster")).Distinct())
                    {
                        var objSeen = new YoloObjectSeen();
                        var clusterDf = adjustedTable.AsEnumerable().Where(row => row.Field<int>("Cluster") == cluster).OrderBy(row => row.Field<int>("Feature")).ToList();
                        foreach (var row in clusterDf)
                        {
                            var featureId = row.Field<int>("Feature");
                            var time = row.Field<double>("Time");
                            var x = row.Field<double>("X");
                            var y = row.Field<double>("Y");

                            objSeen.Features.Add(features.First(feature => feature.FeatureId == featureId));
                        }
                        answer.Add(objSeen);
                    }
                }
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("YoloTracker.CalculateObjectsInLeg", ex);
            }

            return answer;
        }


        static int EstimateClustersSilhouette(DataTable data, int maxClusters)
        {
            var X = data.AsEnumerable().Select(row => new double[] { (double)row["X"], (double)row["Y"], (double)row["Time"] }).ToArray();
            List<double> silhouetteScores = new List<double>();

            for (int nClusters = 2; nClusters <= maxClusters; nClusters++)
            {
                var kmeans = new KMeans(nClusters);
                var clusterLabels = kmeans.Learn(X).Decide(X);
                double silhouetteScore = CalculateSilhouetteScore(X, clusterLabels, nClusters);
                silhouetteScores.Add(silhouetteScore);
            }

            int optimalClusters = silhouetteScores.IndexOf(silhouetteScores.Max()) + 2;
            return optimalClusters;
        }

        static double CalculateSilhouetteScore(double[][] X, int[] labels, int nClusters)
        {
            double[] silhouetteScores = new double[X.Length];

            for (int i = 0; i < X.Length; i++)
            {
                int currentLabel = labels[i];

                var sameCluster = X.Where((t, idx) => labels[idx] == currentLabel && idx != i).ToArray();
                var otherCluster = X.Where((t, idx) => labels[idx] != currentLabel).ToArray();

                double a = sameCluster.Length > 0 ? sameCluster.Average(point => Distance(X[i], point)) : 0;
                double b = otherCluster.Length > 0 ? Enumerable.Range(0, otherCluster.Length)
                                                               .GroupBy(idx => labels[Array.IndexOf(X, otherCluster[idx])])
                                                               .Select(group => group.Average(idx => Distance(X[i], otherCluster[idx])))
                                                               .Min() : 0;

                silhouetteScores[i] = (b - a) / Math.Max(a, b);
            }

            return silhouetteScores.Average();
        }


        static double Distance(double[] point1, double[] point2)
        {
            double sum = 0;
            for (int i = 0; i < point1.Length; i++)
            {
                sum += Math.Pow(point1[i] - point2[i], 2);
            }
            return Math.Sqrt(sum);
        }


        static DataTable AdjustClusters(double yMovePerTimeSlice, DataTable orgClusters)
        {
            try { 
                var adjClusters = new List<DataTable>();

                foreach (var orgCluster in orgClusters.AsEnumerable().Select(row => row.Field<int>("Cluster")).Distinct())
                {
                    var clusterRows = orgClusters.AsEnumerable().Where(row => row.Field<int>("Cluster") == orgCluster).OrderBy(row => row.Field<double>("Time")).ToList();
                    var validFeatures = new List<DataRow>();

                    for (int i = 0; i < clusterRows.Count; i++)
                    {
                        if (i == 0 || (Math.Abs(clusterRows[i].Field<double>("Y") - validFeatures.Last().Field<double>("Y")) <= 25 - yMovePerTimeSlice &&
                                        clusterRows[i].Field<double>("Y") - validFeatures.Last().Field<double>("Y") >= -yMovePerTimeSlice &&
                                        Math.Abs(clusterRows[i].Field<double>("X") - validFeatures.Last().Field<double>("X")) <= 10))
                        {
                            if (validFeatures.Count == 0 || clusterRows[i].Field<double>("Time") != validFeatures.Last().Field<double>("Time"))
                            {
                                validFeatures.Add(clusterRows[i]);
                            }
                        }
                    }

                    if (validFeatures.Count > 0)
                    {
                        var tempTable = orgClusters.Clone();
                        foreach (var row in validFeatures)
                        {
                            tempTable.ImportRow(row);
                        }
                        adjClusters.Add(tempTable);
                    }
                }

                DataTable adjustedDf = adjClusters.Count > 0 ? adjClusters.Aggregate((dt1, dt2) => { dt1.Merge(dt2); return dt1; }) : orgClusters.Clone();
                var remainingFeatures = orgClusters.AsEnumerable().Except(adjustedDf.AsEnumerable()).CopyToDataTable();

                while (remainingFeatures.Rows.Count > 0)
                {
                    bool reassigned = false;
                    foreach (DataRow feature in remainingFeatures.Rows)
                    {
                        int closestCluster = -1;
                        double closestDistance = double.MaxValue;

                        foreach (var cluster in adjustedDf.AsEnumerable().Select(row => row.Field<int>("Cluster")).Distinct())
                        {
                            var clusterDf = adjustedDf.AsEnumerable().Where(row => row.Field<int>("Cluster") == cluster).CopyToDataTable();
                            double distance = Math.Sqrt(Math.Pow((double)feature["X"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("X")).Average(), 2) +
                                                        Math.Pow((double)feature["Y"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Y")).Average(), 2));
                            if (distance < closestDistance)
                            {
                                closestCluster = cluster;
                                closestDistance = distance;
                            }
                        }

                        if (closestCluster != -1)
                        {
                            var clusterDf = adjustedDf.AsEnumerable().Where(row => row.Field<int>("Cluster") == closestCluster).CopyToDataTable();
                            if (Math.Abs((double)feature["Y"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Y")).Max()) <= 15 &&
                                (double)feature["Y"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Y")).Max() >= -10 &&
                                Math.Abs((double)feature["X"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("X")).Average()) <= 10 &&
                                (double)feature["Time"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Time")).Max() <= 5)
                            {
                                adjustedDf.ImportRow(feature);
                                remainingFeatures.Rows.Remove(feature);
                                reassigned = true;
                            }
                        }
                    }

                    if (!reassigned)
                    {
                        break;
                    }
                }

                return adjustedDf;
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("YoloTracker.AdjustClusters", ex);
            }
        }


        static List<string> ValidateClusters(double yMovePerTimeSlice, DataTable df)
        {
            var validationErrors = new List<string>();

            foreach (var cluster in df.AsEnumerable().Select(row => row.Field<int>("Cluster")).Distinct())
            {
                var clusterDf = df.AsEnumerable().Where(row => row.Field<int>("Cluster") == cluster).OrderBy(row => row.Field<string>("Feature")).ToList();
                DataRow previousRow = null;

                foreach (var row in clusterDf)
                {
                    if (previousRow != null)
                    {
                        if ((double)row["Time"] < (double)previousRow["Time"])
                        {
                            validationErrors.Add($"Cluster {cluster}, Feature {row["Feature"]}: Time not monotonically increasing.");
                        }
                        if (Math.Abs((double)row["Y"] - (double)previousRow["Y"]) > 25 - yMovePerTimeSlice || (double)row["Y"] - (double)previousRow["Y"] < -yMovePerTimeSlice)
                        {
                            validationErrors.Add($"Cluster {cluster}, Feature {row["Feature"]}: Y change out of allowed range.");
                        }
                        if ((double)row["Time"] - (double)previousRow["Time"] > 5)
                        {
                            validationErrors.Add($"Cluster {cluster}, Feature {row["Feature"]}: Time gap greater than 5.");
                        }
                    }

                    previousRow = row;
                }
            }

            return validationErrors;
        }
    }
}
