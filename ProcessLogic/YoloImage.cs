// Copyright SkyComb Limited 2024. All rights reserved. 
using System.Data;
using System.Drawing;
using Accord.MachineLearning;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;


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

                    var validationErrors = ValidateClusters(yMovePerTimeSlice, dataTable);


                    // Adjust clusters to meet the criteria
                    var adjustedTable = AdjustClusters(yMovePerTimeSlice, dataTable, features.Count);

                    // Validate the clusters
                    validationErrors = ValidateClusters(yMovePerTimeSlice, adjustedTable);

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


        // Are these two features similar enough to be associated with each other?
        static bool IsGoodNextFeature(double yMovePerTimeSlice, double lastTime, double lastX, double lastY, DataRow nextFeature)
        {
            double maxYPixels = ProcessConfigModel.YoloMaxYPixelsDeltaPerFrame - yMovePerTimeSlice;

            return
                // The two features need to be close in Y direction
                (Math.Abs(nextFeature.Field<double>("Y") - lastY) <= maxYPixels &&
                nextFeature.Field<double>("Y") - lastY >= -yMovePerTimeSlice &&
                // The two features need to be close in X direction
                Math.Abs(nextFeature.Field<double>("X") - lastX) <= ProcessConfigModel.YoloMaxXPixelsDeltaPerCluster) &&
                // The two features must be time ordered and not too far apart
                nextFeature.Field<double>("Time") > lastTime &&
                Math.Abs(nextFeature.Field<double>("Time") - lastTime) <= ProcessConfigModel.YoloMaxTimeGap;
        }


        // Are these two features similar enough to be associated with each other?
        static bool IsGoodNextFeature(double yMovePerTimeSlice, DataRow lastFeature, DataRow nextFeature)
        {
            return IsGoodNextFeature(yMovePerTimeSlice, lastFeature.Field<double>("Time"), lastFeature.Field<double>("X"), lastFeature.Field<double>("Y"), nextFeature);
        }


        static DataTable AdjustClusters(double yMovePerTimeSlice, DataTable orgClusters, int orgNumFeatures)
        {
            try
            {
                var newClusters = new List<DataTable>();
                var newNumFeatures = 0;

                // Shrink the orgClusters by removing any features that fail the NextFeatureGood test
                foreach (var orgCluster in orgClusters.AsEnumerable().Select(row => row.Field<int>("Cluster")).Distinct())
                {
                    var clusterRows = orgClusters.AsEnumerable().Where(row => row.Field<int>("Cluster") == orgCluster).OrderBy(row => row.Field<double>("Time")).ToList();
                    var validFeatures = new List<DataRow>();

                    DataRow? lastFeature = null;
                    for (int i = 0; i < clusterRows.Count; i++)
                    {
                        DataRow nextFeature = clusterRows[i];
                        if (i == 0)
                            validFeatures.Add(nextFeature);
                        else if( IsGoodNextFeature(yMovePerTimeSlice, lastFeature, nextFeature) )
                            validFeatures.Add(nextFeature);
                        lastFeature = nextFeature;
                    }

                    if (validFeatures.Count > 0)
                    {
                        var tempTable = orgClusters.Clone();
                        foreach (var row in validFeatures)
                        {
                            tempTable.ImportRow(row);
                            newNumFeatures++;
                        }
                        newClusters.Add(tempTable);
                    }
                }

                // Find the orphaned features 
                DataTable adjustedDf = newClusters.Count > 0 ? newClusters.Aggregate((dt1, dt2) => { dt1.Merge(dt2); return dt1; }) : orgClusters.Clone();
                var orphanedFeaturesSet = orgClusters.AsEnumerable()
                        .Where(row => !adjustedDf.AsEnumerable().Any(adjustedRow => adjustedRow.Field<int>("Feature") == row.Field<int>("Feature")));
                if( orphanedFeaturesSet.Count() == 0 )
                    return adjustedDf;

                double maxYPixels = ProcessConfigModel.YoloMaxYPixelsDeltaPerFrame - yMovePerTimeSlice;

                // Try to reassign the orphaned features to an existing cluster
                var orphanedFeatures = orphanedFeaturesSet.CopyToDataTable();
                while (orphanedFeatures.Rows.Count > 0)
                {
                    bool reassigned = false;
                    var rowsToRemove = new List<DataRow>();

                    foreach (DataRow feature in orphanedFeatures.Rows)
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
                            // This is the best code:
                            if (Math.Abs((double)feature["Y"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Y")).Max()) <= maxYPixels &&
                                (double)feature["Y"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Y")).Max() >= -yMovePerTimeSlice &&
                                Math.Abs((double)feature["X"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("X")).Average()) <= ProcessConfigModel.YoloMaxXPixelsDeltaPerCluster &&
                                (double)feature["Time"] - clusterDf.AsEnumerable().Select(row => row.Field<double>("Time")).Max() <= ProcessConfigModel.YoloMaxTimeGap)

                            //This is worse:
                            //if( IsGoodNextFeature(yMovePerTimeSlice,
                            //    clusterDf.AsEnumerable().Select(row => row.Field<double>("Time")).Max(),
                            //    clusterDf.AsEnumerable().Select(row => row.Field<double>("X")).Average(),
                            //    clusterDf.AsEnumerable().Select(row => row.Field<double>("Y")).Max(), 
                            //    feature) )
                            // Consider finding the exist cluster feature with the closest but lower Time value and then use IsGoodNextFeature

                            //This is worse:
                            // Find the row with the highest "Time" value that is strictly less than TestTime
                            //DataRow prevFeature = clusterDf.AsEnumerable()
                            //                      .Where(row => row.Field<double>("Time") < (double)feature["Time"])
                            //                      .OrderByDescending(row => row.Field<double>("Time"))
                            //                      .FirstOrDefault();
                            //if( IsGoodNextFeature(yMovePerTimeSlice, prevFeature, feature) )
                            {
                                adjustedDf.ImportRow(feature);
                                rowsToRemove.Add(feature);
                                reassigned = true;
                            }
                        }
                    }

                    foreach (var row in rowsToRemove)
                        orphanedFeatures.Rows.Remove(row);

                    if (!reassigned)
                        break;
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
            var featuresSeen = new List<int>();

            try
            {
                foreach (int cluster in df.AsEnumerable().Select(row => row.Field<int>("Cluster")).Distinct())
                {
                    var clusterDf = df.AsEnumerable().Where(row => row.Field<int>("Cluster") == cluster).OrderBy(row => row.Field<int>("Feature")).ToList();

                    int prevID = -1;
                    double prevY = 0;
                    double prevTime = 0;

                    foreach (var row in clusterDf)
                    {
                        int thisID = (int)row["Feature"];
                        double thisY = (double)row["Y"];
                        double thisTime = (double)row["Time"];

                        if (prevID >= 0)
                        {
                            // Each feature must be in at most one cluster.
                            if (featuresSeen.Contains(thisID))
                                validationErrors.Add($"Cluster {cluster}, Feature {thisID}: Feature already seen in another cluster.");
                            else
                                featuresSeen.Add(thisID);

                            // Features in a cluster must be close to each other in space and time
                            if (Math.Abs(thisY - prevY) > ProcessConfigModel.YoloMaxYPixelsDeltaPerFrame - yMovePerTimeSlice || thisY - prevY < -yMovePerTimeSlice)
                                validationErrors.Add($"Cluster {cluster}, Feature {thisID}: Y change out of allowed range.");

                            // Features in a cluster must be ordered by time
                            if (thisTime < prevTime)
                                validationErrors.Add($"Cluster {cluster}, Feature {thisID}: Time not monotonically increasing.");

                            // A time gap of more than MaxTimeGap frames is not allowed
                            if (thisTime - prevTime > ProcessConfigModel.YoloMaxTimeGap)
                                validationErrors.Add($"Cluster {cluster}, Feature {thisID}: Time gap greater than {ProcessConfigModel.YoloMaxTimeGap}.");
                        }

                        prevID = thisID;
                        prevY = thisY;
                        prevTime = thisTime;
                    }
                }
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("YoloTracker.ValidateClusters", ex);
            }

            return validationErrors;
        }
    }
}
