// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    internal class ClusterInfoC
    {
        public Rectangle BoundingBox { get; set; }
        public int HotPixelCount { get; set; }
        public byte MinHeat { get; set; }
        public byte MaxHeat { get; set; }
        public bool IsSignificant { get; set; }
        public List<Point> HotPixels { get; set; } = new List<Point>();
    }


    internal static class ThresholdFeatureLogic
    {
        static byte HeatThresholdValue = 180;
        static int MinPixels = 8;


        // Enhanced threshold method that also performs clustering analysis
        public static List<ClusterInfoC> AnalyzeWithClustering(Image<Gray, byte> imgInput, ProcessConfigModel config)
        {
            // First, find all hot pixels (excluding those in exclusion zones)
            var hotPixels = FindHotPixels(imgInput, config);

            // Cluster adjacent hot pixels
            var clusters = ClusterAdjacentPixels(hotPixels, imgInput);

            // Merge clusters that are contained within each other
            var mergedClusters = MergeContainedClusters(clusters);

            // Mark significance and finalize cluster info
            var clusterInfos = GenerateClusterInfo(mergedClusters, imgInput);

            return clusterInfos;
        }


        private static HashSet<Point> FindHotPixels(Image<Gray, byte> image, ProcessConfigModel config)
        {
            var hotPixels = new HashSet<Point>();
            var data = image.Data;
            int imageWidth = image.Width;
            int imageHeight = image.Height;

            for (int y = 0; y < imageHeight; y++)
            {
                for (int x = 0; x < imageWidth; x++)
                {
                    // Check if pixel should be processed (not in exclusion zone)
                    if (!config.ShouldProcessPixel(x, y, imageWidth, imageHeight))
                        continue;

                    if (data[y, x, 0] >= HeatThresholdValue)
                    {
                        hotPixels.Add(new Point(x, y));
                    }
                }
            }

            return hotPixels;
        }

        private static List<List<Point>> ClusterAdjacentPixels(HashSet<Point> hotPixels, Image<Gray, byte> image)
        {
            var clusters = new List<List<Point>>();
            var visited = new HashSet<Point>();

            // 8-directional adjacency
            var directions = new Point[]
            {
            new Point(-1, -1), new Point(-1, 0), new Point(-1, 1),
            new Point(0, -1),                    new Point(0, 1),
            new Point(1, -1),  new Point(1, 0),  new Point(1, 1)
            };

            foreach (var pixel in hotPixels)
            {
                if (visited.Contains(pixel))
                    continue;

                var cluster = new List<Point>();
                var stack = new Stack<Point>();
                stack.Push(pixel);

                while (stack.Count > 0)
                {
                    var current = stack.Pop();

                    if (visited.Contains(current))
                        continue;

                    visited.Add(current);
                    cluster.Add(current);

                    // Check all 8 adjacent pixels
                    foreach (var dir in directions)
                    {
                        var neighbor = new Point(current.X + dir.X, current.Y + dir.Y);

                        if (neighbor.X >= 0 && neighbor.X < image.Width &&
                            neighbor.Y >= 0 && neighbor.Y < image.Height &&
                            hotPixels.Contains(neighbor) &&
                            !visited.Contains(neighbor))
                        {
                            stack.Push(neighbor);
                        }
                    }
                }

                if (cluster.Count > 0)
                {
                    clusters.Add(cluster);
                }
            }

            return clusters;
        }

        private static List<List<Point>> MergeContainedClusters(List<List<Point>> clusters)
        {
            // Calculate bounding boxes for each cluster
            var clusterBoxes = clusters.Select(cluster =>
            {
                var minX = cluster.Min(p => p.X);
                var maxX = cluster.Max(p => p.X);
                var minY = cluster.Min(p => p.Y);
                var maxY = cluster.Max(p => p.Y);
                return new Rectangle(minX, minY, maxX - minX + 1, maxY - minY + 1);
            }).ToList();

            var merged = new List<List<Point>>();
            var processed = new bool[clusters.Count];

            for (int i = 0; i < clusters.Count; i++)
            {
                if (processed[i])
                    continue;

                var mainCluster = new List<Point>(clusters[i]);
                var mainBox = clusterBoxes[i];
                processed[i] = true;

                // Check if any other clusters are contained within this cluster's bounding box
                for (int j = 0; j < clusters.Count; j++)
                {
                    if (i == j || processed[j])
                        continue;

                    var otherBox = clusterBoxes[j];

                    // Check if cluster j is fully contained within cluster i's bounding box
                    if (IsRectangleContained(otherBox, mainBox))
                    {
                        mainCluster.AddRange(clusters[j]);
                        processed[j] = true;

                        // Update the main bounding box to include the merged cluster
                        var newMinX = Math.Min(mainBox.Left, otherBox.Left);
                        var newMinY = Math.Min(mainBox.Top, otherBox.Top);
                        var newMaxX = Math.Max(mainBox.Right, otherBox.Right);
                        var newMaxY = Math.Max(mainBox.Bottom, otherBox.Bottom);
                        mainBox = new Rectangle(newMinX, newMinY, newMaxX - newMinX, newMaxY - newMinY);
                    }
                }

                merged.Add(mainCluster);
            }

            return merged;
        }

        private static bool IsRectangleContained(Rectangle inner, Rectangle outer)
        {
            return outer.Left <= inner.Left &&
                   outer.Top <= inner.Top &&
                   outer.Right >= inner.Right &&
                   outer.Bottom >= inner.Bottom;
        }

        private static List<ClusterInfoC> GenerateClusterInfo(List<List<Point>> clusters, Image<Gray, byte> image)
        {
            var clusterInfos = new List<ClusterInfoC>();
            var data = image.Data;

            foreach (var cluster in clusters)
            {
                if (cluster.Count == 0)
                    continue;

                var info = new ClusterInfoC();

                // Calculate bounding box
                var minX = cluster.Min(p => p.X);
                var maxX = cluster.Max(p => p.X);
                var minY = cluster.Min(p => p.Y);
                var maxY = cluster.Max(p => p.Y);
                info.BoundingBox = new Rectangle(minX, minY, maxX - minX + 1, maxY - minY + 1);

                // Count hot pixels and find min/max heat
                info.HotPixelCount = cluster.Count;
                info.HotPixels = new List<Point>(cluster);

                byte minHeat = 255;
                byte maxHeat = 0;

                foreach (var pixel in cluster)
                {
                    var heatValue = data[pixel.Y, pixel.X, 0];
                    minHeat = Math.Min(minHeat, heatValue);
                    maxHeat = Math.Max(maxHeat, heatValue);
                }

                info.MinHeat = minHeat;
                info.MaxHeat = maxHeat;
                info.IsSignificant = info.HotPixelCount >= MinPixels;

                clusterInfos.Add(info);
            }

            return clusterInfos.OrderByDescending(c => c.HotPixelCount).ToList();
        }
    

        public static void CreateFeaturesFromImage(
            CombProcess combProcess,
            ProcessFeatureList featuresInBlock,
            ProcessBlock block,
            in Image<Bgr, byte> imgOriginal,    // read-only
            in Image<Gray, byte> imgThreshold)  // read-only
        {
            HeatThresholdValue = (byte) combProcess.ProcessConfig.HeatThresholdValue;
            MinPixels = ProcessConfigModel.FeatureMinPixels;

            var clusters = AnalyzeWithClustering(imgThreshold, combProcess.ProcessConfig);

            foreach (var cluster in clusters)
            {
                var feature = new CombFeature(combProcess, block, FeatureTypeEnum.Real);

                feature.ClearHotPixelData();
                feature.Pixels = new();

                foreach (var hotPixel in cluster.HotPixels)
                    feature.AddHotPixel(hotPixel.Y, hotPixel.X, imgOriginal[hotPixel.Y, hotPixel.X]);
                
                feature.PixelBox = cluster.BoundingBox;
                feature.Calculate_HotPixelData();
                feature.Calculate_Significant();
                feature.IsTracked = feature.Significant;

                featuresInBlock.AddFeature(feature);
            }
        }
    }
}
