// Copyright SkyComb Limited 2024. All rights reserved. 
using System.Drawing;


// If this doesnt work consider using a Kalman filter


namespace SkyCombImage.ProcessModel
{
    class FeatureSeen
    {
        public int BlockId;
        public int FeatureId;
        public Rectangle Box;
    }


    class TargetSeen
    {
        public List<FeatureSeen> Features;
        public double AverageVelocityX;
        public double AverageVelocityY;


        public TargetSeen()
        {
            Features = new List<FeatureSeen>();
            AverageVelocityX = 0;
            AverageVelocityY = 0;
        }


        public void CalculateAverageVelocity()
        {
            if (Features.Count < 2) return;

            double totalVelocityX = 0;
            double totalVelocityY = 0;
            int count = 0;

            for (int i = 1; i < Features.Count; i++)
            {
                var prev = Features[i - 1];
                var curr = Features[i];

                if (curr.BlockId - prev.BlockId == 1)
                {
                    totalVelocityX += curr.Box.X - prev.Box.X;
                    totalVelocityY += curr.Box.Y - prev.Box.Y;
                    count++;
                }
            }

            if (count > 0)
            {
                AverageVelocityX = totalVelocityX / count;
                AverageVelocityY = totalVelocityY / count;
            }
        }
    }


    class YoloImageTracker
    {
        private List<TargetSeen> Targets;

        // % overlap between features in successive images 
        private float IoUThreshold;

        // % confidence in merging two objects based on movement over time
        private float MergeConfidenceThreshold;


        public YoloImageTracker(
            float ioUThreshold = 0.1f, // 10% overlap 
            float mergeConfidenceThreshold = 0.8f) // 80% confidence in merging
        {
            Targets = new();
            IoUThreshold = ioUThreshold;
            MergeConfidenceThreshold = mergeConfidenceThreshold;
        }


        public void ProcessFrame(int blockId, List<FeatureSeen> features, bool firstBlock)
        {
            if (firstBlock)
            {
                // For the first frame, create a new target for each detection
                foreach (var feature in features)
                {
                    Targets.Add(new TargetSeen { Features = new List<FeatureSeen> { new FeatureSeen { BlockId = blockId, Box = feature.Box, FeatureId = feature.FeatureId } } });
                }
            }
            else
            {
                var unassignedDetections = new List<FeatureSeen>();
                foreach (var feature in features)
                {
                    unassignedDetections.Add(new FeatureSeen { BlockId = blockId, Box = feature.Box, FeatureId = feature.FeatureId });
                }

                foreach (var target in Targets)
                {
                    if (target.Features.Any() && blockId - target.Features.Last().BlockId == 1)
                    {
                        var lastKnownBox = target.Features.Last().Box;
                        var bestMatch = FindBestMatch(lastKnownBox, unassignedDetections);
                        if (bestMatch != null)
                        {
                            target.Features.Add(bestMatch);
                            unassignedDetections.Remove(bestMatch);
                        }
                    }
                }

                // Create new targets for unassigned detections
                foreach (var detection in unassignedDetections)
                {
                    Targets.Add(new TargetSeen { Features = new List<FeatureSeen> { detection } });
                }
            }
        }


        private FeatureSeen? FindBestMatch(Rectangle lastKnownBox, List<FeatureSeen> detections)
        {
            double maxIoU = 0;
            FeatureSeen? bestMatch = null;

            foreach (var detection in detections)
            {
                double iou = CalculateIoU(lastKnownBox, detection.Box);
                if (iou > maxIoU && iou >= IoUThreshold)
                {
                    maxIoU = iou;
                    bestMatch = detection;
                }
            }

            return bestMatch;
        }


        private double CalculateIoU(Rectangle box1, Rectangle box2)
        {
            // Calculate the intersection rectangle
            double x1 = Math.Max(box1.X, box2.X);
            double y1 = Math.Max(box1.Y, box2.Y);
            double x2 = Math.Min(box1.X + box1.Width, box2.X + box2.Width);
            double y2 = Math.Min(box1.Y + box1.Height, box2.Y + box2.Height);

            // Calculate intersection area
            double intersectionArea = Math.Max(0, x2 - x1) * Math.Max(0, y2 - y1);

            // Calculate union area
            double box1Area = box1.Width * box1.Height;
            double box2Area = box2.Width * box2.Height;
            double unionArea = box1Area + box2Area - intersectionArea;

            return intersectionArea / unionArea;
        }


        public (double X, double Y) CalculateAverageDroneVelocity()
        {
            double totalVelocityX = 0;
            double totalVelocityY = 0;
            int count = 0;

            foreach (var target in Targets)
            {
                target.CalculateAverageVelocity();
                if (target.Features.Count >= 2)
                {
                    totalVelocityX += target.AverageVelocityX;
                    totalVelocityY += target.AverageVelocityY;
                    count++;
                }
            }

            if (count > 0)
            {
                return (totalVelocityX / count, totalVelocityY / count);
            }
            return (0, 0);
        }


        private double CalculateTargetSimilarity(TargetSeen t1, TargetSeen t2, (double X, double Y) droneVelocity)
        {
            // Velocity similarity
            double velocitySimilarity = 1 - (Math.Abs(t1.AverageVelocityX - t2.AverageVelocityX) / Math.Abs(droneVelocity.X) +
                                                Math.Abs(t1.AverageVelocityY - t2.AverageVelocityY) / Math.Abs(droneVelocity.Y)) / 2;

            // Position similarity
            var lastDetection1 = t1.Features.Last();
            var firstDetection2 = t2.Features.First();
            int frameGap = firstDetection2.BlockId - lastDetection1.BlockId;
            double expectedX = lastDetection1.Box.X + t1.AverageVelocityX * frameGap;
            double expectedY = lastDetection1.Box.Y + t1.AverageVelocityY * frameGap;
            double positionSimilarity = 1 - (Math.Abs(expectedX - firstDetection2.Box.X) / Math.Abs(droneVelocity.X * frameGap) +
                                                Math.Abs(expectedY - firstDetection2.Box.Y) / Math.Abs(droneVelocity.Y * frameGap)) / 2;

            // Size similarity
            double sizeSimilarity = 1 - (Math.Abs(lastDetection1.Box.Width - firstDetection2.Box.Width) / lastDetection1.Box.Width +
                                            Math.Abs(lastDetection1.Box.Height - firstDetection2.Box.Height) / lastDetection1.Box.Height) / 2;

            // Combine similarities (you can adjust weights as needed)
            return (velocitySimilarity * 0.4 + positionSimilarity * 0.4 + sizeSimilarity * 0.2);
        }


        public void MergeSimilarTargets()
        {
            var droneVelocity = CalculateAverageDroneVelocity();
            var targetsToMerge = new List<(TargetSeen, TargetSeen)>();

            for (int i = 0; i < Targets.Count; i++)
            {
                for (int j = i + 1; j < Targets.Count; j++)
                {
                    var t1 = Targets[i];
                    var t2 = Targets[j];

                    if (t1.Features.Last().BlockId < t2.Features.First().BlockId)
                    {
                        double similarity = CalculateTargetSimilarity(t1, t2, droneVelocity);
                        if (similarity > MergeConfidenceThreshold)
                        {
                            targetsToMerge.Add((t1, t2));
                        }
                    }
                }
            }

            foreach (var (t1, t2) in targetsToMerge)
            {
                t1.Features.AddRange(t2.Features);
                t1.Features.Sort((a, b) => a.BlockId.CompareTo(b.BlockId));
                Targets.Remove(t2);
            }

            // Recalculate velocities for merged targets
            foreach (var target in Targets)
            {
                target.CalculateAverageVelocity();
            }
        }


        public List<TargetSeen> CalculateTargets(List<FeatureSeen> features)
        {
            var minFrameId = features[0].BlockId;
            var maxFrameId = features[^1].BlockId;

            bool firstFrame = true;

            // Process each frame
            for (int frameId = minFrameId; frameId <= maxFrameId; frameId++)
            {
                List<FeatureSeen> frameFeatures = new();
                foreach(var feature in features)
                    if (feature.BlockId == frameId)
                        frameFeatures.Add(feature);

                if (frameFeatures.Count > 0)
                {
                    ProcessFrame(frameId, frameFeatures, firstFrame);
                    firstFrame = false;
                }
            }

            // Merge similar targets
            MergeSimilarTargets();

            return Targets;
        }
    }
}
