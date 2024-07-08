// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Drawing;


// If this doesnt work consider using a Kalman filter


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
    }


    // An object is a collection of features over several frames
    public class ProcessObjectseen
    {
        // The features that make up the object. Implies the first and last frame the object was seen in
        public YoloFeatureSeenList Features;

        // Average pixel velocity of the object over time in the image
        public double AverageVelocityX;
        public double AverageVelocityY;


        public ProcessObjectseen()
        {
            Features = new();
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


    public class ProcessObjectseenList : List<ProcessObjectseen>
    {
        public ProcessObjectseenList() : base() { }


        public int NumSignificant()
        {
            int numSignificant = 0;

            foreach (var target in this)
                if (target.Features.Any())
                    numSignificant++;

            return numSignificant;
        }
    }


    // A class to track all objects across all frames. 
    // Physical objects may be visible, then obscured, then visible resulting in two ObjectSeen objects. Try to merge them.
    public class YoloTracker
    {
        public ProcessObjectseenList ObjectsSeen;

        // Minimum % overlap between features in successive images 
        private float IoUThreshold;

        // Minimum % confidence in merging two objects based on movement over time
        private float MergeConfidenceThreshold;


        public YoloTracker(
            float ioUThreshold, // e.g. 0.25 for 25% overlap 
            float mergeConfidenceThreshold = 0.6f) // e.g. 0.6 for 60% confidence in merging
        {
            ObjectsSeen = new();
            IoUThreshold = ioUThreshold;
            MergeConfidenceThreshold = mergeConfidenceThreshold;
        }


        public void ProcessFrame(int blockId, YoloFeatureSeenList features, bool firstBlock)
        {
            if (firstBlock)
            {
                // For the first frame, create a new target for each detection
                foreach (var feature in features)
                    ObjectsSeen.Add(new ProcessObjectseen { Features = new YoloFeatureSeenList { new YoloFeatureSeen { BlockId = blockId, Box = feature.Box, FeatureId = feature.FeatureId } } });
                return;
            }

            var unassignedFeatures = features.Clone();

            foreach (var existingObject in ObjectsSeen)
            {
                if (existingObject.Features.Any() && (blockId == existingObject.Features.Last().BlockId + 1))
                {
                    var lastBox = existingObject.Features.Last().Box;
                    var bestMatch = FindBestMatch(lastBox, unassignedFeatures);
                    if (bestMatch != null)
                    {
                        existingObject.Features.Add(bestMatch);
                        var removed = unassignedFeatures.Remove(bestMatch);
                        BaseConstants.Assert(removed, "YoloTracker.ProcessFrame: Failed to remove feature from unassigned list");
                    }
                }
            }

            // Create new objects for unassigned features
            foreach (var detection in unassignedFeatures)
                ObjectsSeen.Add(new ProcessObjectseen { Features = new YoloFeatureSeenList { detection } });
        }


        // Find the feature with the best overlap. Overlap must exceed IoUThreshold 
        private YoloFeatureSeen? FindBestMatch(Rectangle lastKnownBox, YoloFeatureSeenList detections)
        {
            double maxIoU = 0;
            YoloFeatureSeen? bestMatch = null;

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


        // Calculate the overlap
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

            foreach (var target in ObjectsSeen)
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
                return (totalVelocityX / count, totalVelocityY / count);

            return (0, 0);
        }


        private double CalculateTargetSimilarity(ProcessObjectseen t1, ProcessObjectseen t2, (double X, double Y) droneVelocity)
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


        public void MergeSimilarObjects()
        {
            var droneVelocity = CalculateAverageDroneVelocity();
            var objectsToMerge = new List<(ProcessObjectseen, ProcessObjectseen)>();

            for (int i = 0; i < ObjectsSeen.Count; i++)
            {
                for (int j = i + 1; j < ObjectsSeen.Count; j++)
                {
                    var t1 = ObjectsSeen[i];
                    var t2 = ObjectsSeen[j];

                    if(t1.Features.Any() && t2.Features.Any())
                        if (t1.Features.Last().BlockId < t2.Features.First().BlockId)
                        {
                            double similarity = CalculateTargetSimilarity(t1, t2, droneVelocity);
                            if (similarity > MergeConfidenceThreshold)
                                // We have found two objects to merge
                                objectsToMerge.Add((t1, t2));
                        }
                }
            }

            foreach (var (t1, t2) in objectsToMerge)
            {
                t1.Features.AddRange(t2.Features);
                t1.Features.Sort((a, b) => a.BlockId.CompareTo(b.BlockId));
                t2.Features.Clear();
                var removed = ObjectsSeen.Remove(t2);
                //if( ! removed)
                //    BaseConstants.Assert(removed, "YoloTracker.MergeSimilarObjects: Failed to remove target from list");
            }

            // Recalculate velocities for merged targets
            foreach (var target in ObjectsSeen)
                target.CalculateAverageVelocity();
        }


        public (int, int) CalculateObjectsInLeg(YoloFeatureSeenList features)
        {
            var minFrameId = features[0].BlockId;
            var maxFrameId = features[^1].BlockId;

            bool firstFrame = true;

            // Process each frame
            for (int frameId = minFrameId; frameId <= maxFrameId; frameId++)
            {
                YoloFeatureSeenList frameFeatures = new();
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
            int preMerge = ObjectsSeen.NumSignificant();
            MergeSimilarObjects();
            int postMerge = ObjectsSeen.NumSignificant();

            return (preMerge, postMerge);
        }
    }
}
