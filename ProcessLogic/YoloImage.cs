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
    public class YoloObjectSeen
    {
        // The features that make up the object. Implies the first and last frame the object was seen in
        public YoloFeatureSeenList Features;

        // Average pixel velocity of the object over time in the image
        public double AverageVelocityX;
        public double AverageVelocityY;


        public YoloObjectSeen()
        {
            Features = new();
            DefaultAverageVelocity();
        }


        public void DefaultAverageVelocity(double velocityX = 0, double velocityY = 0)
        {
            AverageVelocityX = velocityX;
            AverageVelocityY = velocityY;
        }


        public void CalculateAverageVelocity()
        {
            if (Features.Count < 2) 
                return;

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


    public class YoloObjectSeenList : List<YoloObjectSeen>
    {
        public YoloObjectSeenList() : base() { }


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
        public YoloObjectSeenList ObjectsSeen;

        // Minimum % overlap between features in successive images 
        private float IoUThreshold;

        // Minimum % confidence in merging two objects based on movement over time
        private float MergeConfidenceThreshold;

        // Weightings for three similarity criteria between two objects
        private float MergeVelocityWeighting = 0.45f;
        private float MergePositionWeighting = 0.45f;
        private float MergeSizeWeighting { get { return 1.0f - (MergeVelocityWeighting + MergePositionWeighting); } } 


        public YoloTracker(
            float ioUThreshold, // e.g. 0.25 for 25% overlap 
            float mergeConfidenceThreshold, // e.g. 0.6 for 60% confidence in merging
            float mergeVelocityWeighting,
            float mergePositionWeighting )
        {
            ObjectsSeen = new();
            IoUThreshold = ioUThreshold;
            MergeConfidenceThreshold = mergeConfidenceThreshold;
            MergeVelocityWeighting = mergeVelocityWeighting;
            MergePositionWeighting = mergePositionWeighting;
        }


    public void ProcessFrame(int blockId, YoloFeatureSeenList features, bool firstBlock)
        {
            if (firstBlock)
            {
                // For the first frame, create a new target for each detection
                foreach (var feature in features)
                    ObjectsSeen.Add(new YoloObjectSeen { Features = new YoloFeatureSeenList { new YoloFeatureSeen { BlockId = blockId, Box = feature.Box, FeatureId = feature.FeatureId } } });
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
                ObjectsSeen.Add(new YoloObjectSeen { Features = new YoloFeatureSeenList { detection } });
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
            double answerVelocityX = 0;
            double answerVelocityY = 0;

            double totalVelocityX = 0;
            double totalVelocityY = 0;
            int count = 0;

            foreach (var target in ObjectsSeen)
                if (target.Features.Count >= 2)
                {
                    target.CalculateAverageVelocity();
                    totalVelocityX += target.AverageVelocityX;
                    totalVelocityY += target.AverageVelocityY;
                    count++;
                }

            if (count > 0)
            {
                answerVelocityX = totalVelocityX / count;
                answerVelocityY = totalVelocityY / count;

                foreach (var target in ObjectsSeen)
                    if (target.Features.Count == 1)
                        target.DefaultAverageVelocity(answerVelocityX, answerVelocityY);
            }

            return (answerVelocityX, answerVelocityY);
        }


        private double CalculateTargetSimilarity(YoloObjectSeen obj1, YoloObjectSeen obj2, (double X, double Y) droneVelocity)
        {
            // Time gap
            var lastDetection1 = obj1.Features.Last();
            var firstDetection2 = obj2.Features.First();
            int frameGap = firstDetection2.BlockId - lastDetection1.BlockId;
            if (frameGap <= 0)
                return 0;

            const double velocityEpsilon = 1e-2; // Pixels per frame
            bool velocityXSignificant = Math.Abs(droneVelocity.X) > velocityEpsilon;
            bool velocityYSignificant = Math.Abs(droneVelocity.Y) > velocityEpsilon;


            // Velocity similarity
            double velocityDiffX = obj1.AverageVelocityX - obj2.AverageVelocityX;
            double velocityDiffY = obj1.AverageVelocityY - obj2.AverageVelocityY;
            double velocitySimilarityX = 1;
            double velocitySimilarityY = 1;
            if (velocityXSignificant)
                velocitySimilarityX = 1 - Math.Abs(velocityDiffX / droneVelocity.X);
            else if (Math.Abs(velocityDiffX) > velocityEpsilon)
                velocitySimilarityX = 0;
            if (velocityYSignificant)
                velocitySimilarityY = 1 - Math.Abs(velocityDiffY / droneVelocity.Y);
            else if (Math.Abs(velocityDiffY) > velocityEpsilon)
                velocitySimilarityY = 0;
            double velocitySimilarity = (velocitySimilarityX + velocitySimilarityY) / 2;

            // Position similarity
            const double positionEpsilon = 1; // Pixels 
            double expectedX = lastDetection1.Box.X + obj1.AverageVelocityX * frameGap;
            double expectedY = lastDetection1.Box.Y + obj1.AverageVelocityY * frameGap;
            double diffX = expectedX - firstDetection2.Box.X;
            double diffY = expectedY - firstDetection2.Box.Y;
            double positionSimilarityX = 1;
            double positionSimilarityY = 1;
            if (velocityXSignificant)
                positionSimilarityX = 1 - Math.Abs(diffX) / Math.Abs(droneVelocity.X * frameGap);
            else if (Math.Abs(diffX) > positionEpsilon)
                positionSimilarityX = 0;
            if (velocityYSignificant)
                positionSimilarityY = 1 - Math.Abs(diffY) / Math.Abs(droneVelocity.Y * frameGap);
            else if (Math.Abs(diffY) > positionEpsilon)
                positionSimilarityY = 0;
            double positionSimilarity = (positionSimilarityX + positionSimilarityY) / 2;

            // Size similarity
            double sizeSimilarity = 0;
            if( lastDetection1.Box.Width > 1 && lastDetection1.Box.Height > 1)
                sizeSimilarity = 1-(Math.Abs(lastDetection1.Box.Width - firstDetection2.Box.Width) / (1.0 * lastDetection1.Box.Width) +
                                    Math.Abs(lastDetection1.Box.Height - firstDetection2.Box.Height) / (1.0 * lastDetection1.Box.Height)) / 2.0;

            // Combine similarities (you can adjust weights as needed)
            var answer =
                velocitySimilarity * MergeVelocityWeighting +
                positionSimilarity * MergePositionWeighting +
                sizeSimilarity * MergeSizeWeighting;

            return answer;
        }


        public void MergeSimilarObjects()
        {
            var droneVelocity = CalculateAverageDroneVelocity();
            var objectsToMerge = new List<(YoloObjectSeen, YoloObjectSeen)>();

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
                if (ObjectsSeen.Contains(t2))
                {
                    t1.Features.AddRange(t2.Features);
                    t1.Features.Sort((a, b) => a.BlockId.CompareTo(b.BlockId));
                    t2.Features.Clear();
                    var removed = ObjectsSeen.Remove(t2);
                    BaseConstants.Assert(removed, "YoloTracker.MergeSimilarObjects: Failed to remove target from list");

                    t1.CalculateAverageVelocity();
                    t2.DefaultAverageVelocity();
                }
                else
                {
                    // t2 has already been merged and removed in a previous iteration
                }
            }
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
