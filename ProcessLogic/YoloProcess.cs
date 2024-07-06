// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using System.Drawing;
using Compunet.YoloV8.Data;
using SkyCombGround.GroundLogic;
using SixLabors.ImageSharp;



namespace SkyCombImage.ProcessModel
{
    // A class to hold all Yolo feature and block data associated with a video
    // Ignores thermal threshold.
    public class YoloProcess : ProcessAll
    {
        // YOLO (You only look once) V8 image processing
        public YoloDetect YoloDetect;

        // List of features detected in each frame in a leg by YoloDetect 
        List<FeatureSeen> LegFrameFeatures;

        public YoloObjectList YoloObjects;

        // If UseFlightLegs, how many significant objects have been found in this FlightLeg?
        public int FlightLeg_SigObjects { get; set; }
        public int FlightLeg_StartObjects { get; set; }



        public YoloProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, string yoloDirectory) : base(ground, video, drone, config)
        {
            YoloDetect = new YoloDetect(yoloDirectory, config.YoloDetectConfidence, config.YoloIoU);
            LegFrameFeatures = new();
            YoloObjects = new(this);
            YoloObjects.LegFirstIndex = 0;
            FlightLeg_SigObjects = 0;
            FlightLeg_StartObjects = 0;
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public override void ResetModel()
        {
            LegFrameFeatures.Clear();
            YoloObjects.Clear();
            YoloObjects.LegFirstIndex = 0;
            FlightLeg_SigObjects = 0;
            FlightLeg_StartObjects = 0;

            base.ResetModel();
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public override void EnsureObjectsNamed()
        {
            YoloObjects.EnsureObjectsNamed();
        }


        // For process robustness, we want to process each leg independently.
        public override void ProcessFlightLegStart(int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                YoloObjects.StopTracking();
                LegFrameFeatures.Clear();
                FlightLeg_SigObjects = 0;
                FlightLeg_StartObjects = YoloObjects.Count;
            }
        }


        public override void ProcessFlightLegEnd(int legId)
        {
            if (Drone.UseFlightLegs)
            {
                if (LegFrameFeatures.Count > 0)
                {
                    /*
                        // Number of objects found in this leg using just IoU testing.
                        int featuresCount = LegFrameFeatures.Count;
                        int combLegObjectCount = YoloObjects.Count - FlightLeg_StartObjects;

                        YoloImageTracker tracker = new( 
                            0.1f, // % overlap between feature in successive images 
                            0.66f ); // % confidence in merging two objects
                        var targets = tracker.CalculateTargets(LegFrameFeatures);

                        // Number of objects found in this leg using just IoU and merging.
                        int yoloImageLegObjectCount = targets.Count;

                        Assert(yoloImageLegObjectCount <= featuresCount, "Bad YoloImageTracker.Targets.Count 1");
                        Assert(yoloImageLegObjectCount <= combLegObjectCount, "Bad YoloImageTracker.Targets.Count 2");
                    */
                }

                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                YoloObjects.StopTracking();
                LegFrameFeatures.Clear();
            }

            EnsureObjectsNamed();
        }



        // Create a Yolo feature and add it to the Yolo object. Update the block totals
        public void ObjectClaimsNewFeature(ProcessBlock block, YoloObject theObject, YoloFeature newFeature)
        {
            try
            {
                block.AddFeature(newFeature);
                theObject.ClaimFeature(newFeature);
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloProcessModel.ObjectClaimsNewFeature", ex);
            }
        }


        public int ProcessBlock(
            ProcessScope scope,
            Image<Gray, byte> prevGray,
            Image<Gray, byte> currGray,
            DetectionResult? result)
        {
            int Phase = 0;

            try
            {
                if (result == null)
                    return 0;
                var numSig = result.Boxes.Count();

                Phase = 1;
                var currBlock = Blocks.LastBlock;
                int blockID = currBlock.BlockId;


                // Convert Yolo Bounding Boxes to YoloFeatures
                ProcessFeatureList featuresInBlock = new(this.ProcessConfig);
                foreach (var box in result.Boxes)
                {
                    // We have found a new feature/object
                    var imagePixelBox = new System.Drawing.Rectangle(box.Bounds.Left, box.Bounds.Top, box.Bounds.Width, box.Bounds.Height);
                    var newFeature = new YoloFeature(this, blockID, imagePixelBox, box);
                    featuresInBlock.AddFeature(newFeature);
                    LegFrameFeatures.Add(new FeatureSeen { BlockId = blockID, Box = imagePixelBox, FeatureId = newFeature.FeatureId });
                }


                // We only want to consider objects that are active.
                // For long flights most objects will have become inactive seconds or minutes ago.
                Phase = 2;
                YoloObjectList inScopeObjects = new(this);
                YoloObjectList availObjects = new(this);
                foreach (var theObject in YoloObjects)
                    if ((theObject.Value.LastFeature != null) &&
                        (theObject.Value.LastFeature.Block.BlockId == blockID - 1))
                    {
                        inScopeObjects.AddObject(theObject.Value);
                        availObjects.AddObject(theObject.Value);
                    }

                // Each feature can only be claimed once
                ProcessFeatureList availFeatures = featuresInBlock.Clone();


                // For each active object, consider each feature (significant or not)
                // found in this frame to see if it overlaps.
                // This priviledges objects with multiple real features,
                // as they can more accurately estimate their expected location.
                // We priviledge objects with a "real" last feature over objects with a "unreal" last feature.
                Phase = 3;
                for (int pass = 0; pass < 2; pass++)
                    foreach (var theObject in inScopeObjects)
                    {
                        var lastFeat = theObject.Value.LastFeature;
                        if ((lastFeat.Block.BlockId == blockID - 1) &&
                            (pass == 0 ? lastFeat.Type == FeatureTypeEnum.Real : lastFeat.Type != FeatureTypeEnum.Unreal))
                        {
                            // If one or more features overlaps the object's expected location,
                            // claim ownership of the feature(s), and mark them as Significant.
                            var expectedObjectLocation = theObject.Value.ExpectedLocationThisBlock();

                            bool claimedFeatures = false;
                            foreach (var feature in featuresInBlock)
                                // Object will claim feature if the object remains viable after claiming feature
                                if ((theObject.Value as YoloObject).MaybeClaimFeature(feature.Value as YoloFeature, expectedObjectLocation))
                                {
                                    availFeatures.Remove(feature.Value.FeatureId);
                                    claimedFeatures = true;
                                }
                            if (claimedFeatures)
                                availObjects.Remove(theObject.Value.ObjectId);
                        }
                    }

                // An active object with exactly one real feature can't estimate its expected location at all.
                // An active object with two features has a lot of wobble in its expected movement/location.
                // If the object is moving in the image quickly, the object location
                // and feature location will not overlap. Instead the feature will (usually)
                // be vertically below the object estimated location.
                Phase = 4;
                foreach (var theObject in availObjects)
                    if (theObject.Value.NumRealFeatures() <= 2)
                    {
                        // If one or more features overlaps the object's expected location,
                        // claim ownership of the feature(s), and mark them as Significant.
                        var expectedObjectLocation = theObject.Value.ExpectedLocationThisBlock();

                        // Search higher in the image 
                        expectedObjectLocation = new System.Drawing.Rectangle(
                            expectedObjectLocation.X,
                            expectedObjectLocation.Y + 20, // Higher
                            expectedObjectLocation.Width,
                            expectedObjectLocation.Height);

                        foreach (var feature in availFeatures)
                            (theObject.Value as YoloObject).MaybeClaimFeature(feature.Value as YoloFeature, expectedObjectLocation);
                    }


                Phase = 5;
                currBlock.AddFeatureList(featuresInBlock);
                ProcessFeatures.AddFeatureList(featuresInBlock);

/*
                // For each active object, where the above code did not find an 
                // overlapping feature in this Block, if it is worth continuing tracking...
                Phase = 6;
                foreach (var theObject in inScopeObjects)
                    if (theObject.Value.COM.BeingTracked &&
                        (theObject.Value.COM.LastRealFeatureIndex != UnknownValue) &&
                        (theObject.Value.LastRealFeature().Block.BlockId < blockID) &&
                        theObject.Value.KeepTracking(blockID))
                        // ... persist this object another Block. Create an unreal feature, with no pixels, with a rectangle   
                        // calculated from the object's last bounding rectangle and the average frame movement.
                        AddPersistFeature(theObject.Value);
*/


 //               // All active features have passed the min pixels and min density tests, and are worth tracking.
                // For all unowned active features in this frame, create a new object to own the feature.
                Phase = 7;
                foreach (var feature in availFeatures)
                    if (feature.Value.ObjectId == 0)
                    {
                        YoloObjects.AddObject(scope, feature.Value as YoloFeature );
                        if (blockID >= 2)
                        {
                            // TODO: Consider claiming overship of overlapping inactive features from the previous Block(s).
                        }
                    }


                if (Drone.UseFlightLegs)
                {
                    // Ensure each significant object in this leg has a "significant" name e.g. C5
                    // Needs to be done ASAP so the "C5" name can be drawn on video frames.
                    // Note: Some objects never become significant.
                    Phase = 8;
                    foreach (var theObject in inScopeObjects)
                        if ((theObject.Value.FlightLegId > 0) &&
                            ((scope.CurrRunFlightStep == null) || (theObject.Value.FlightLegId == scope.CurrRunFlightStep.FlightLegId)) &&
                            (theObject.Value.Significant) &&
                            (theObject.Value.Name == ""))
                        {
                            FlightLeg_SigObjects++;
                            theObject.Value.SetName(FlightLeg_SigObjects);
                        }
                }
                else
                {
                    // Track data related to a CombSpan (not a FlightLeg)
                    Phase = 9;
                    // PQR TODO ProcessObjectsFlightSteps(inScopeObjects, currBlock);
                }


                return numSig;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloProcessModel.ProcessBlock" +
                    "(CurrBlockId=" + scope.PSM.CurrBlockId +
                    ",LastBlockId=" + scope.PSM.LastBlockId + 
                    ",Phase=" + Phase + ")", ex);
            }
        }


        override public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "# Blocks", Blocks.Count },
                { "# Features", ProcessFeatures.Count},
                { "# Objects", YoloObjects.Count},
            };
        }

    };

}
