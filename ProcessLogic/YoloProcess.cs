// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using Compunet.YoloV8.Data;
using SkyCombGround.GroundLogic;


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

        public YoloObjList YoloObjects;

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
        protected override void ProcessStart()
        {
            LegFrameFeatures.Clear();
            YoloObjects.Clear();
            YoloObjects.LegFirstIndex = 0;
            FlightLeg_SigObjects = 0;
            FlightLeg_StartObjects = 0;

            base.ProcessStart();
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public override void EnsureObjectsNamed()
        {
            YoloObjects.EnsureObjectsNamed();
        }


        // For process robustness, we want to process each leg independently.
        protected override void ProcessFlightLegStart(ProcessScope scope, int legId)
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


        protected override void ProcessFlightLegEnd(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
            {
                if (LegFrameFeatures.Count > 0)
                {
                    // During the leg, we found features using Yolo image detection,
                    // and defined objects using IoU overlap in successive frames.
                    int featuresCount = LegFrameFeatures.Count;
                    int basicObjectCount = YoloObjects.Count - FlightLeg_StartObjects;
                    Assert(basicObjectCount <= featuresCount, "Basic detection bad");

                    YoloTracker tracker = new( 
                        0.1f, // % overlap between feature in successive images 
                        0.66f ); // % confidence in merging two objects
                    var objectsSeen = tracker.CalculateObjects(LegFrameFeatures);

                    // For each ObjectSeen, create a YoloObject
                    FlightLeg_SigObjects = 0;
                    foreach (var objSeen in objectsSeen)
                    {
                        FlightLeg_SigObjects++;
                        YoloFeature firstFeature = ProcessFeatures[ objSeen.Features[0].FeatureId ] as YoloFeature;
                        YoloObject newObject = YoloObjects.AddObject(scope, firstFeature);

                        // Add remaining features to the object
                        for (int i = 1; i < objSeen.Features.Count; i++)
                        {
                            YoloFeature nextFeature = ProcessFeatures[objSeen.Features[i].FeatureId] as YoloFeature;
                            newObject.ClaimFeature(nextFeature);
                        }
                    }
                }

                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                YoloObjects.StopTracking();
                LegFrameFeatures.Clear();
            }

            EnsureObjectsNamed();
        }


        // Process the image using YOLO to find features. 
        // For each feature found, if it overlaps an object (from previous frame), object claims the feature.
        // Else create a new object to own the feature.
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

/*
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


                // For each active object, consider each feature 
                // found in this frame to see if it overlaps.
                Phase = 3;
                foreach (var theObject in inScopeObjects)
                {
                    var yoloObject = theObject.Value as YoloObject;

                    var lastFeat = theObject.Value.LastFeature;
                    if (lastFeat.Block.BlockId == blockID - 1)
                    {
                        // If one or more features overlaps the object's expected location,
                        // claim ownership of the feature(s), and mark them as Significant.
                        var expectedObjectLocation = theObject.Value.ExpectedLocationThisBlock();

                        bool claimedFeatures = false;
                        foreach (var feature in featuresInBlock)
                            // Object will claim feature if location overlaps byProcessConfig.FeatureMinOverlapPerc
                            if (yoloObject.MaybeClaimFeature(feature.Value as YoloFeature, expectedObjectLocation))
                            {
                                availFeatures.Remove(feature.Value.FeatureId);
                                claimedFeatures = true;
                            }
                        if (claimedFeatures)
                            availObjects.Remove(theObject.Value.ObjectId);
                    }
                }
*/

                Phase = 5;
                currBlock.AddFeatureList(featuresInBlock);
                ProcessFeatures.AddFeatureList(featuresInBlock);

/*
                // For all unowned active features in this frame, create a new object to own the feature.
                foreach (var feature in availFeatures)
                    if (feature.Value.ObjectId == 0)
                        YoloObjects.AddObject(scope, feature.Value as YoloFeature);

                Phase = 8;
                if (Drone.UseFlightLegs)
                    // Ensure each significant object in this leg has a "significant" name e.g. C5
                    // Needs to be done ASAP so the "C5" name can be drawn on video frames.
                    // Note: Some objects never become significant.
                    FlightLeg_SigObjects = EnsureObjectsNamed(FlightLeg_SigObjects, inScopeObjects, scope.CurrRunFlightStep);
*/

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
