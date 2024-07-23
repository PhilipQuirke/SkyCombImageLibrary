// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold all feature data and Block data associated with a video
    public class CombProcess : ProcessAll
    {
        // If UseFlightLegs, how many significant objects have been found in this FlightLeg?
        public int FlightLeg_SigObjects { get; set; }


        public CombProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config) : base(ground, video, drone, config)
        {
            FlightLeg_SigObjects = 0;
         }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        protected override void ProcessStart()
        {
            FlightLeg_SigObjects = 0;
 
            base.ProcessStart();

            // We want the category information to be retained between runs, so don't clear it.
            // Categories.Clear();
            // ObjectCategories.Clear();
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public override void EnsureObjectsNamed()
        {
            ProcessObjects.EnsureObjectsNamed();
        }


        protected override void ProcessFlightLegStart(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                ProcessObjects.StopTracking();
                FlightLeg_SigObjects = 0;
            }
        }


        // Create an unreal feature, with no pixels, with a rectangle calculated from the object's 
        // last bounding rectangle, maximum pixel box, and the average object movement.
        public void AddPersistFeature(CombObject theObject)
        {
            var theBlock = Blocks.LastBlock;

            CombFeature theFeature = new(this, theBlock, FeatureTypeEnum.Unreal);
            theFeature.PixelBox = theObject.ExpectedLocationThisBlock();
            ProcessFeatures.AddFeature(theFeature);

            Assert(Blocks.Count == theFeature.Block.BlockId, "AddPersistFeature: Bad Blocks count");

            theObject.ClaimFeature(theFeature);
        }


        // Save memory (if compatible with Config settings) by deleting pixel data
        // For Comb, we only care about pixel data for objects / features in legs
        public void DeleteFeaturePixelsForObjects()
        {
            if (ProcessConfig.SavePixels == SavePixelsEnum.None)
                foreach (var theObject in ProcessObjects)
                {
                    var combObject = theObject.Value as CombObject;
                    if (combObject.FlightLegId <= 0)
                        foreach (var feature in combObject.ProcessFeatures)
                        {
                            feature.Value.Pixels?.Clear();
                            // NumHotPixels is not cleared 
                        }
                }
        }


        // If !UseFlightLegs, track significant objects, based on their corresponding FlightSteps.
        // Does not make use of FlightLegs.
        private void ProcessObjectsFlightSteps(ProcessObjList inScopeObjects, ProcessBlock currBlock)
        {
            if (!Drone.UseFlightLegs)
            {
                inScopeObjects.EnsureObjectsNamed();

                int sigObjects = inScopeObjects.NumSignificantObjects();
                if (sigObjects > 0)
                {
                    if (FlightSteps_PrevSigObjects == 0)
                        // PROCESS SPAN START EVENT
                        // Object(s) have just become significant. Last frame there were no significant objects.
                        // It takes a few steps to become significant, so get the minimal FlightStep.StepID of the object(s).
                        // ProcessSpan_MinFlightStepId is the starting step of a future ProcessSpan object.
                        FlightSteps_MinStepId = inScopeObjects.GetMinStepId();

                    FlightSteps_PrevSigObjects = Math.Max(FlightSteps_PrevSigObjects, sigObjects);
                    FlightSteps_MaxStepId = currBlock.FlightStepId;
                }
                else
                {
                    // We have no significant objects 

                    if (FlightSteps_PrevSigObjects > 0)
                    {
                        // We have unprocessed significant objects from previous blocks. 

                        if (currBlock.FlightStepId - FlightSteps_MaxStepId > 8)
                        {
                            // PROCESS SPAN END EVENT
                            // We tracked some significant objects, then they all became insignificant, and 8 frames have passed
                            ProcessSpan_Create();

                            ResetSpanData();
                        }
                    }
                }
            }
        }


        // Process the features found in the current block/frame, which is part of a leg,
        // by preference adding them to existing objects (created in previous blocks/frames),
        // else creating new objects to hold the features.
        public void ProcessBlockForObjects(ProcessScope scope, ProcessFeatureList featuresInBlock)
        {
            int Phase = 0;

            try
            {
                Phase = 1;
                var currBlock = Blocks.LastBlock;
                int blockID = currBlock.BlockId;


                // We only want to consider objects that are active.
                // For long flights most objects will have become inactive seconds or minutes ago.
                Phase = 2;
                ProcessObjList inScopeObjects = new();
                ProcessObjList availObjects = new();
                foreach (var theObject in ProcessObjects)
                {
                    var combObject = theObject.Value as CombObject;
                    if ((combObject.LastFeature != null) &&
                        (combObject.LastFeature.Block.BlockId == blockID - 1) &&
                        combObject.VaguelySignificant())
                    {
                        inScopeObjects.AddObject(combObject);
                        availObjects.AddObject(combObject);
                    }
                }

                // Each feature can only be claimed once. Clone the list (not the features).
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
                        var combObject = theObject.Value as CombObject;
                        var lastFeat = combObject.LastFeature;
                        if ((lastFeat.Block.BlockId == blockID - 1) &&
                            (pass == 0 ? lastFeat.Type == FeatureTypeEnum.Real : lastFeat.Type != FeatureTypeEnum.Unreal))
                        {
                            // If one or more features overlaps the object's expected location,
                            // claim ownership of the feature(s), and mark them as Significant.
                            var expectedObjectLocation = combObject.ExpectedLocationThisBlock();

                            bool claimedFeatures = false;
                            foreach (var feature in featuresInBlock)
                                // Object will claim feature if the object remains viable after claiming feature
                                if (combObject.MaybeClaimFeature(feature.Value as CombFeature, expectedObjectLocation))
                                {
                                    availFeatures.Remove(feature.Value.FeatureId);
                                    claimedFeatures = true;
                                }
                            if (claimedFeatures)
                                availObjects.Remove(combObject.ObjectId);
                        }
                    }

                // An active object with exactly one real feature can't estimate its expected location at all.
                // An active object with two features has a lot of wobble in its expected movement/location.
                // If the object is moving in the image quickly, the object location
                // and feature location will not overlap. Instead the feature will (usually)
                // be vertically below the object estimated location.
                Phase = 4;
                foreach (var theObject in availObjects)
                {
                    var combObject = theObject.Value as CombObject;
                    if (combObject.NumRealFeatures() <= 2)
                    {
                        // If one or more features overlaps the object's expected location,
                        // claim ownership of the feature(s), and mark them as Significant.
                        var expectedObjectLocation = combObject.ExpectedLocationThisBlock();

                        // Search higher in the image 
                        expectedObjectLocation = new System.Drawing.Rectangle(
                            expectedObjectLocation.X,
                            expectedObjectLocation.Y + ProcessConfigModel.CombHigherPixels, // Higher
                            expectedObjectLocation.Width,
                            expectedObjectLocation.Height);

                        foreach (var feature in availFeatures)
                            combObject.MaybeClaimFeature(feature.Value as CombFeature, expectedObjectLocation);
                    }
                }

                Phase = 5;
                currBlock.AddFeatureList(featuresInBlock);
                ProcessFeatures.AddFeatureList(featuresInBlock);

                // For each active object, where the above code did not find an 
                // overlapping feature in this Block, if it is worth continuing tracking...
                Phase = 6;
                foreach (var theObject in inScopeObjects)
                {
                    var combObject = theObject.Value as CombObject;
                    if (combObject.BeingTracked &&
                       (combObject.LastRealFeatureId > 0) &&
                       (combObject.LastRealFeature.Block.BlockId < blockID) &&
                       combObject.KeepTracking(blockID))
                        // ... persist this object another Block. Create an unreal feature, with no pixels, with a rectangle   
                        // calculated from the object's last bounding rectangle and the average frame movement.
                        AddPersistFeature(combObject);
                }

                // All active features have passed the min pixels and min density tests, and are worth tracking.
                // For all unowned active features in this frame, create a new object to own the feature.
                Phase = 7;
                foreach (var feature in availFeatures)
                    if (feature.Value.IsTracked && (feature.Value.ObjectId == 0))
                    {
                        var theObject = ProcessFactory.NewCombObject(scope, this, feature.Value as CombFeature);
                        ProcessObjects.AddObject(theObject);
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
                    FlightLeg_SigObjects = EnsureObjectsNamed(FlightLeg_SigObjects, inScopeObjects, scope.CurrRunFlightStep);
                }
                else
                {
                    // Track data related to a ProcessSpan (not a FlightLeg)
                    Phase = 9;
                    ProcessObjectsFlightSteps(inScopeObjects, currBlock);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessBlockForObjects.Phase=" + Phase, ex);
            }
        }


        // Process the features found in the current block/frame, which is NOT in a leg.
        // We store the features so we can draw them on the video frame later.
        public void ProcessBlockForFeatures(ProcessFeatureList featuresInBlock)
        {
            foreach (var feature in featuresInBlock)
            {
                feature.Value.IsTracked = false;
                feature.Value.Significant = false;
            }

            Blocks.LastBlock.AddFeatureList(featuresInBlock);
            ProcessFeatures.AddFeatureList(featuresInBlock);
        }
    }
}
