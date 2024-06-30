// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold all feature data and Block data associated with a video
    public class CombProcess : ProcessAll
    {
        // Ground (DEM, DSM and Swathe) data under the drone flight path
        public GroundData GroundData { get; set; }

        // List of comb features found. Each feature is a cluster of hot pixels, with a bounding retangle
        public CombFeatureList CombFeatures { get; set; }

        // List of comb objects found. Each is a logical object derived from overlapping features over successive frames. 
        public CombObjs CombObjs { get; set; }

        // List of CombSpans that analsyse CombObjects to generate FixAltM data
        public CombSpanList CombSpans { get; set; }

        // If UseFlightLegs, how many significant objects have been found in this FlightLeg?
        public int FlightLeg_SigObjects { get; set; }

        // If !UseFlightLegs, we need to generate CombSpans for each cluster of significant objects (over a series of FlightSteps)
        public int FlightSteps_PrevSigObjects { get; set; }
        public int FlightSteps_MinStepId { get; set; }
        public int FlightSteps_MaxStepId { get; set; }


        private void ResetCombSpanData()
        {
            FlightSteps_PrevSigObjects = 0;
            FlightSteps_MinStepId = UnknownValue;
            FlightSteps_MaxStepId = UnknownValue;
        }


        public CombProcess(ProcessConfigModel config, VideoData video, GroundData groundData, Drone drone) : base(config, video, drone)
        {
            CombFeature.NextFeatureId = 0;

            GroundData = groundData;
            CombFeatures = new(config);
            CombObjs = new(this);
            CombSpans = new();
            FlightLeg_SigObjects = 0;
            ResetCombSpanData();
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public override void ResetModel()
        {
            CombFeature.NextFeatureId = 0;
            FlightLeg_SigObjects = 0;
            ResetCombSpanData();

            CombFeatures.Clear();
            CombObjs.CombObjList.Clear();
            CombSpans.Clear();

            base.ResetModel();

            // We want the category information to be retained between runs, so don't clear it.
            // Categories.Clear();
            // ObjectCategories.Clear();
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public override void EnsureObjectsNamed()
        {
            foreach (var theObject in CombObjs.CombObjList)
                if (theObject.Value.Name == "")
                    theObject.Value.SetName();
        }


        public override void ProcessFlightLegStart(int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For "Comb" process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                CombObjs.StopTracking();
                FlightLeg_SigObjects = 0;
            }
        }


        public override void ProcessFlightLegEnd(int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For "Comb" process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                CombObjs.StopTracking();

                // If we are lacking the current CombSpan then create it.
                if ((legId > 0) && !CombSpans.TryGetValue(legId, out _))
                {
                    // Post process the objects found in the leg & maybe set FlightLegs.FixAltM 
                    var combSpan = ProcessFactory.NewCombSpan(this, legId);
                    CombSpans.AddSpan(combSpan);
                    combSpan.CalculateSettings_from_FlightLeg();
                    combSpan.AssertGood();
                }
            }

            EnsureObjectsNamed();
        }


        // If we have been tracking some significant objects, create a CombSpan for them
        public void Process_CombSpan_Create()
        {
            if ((!Drone.UseFlightLegs) && (FlightSteps_PrevSigObjects > 0))
            {
                var combSpan = ProcessFactory.NewCombSpan(this, CombSpans.Count() + 1);
                CombSpans.AddSpan(combSpan);
                combSpan.CalculateSettings_from_FlightSteps(FlightSteps_MinStepId, FlightSteps_MaxStepId);
                ResetCombSpanData();
            }
        }


        // Add a new block
        public ProcessBlock AddCombBlock(ProcessScope scope)
        {
            var currBlock = ProcessFactory.NewBlock(scope);

            Blocks.AddBlock(currBlock, scope, Drone);

            return currBlock;
        }


        // Create an unreal feature, with no pixels, with a rectangle calculated from the object's 
        // last bounding rectangle, maximum pixel box, and the average object movement.
        public void AddPersistFeature(CombObject theObject)
        {
            var theBlock = Blocks.LastBlock;

            CombFeature theFeature = new(this, theBlock, FeatureTypeEnum.Unreal);
            theFeature.PixelBox = theObject.ExpectedLocationThisBlock();
            CombFeatures.AddFeature(theFeature);

            Assert(Blocks.Count == theFeature.Block.BlockId, "AddPersistFeature: Bad Blocks count");

            theObject.ClaimFeature(theFeature);
        }


        // Save memory (if compatible with Config settings) by deleting pixel data
        // For Comb, we only care about pixel data for objects / features in legs
        public void DeleteFeaturePixelsForObjects()
        {
            if (ProcessConfig.SavePixels == SavePixelsEnum.None)
                foreach (var theObject in CombObjs.CombObjList)
                    if (theObject.Value.FlightLegId <= 0)
                        foreach (var feature in theObject.Value.Features)
                            feature.Value.Pixels?.Clear();
        }


        // If !UseFlightLegs, track significant objects, based on their corresponding FlightSteps.
        // Does not make use of FlightLegs.
        private void ProcessObjectsFlightSteps(CombObjList inScopeObjects, ProcessBlock currBlock)
        {
            if (!Drone.UseFlightLegs)
            {
                foreach (var theObject in inScopeObjects)
                    if (theObject.Value.Name == "")
                        theObject.Value.SetName();

                int sigObjects = inScopeObjects.NumSignificantObjects();
                if (sigObjects > 0)
                {
                    if (FlightSteps_PrevSigObjects == 0)
                        // COMB SPAN START EVENT
                        // Object(s) have just become significant. Last frame there were no significant objects.
                        // It takes a few steps to become significant, so get the minimal FlightStep.StepID of the object(s).
                        // CombSpan_MinFlightStepId is the starting step of a future CombSpan object.
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
                            // COMB SPAN END EVENT
                            // We tracked some significant objects, then they all became insignificant, and 8 frames have passed
                            Process_CombSpan_Create();

                            ResetCombSpanData();
                        }
                    }
                }
            }
        }


        // Process the features found in the current block/frame, which is part of a leg,
        // by preference adding them to existing objects (created in previous blocks/frames),
        // else creating new objects to hold the features.
        public void ProcessBlockForObjects(ProcessScope scope, CombFeatureList featuresInBlock)
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
                CombObjList inScopeObjects = new();
                CombObjList availObjects = new();
                foreach (var theObject in CombObjs.CombObjList)
                    if ((theObject.Value.LastFeature != null) &&
                        (theObject.Value.LastFeature.Block.BlockId == blockID - 1) &&
                        theObject.Value.VaguelySignificant())
                    {
                        inScopeObjects.AddObject(theObject.Value);
                        availObjects.AddObject(theObject.Value);
                    }

                // Each feature can only be claimed once
                CombFeatureList availFeatures = featuresInBlock.Clone();


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
                                if (theObject.Value.MaybeClaimFeature(feature.Value, expectedObjectLocation))
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
                            theObject.Value.MaybeClaimFeature(feature.Value, expectedObjectLocation);
                    }


                Phase = 5;
                currBlock.AddFeatureList(featuresInBlock);
                CombFeatures.AddFeatureList(featuresInBlock);

                // For each active object, where the above code did not find an 
                // overlapping feature in this Block, if it is worth continuing tracking...
                Phase = 6;
                foreach (var theObject in inScopeObjects)
                    if (theObject.Value.COM.BeingTracked &&
                       (theObject.Value.COM.LastRealFeatureIndex != UnknownValue) &&
                       (theObject.Value.LastRealFeature.Block.BlockId < blockID) &&
                       theObject.Value.KeepTracking(blockID))
                        // ... persist this object another Block. Create an unreal feature, with no pixels, with a rectangle   
                        // calculated from the object's last bounding rectangle and the average frame movement.
                        AddPersistFeature(theObject.Value);


                // All active features have passed the min pixels and min density tests, and are worth tracking.
                // For all unowned active features in this frame, create a new object to own the feature.
                Phase = 7;
                foreach (var feature in availFeatures)
                    if (feature.Value.IsTracked && (feature.Value.ObjectId == 0))
                    {
                        CombObjs.Add(scope, feature.Value);
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
        public void ProcessBlockForFeatures(CombFeatureList featuresInBlock)
        {
            foreach (var feature in featuresInBlock)
            {
                feature.Value.IsTracked = false;
                feature.Value.Significant = false;
            }

            Blocks.LastBlock.AddFeatureList(featuresInBlock);
            CombFeatures.AddFeatureList(featuresInBlock);
        }


        override public DataPairList GetSettings()
        {
            int numPixels = 0;
            foreach (var feature in CombFeatures)
                if (feature.Value.Pixels != null)
                    numPixels += feature.Value.Pixels.Count;

            return new DataPairList
            {
                { "# Blocks", Blocks.Count },
                { "# Objects", CombObjs.CombObjList.Count },
                { "# Significant Objects", CombObjs.NumSig },
                { "# Features", CombFeatures.Count },
                { "# Significant Features", CombFeatures.NumSig },
                { "# Pixels", numPixels },
            };
        }
    }
}
