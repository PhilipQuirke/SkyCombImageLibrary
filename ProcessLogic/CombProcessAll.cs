// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold all feature data and Block data associated with a video
    public class CombProcessAll : ProcessAll
    {
        // Ground (DEM, DSM and Swathe) data under the drone flight path
        public GroundData GroundData { get; set; }

        // List of Blocks (aka frames processed)  
        public ProcessBlockList Blocks { get; set; }

        // List of comb features found. Each feature is a cluster of hot pixels, with a bounding retangle
        public CombFeatureList CombFeatures { get; set; }

        // List of comb objects found. Each is a logical object derived from overlapping features over successive frames. 
        public CombObjs CombObjs { get; set; }

        // List of CombLegs that analsyse CombObjects found in the leg to refine FlightLeg etc data.
        public CombLegList CombLegs { get; set; }

        // How many significant objects have been found in this leg?
        private int LegSignificantObjects { get; set; } = 0;



        public CombProcessAll(ProcessConfigModel config, VideoData video, GroundData groundData, Drone drone) : base(config, video, drone)
        {
            CombFeature.Config = config;
            CombObject.Config = config;

            CombFeature.NextFeatureId = 0;

            GroundData = groundData;
            Blocks = new();
            CombFeatures = new();
            CombObjs = new(this);
            CombLegs = new();
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public override void ResetModel()
        {
            CombFeature.NextFeatureId = 0;
            LegSignificantObjects = 0;

            Blocks.Clear();
            CombFeatures.Clear();
            CombObjs.CombObjList.Clear();
            CombLegs.Clear();

            base.ResetModel();

            // We want the category information to be retained between runs, so don't clear it.
            // Categories.Clear();
            // ObjectCategories.Clear();
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public override void EnsureObjectsNamed(Drone drone)
        {
            foreach (var theObject in CombObjs.CombObjList)
                if (theObject.Value.Name == "")
                    theObject.Value.SetName();
        }


        public override void ProcessLegStart(int legId, Drone drone)
        {
            if (drone.UseFlightLegs)
            {
                // For "Comb" process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                CombObjs.StopTracking();
                LegSignificantObjects = 0;
/*
                // If the FlightLeg has a FixAltitudeM value (from a previous processing run) then use it,
                // else use the FixAltitudeM from the previous leg (if any) as a starting point.
                if ((legId >= 2) &&
                    (drone.FlightLegs.Legs[legId - 1].FixAltitudeM == 0))
                    drone.FlightLegs.Legs[legId - 1].FixAltitudeM =
                        drone.FlightLegs.Legs[legId - 2].FixAltitudeM;
*/
            }
        }


        public override void ProcessLegEnd(int legId, Drone drone)
        {
            if (drone.UseFlightLegs)
            {
                // For "Comb" process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                CombObjs.StopTracking();

                if (legId > 0)
                {
                    // If we are lacking the current CombLeg then create it.
                    CombLeg combLeg;
                    if (!CombLegs.TryGetValue(legId, out combLeg))
                    {
                        // Post process the objects found in the leg & maybe set FlightLegs.FixAltitudeM 
                        combLeg = ProcessFactory.NewCombLeg(this, legId, drone);
                        CombLegs.Add(combLeg);
                        combLeg.CalculateSettings(VideoData, drone);
                        combLeg.AssertGood();
                    }
                }
            }

            EnsureObjectsNamed(drone);
        }


        // Add a new block
        public ProcessBlock AddCombBlock(ProcessScope scope, Drone drone)
        {
            var currBlock = ProcessFactory.NewBlock(scope);

            Blocks.AddBlock(currBlock, scope, drone);

            return currBlock;
        }


        // Create an unreal feature, with no pixels, with a rectangle calculated from the object's 
        // last bounding rectangle, maximum pixel box, and the average object movement.
        public void AddPersistFeature(CombObject theObject)
        {
            var theBlock = Blocks.LastBlock;

            CombFeature theFeature = new(this, theBlock, CombFeatureTypeEnum.Unreal);
            theFeature.CFM.PixelBox = theObject.ExpectedLocationThisBlock();
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
                    if (theObject.Value.LegId <= 0)
                        foreach (var feature in theObject.Value.Features)
                            feature.Value.Pixels?.Clear();
        }


        // Process the features found in the current block/frame, which is part of a leg,
        // by preference adding them to existing objects (created in previous blocks/frames),
        // else creating new objects to hold the features.
        public void ProcessBlockForObjects(ProcessScope scope, CombFeatureList featuresInBlock)
        {
            var currBlock = Blocks.LastBlock;
            int blockID = currBlock.BlockId;


            // We only want to consider objects that are still active.
            // For long flights most objects will have become inactive seconds or minutes ago.
            CombObjList inScopeObjects = new();
            CombObjList availObjects = new();
            foreach (var theObject in CombObjs.CombObjList)
                if ((theObject.Value.LastFeature() != null) &&
                    (theObject.Value.LastFeature().Block.BlockId == blockID - 1) &&
                    theObject.Value.VaguelySignificant())
                {
                    inScopeObjects.AddObject(theObject.Value);
                    availObjects.AddObject(theObject.Value);
                }
            // Each feature can only be claimed once
            CombFeatureList availFeatures = featuresInBlock.Clone();


            // For each active object, consider each frame feature (significant or not)
            // found in this frame to see if it overlaps.
            // This priviledges objects with multiple real features,
            // as they can more accurately estimate their expected location.
            // We priviledge objects with a "real" last feature over objects with a "unreal" last feature.
            for (int pass = 0; pass < 2; pass++)
                foreach (var theObject in inScopeObjects)
                {
                    var lastFeat = theObject.Value.LastFeature();
                    if((lastFeat.Block.BlockId == blockID - 1) &&
                        (pass == 0 ? lastFeat.CFM.Type == CombFeatureTypeEnum.Real : lastFeat.CFM.Type != CombFeatureTypeEnum.Unreal))
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
            foreach (var theObject in availObjects)
                if(theObject.Value.NumRealFeatures() <= 2)
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


            currBlock.AddFeatureList(featuresInBlock);
            CombFeatures.AddFeatureList(featuresInBlock);

            // For each active object, where the above code did not find an 
            // overlapping feature in this Block, if it is worth continuing tracking...
            foreach (var theObject in inScopeObjects)
                if (theObject.Value.COM.BeingTracked &&
                   (theObject.Value.COM.LastRealFeatureIndex != UnknownValue) &&
                   (theObject.Value.LastRealFeature().Block.BlockId < blockID) &&
                   theObject.Value.KeepTracking(blockID))
                    // ... persist this object another Block. Create an unreal feature, with no pixels, with a rectangle   
                    // calculated from the object's last bounding rectangle and the average frame movement.
                    AddPersistFeature(theObject.Value);


            // All active features have passed the min pixels and min density tests, and are worth tracking.
            // For all unowned active features in this frame, create a new object to own the feature.
            foreach (var feature in availFeatures)
                if (feature.Value.IsTracked && (feature.Value.ObjectId == 0))
                {
                    CombObjs.Add(scope, feature.Value);
                    if (blockID >= 2)
                    {
                        // TODO: Consider claiming overship of overlapping inactive features from the previous Block(s).
                    }
                }


            // Ensure each significant object in this leg has a "significant" name e.g. C5
            // Needs to be done ASAP so the "C5" name can be drawn on video frames.
            // Note: Some objects never become significant.
            foreach (var theObject in inScopeObjects)
                if ((theObject.Value.LegId > 0) &&
                   ((scope.CurrRunFlightStep == null) || (theObject.Value.LegId == scope.CurrRunFlightStep.LegId)) &&
                   (theObject.Value.Significant) &&
                   (theObject.Value.Name == ""))
                {
                    LegSignificantObjects++;
                    theObject.Value.SetName(LegSignificantObjects);
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


        public DataPairList GetSettings()
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


        // Add object-specific metrics to the drone-specific metrics to display
        // in FlightPath form, aligned with the scope showing in the form now.
        public void AddSettings_FlightPath(ProcessScope processScope, int groundSeenM2, ref DataPairList metrics)
        {
            var combObjList = CombObjs.CombObjList.GetSignificantByScope(processScope);
            if ((combObjList == null) || (combObjList.Count == 0))
                return;

            combObjList.CalculateSettings(combObjList);

            metrics.Add("# Objects", combObjList.Count);
            metrics.Add("# Objects per KM2", (groundSeenM2 <= 0 ? 0 : 1000000.0f * combObjList.Count / groundSeenM2), 0);
            metrics.Add("Avg Obj Height M", combObjList.AvgHeightM, HeightNdp);
            metrics.Add("Max Obj Size CM2", combObjList.MaxSizeCM2, AreaCM2Ndp);
            metrics.Add("Min Obj Size CM2", combObjList.MinSizeCM2, AreaCM2Ndp);
        }
    }
}
