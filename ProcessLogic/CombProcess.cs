// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold all feature data and Block data associated with a video
    public class CombProcess : ProcessAll
    {
        public CombProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config) : base(ground, video, drone, config)
        {
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


        // Save memory by deleting pixel data
        public void DeleteFeaturePixelsForObjects()
        {
            foreach (var theObject in ProcessObjects)
            {
                var combObject = theObject.Value;
                combObject.ClearHotPixels();
            }
        }


        // If !UseFlightLegs, track significant objects, based on their corresponding FlightSteps.
        // Does not make use of FlightLegs.
        private void ProcessObjectsFlightSteps(ProcessObjList inScopeObjects, ProcessBlock currBlock)
        {
            if (!Drone.UseFlightLegs)
            {
                inScopeObjects.EnsureObjectsNamed();

                int sigObjects = inScopeObjects.NumSignificantObjects;
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
                (var inScopeObjects, var availObjects, var availFeatures) = ProcessBlockForObjects_Core(scope, featuresInBlock);
                var currBlock = Blocks.LastBlock;
                int blockID = currBlock.BlockId;


                // For each active object, where the above code did not find an 
                // overlapping feature in this Block, if it is worth continuing tracking...
                Phase = 10;
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
                Phase = 11;
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
                    Phase = 12;
                    EnsureObjectsNamed(inScopeObjects, scope.CurrRunFlightStep);
                }
                else
                {
                    // Track data related to a ProcessSpan (not a FlightLeg)
                    Phase = 13;
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
            try
            {
                foreach (var feature in featuresInBlock)
                {
                    feature.Value.IsTracked = false;
                    feature.Value.Significant = false;
                }

                Blocks.LastBlock.AddFeatureList(featuresInBlock);
                ProcessFeatures.AddFeatureList(featuresInBlock);
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessBlockForFeatures", ex);
            }
        }
    }
}
