// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold all feature data and Block data associated with a video
    public class CombProcess : ProcessAll
    {
        public CombProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, RunUserInterface runUI) : base(ground, video, drone, config, runUI)
        {
        }


        public override ProcessFeature? NewPersistFeature()
        {
            return new CombFeature(this, Blocks.LastBlock, FeatureTypeEnum.Unreal);
        }


        // Save memory by deleting pixel data
        public void DeleteFeaturePixelsForObjects()
        {
            foreach (var theObject in ProcessObjects)
            {
                var combObject = theObject.Value;
                combObject.ClearHotPixelArray();
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


                // All active features have passed the min pixels test, and are worth tracking.
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
                    FlightLeg_SigObjects = EnsureObjectsNamed(FlightLeg_SigObjects, inScopeObjects, scope.CurrRunFlightStep);
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
                throw ThrowException("Comb.ProcessBlockForObjects.Phase=" + Phase, ex);
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
