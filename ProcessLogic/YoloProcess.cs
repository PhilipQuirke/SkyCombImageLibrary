// Copyright SkyComb Limited 2025. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;
using System.Windows.Forms;
using YoloDotNet.Models;


namespace SkyCombImage.ProcessLogic
{
    // Process video frames, gathering Yolo features and block data, and generating ProcessFeatures
    public class YoloProcess : ProcessAll
    {
        // YOLO (You only look once) V8 image processing
        public YoloDetect YoloDetect;

        // List of features detected in each frame in a leg by YoloDetect 
        public YoloFeatureSeenList LegFrameFeatures;


        public YoloProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, RunUserInterface runUI, string yoloPath) : base(ground, video, drone, config, runUI)
        {
            YoloDetect = new YoloDetect(yoloPath, config.YoloDetectConfidence, config.YoloIoU);
            LegFrameFeatures = new();
        }


        public override ProcessFeature? NewPersistFeature()
        {
            return new YoloFeature(this, Blocks.LastBlock.BlockId, null, FeatureTypeEnum.Unreal);
        }


        // Reset any internal state of the model 
        public override void RunStart(ProcessScope scope)
        {
            LegFrameFeatures.Clear();

            base.RunStart(scope);
        }


        // For process robustness, we want to process each leg independently.
        protected override void ProcessFlightLegStart(ProcessScope scope, int legId)
        {
            base.ProcessFlightLegStart(scope, legId);

            if (Drone.UseFlightLegs)
                LegFrameFeatures.Clear();
        }


        protected override void ProcessFlightLegEnd(ProcessScope scope, int legId, SkyCombImageLibrary.ProcessLogic.ProcessObjectParameters parameters)
        {
            base.ProcessFlightLegEnd(scope, legId, parameters);
        }


        public List<ObjectDetection>? YoloDetectImage(Bitmap currBmp, ProcessBlock thisBlock)
        {
            return YoloDetect.DetectFrame(currBmp);
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
                {
                    var thisFeature = feature.Value as YoloFeature;
                    if (thisFeature.IsTracked && (thisFeature.ObjectId == 0))
                    {
                        var theObject = ProcessFactory.NewYoloObject(this, scope, scope.PSM.CurrRunLegId, thisFeature, thisFeature.Label.Name, Color.Red, thisFeature.Confidence);

                        ProcessObjects.AddObject(theObject);
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
                throw ThrowException("Yolo.ProcessBlockForObjects.Phase=" + Phase, ex);
            }
        }
    };
}
