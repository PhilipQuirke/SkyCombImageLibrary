// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;
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


        // Do pre-run processing 
        public override void PreRunStart(ProcessScope scope)
        {
            YoloDetect.ProcessMode = YoloProcessMode.ByFrame;
            return;

            if (Drone.UseFlightLegs)
            {
                // "Frame inaccuracies" builds up when processing long sequences of frames.
                // To minimise this, we reset the video frame position at the start of each leg.
                // As we only try to detect objects on legs this is not a problem.
                YoloDetect.ProcessMode = YoloProcessMode.TimeRange;
                YoloDetect.Results = new();

                // For each flight leg that overlaps the scope, detect hotspots over the entire leg, using YOLO and GPU.
                foreach (var leg in Drone.FlightLegs.Legs)
                {
                    if ((leg.MinSumTimeMs >= scope.PSM.LastVideoFrameMs) || (leg.MaxSumTimeMs <= scope.PSM.FirstVideoFrameMs))
                        continue;

                    RunUI.ShowRunSummary("Pre-run processing: Leg " + leg.Name);

                    var legResults = YoloDetect.DetectTimeRange(Drone.InputVideo.FileName, leg.MinSumTimeMs / 1000, leg.MaxSumTimeMs / 1000 + 1);
                    if (legResults != null)
                        foreach (var kvp in legResults)
                            if (kvp.Value.Count > 0)
                            {
                                int stepID = leg.MinStepId + kvp.Key;
                                if (stepID <= leg.MaxStepId)
                                    YoloDetect.Results.Add(stepID, kvp.Value);
                            }

                    // Temp. Try to avoid YoloDotNet exceptions
                    System.Threading.Thread.Sleep(500);
                }
            }
            else
                // Process the entire scope, using YOLO and GPU. Do not create an output file yet.
                YoloDetect.ProcessScope(scope, Drone.InputVideo);
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


        protected override void ProcessFlightLegEnd(ProcessScope scope, int legId)
        {
            base.ProcessFlightLegEnd(scope, legId);
        }


        public YoloObject AddYoloObject(ProcessScope scope, int legId, YoloFeature firstFeature)
        {
            BaseConstants.Assert(firstFeature != null, "YoloObjectList.AddObject: No firstFeature");

            string className = firstFeature.Label != null ? firstFeature.Label.Name : "??";
            double classConfidence = firstFeature.Confidence;

            var answer = new YoloObject(this, scope, legId, firstFeature, className, Color.Red, classConfidence);
            ProcessObjects.AddObject(answer);
            return answer;
        }


        private void AddYoloObjectAndFeature(YoloObjectSeenList objectsSeen, ProcessScope scope, int legId, ProcessObjList legObjs)
        {
            // For each ObjectSeen, create a YoloObject
            foreach (var objSeen in objectsSeen)
            {
                Assert(objSeen.Features.Any(), "YoloObject has no features");

                var firstFeature = ProcessFeatures[objSeen.Features[0].FeatureId];
                firstFeature.CalculateSettings_LocationM_FlatGround(null);
                firstFeature.CalculateSettings_LocationM_HeightM_LineofSight(GroundData);
                YoloObject newObject = AddYoloObject(scope, legId, firstFeature as YoloFeature);
                legObjs.AddObject(newObject);

                // Add remaining features to the object
                for (int i = 1; i < objSeen.Features.Count; i++)
                {
                    var theFeature = ProcessFeatures[objSeen.Features[i].FeatureId];
                    theFeature.CalculateSettings_LocationM_FlatGround(null);
                    theFeature.CalculateSettings_LocationM_HeightM_LineofSight(GroundData);
                    newObject.ClaimFeature(theFeature);
                }
            }
        }


        public List<ObjectDetection>? YoloDetectImage(Bitmap currBmp, ProcessBlock thisBlock)
        {
            if (YoloDetect.ProcessMode == YoloProcessMode.ByFrame)
                return YoloDetect.DetectFrame(currBmp);

            try
            {
                return YoloDetect.Results[thisBlock.BlockId];
            }
            catch
            {
                return null;
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
