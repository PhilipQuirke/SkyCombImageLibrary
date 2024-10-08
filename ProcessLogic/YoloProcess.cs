// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using System.Drawing;
using YoloDotNet.Models;


namespace SkyCombImage.ProcessLogic
{
    // Process video frames, gathering Yolo features and block data, and generating ProcessFeatures
    public class YoloProcess : ProcessAll
    {
        // YOLO (You only look once) V8 image processing
        public YoloDetect YoloDetect;

        // For speed, do we process all frames in the video or just the user specified range?
        public bool YoloProcessAllFrames;

        // List of features detected in each frame in a leg by YoloDetect 
        public YoloFeatureSeenList LegFrameFeatures;


        public YoloProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, string yoloPath) : base(ground, video, drone, config)
        {
            YoloDetect = new YoloDetect(yoloPath, config.YoloDetectConfidence, config.YoloIoU);
            YoloProcessAllFrames = false;
            LegFrameFeatures = new();
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public override void RunStart()
        {
            LegFrameFeatures.Clear();

            base.RunStart();
        }


        // For process robustness, we want to process each leg independently.
        protected override void ProcessFlightLegStart(ProcessScope scope, int legId)
        {
            base.ProcessFlightLegStart(scope, legId);

            if (Drone.UseFlightLegs)
                LegFrameFeatures.Clear();
        }


        public YoloObject AddYoloObject(ProcessScope scope, int legId, YoloFeature firstFeature)
        {
            BaseConstants.Assert(firstFeature != null, "YoloObjectList.AddObject: No firstFeature");

            string className = firstFeature.Label != null ? firstFeature.Label.Name : "??";
            double classConfidence = firstFeature.Confidence;
            /*
            newObject.ClassId = box.Class.Id;
            newObject.ClassType = box.Class.Type;
            newObject.ClassDescription = box.Class.Description;
            */

            var answer = new YoloObject(this, scope, legId, firstFeature, className, Color.Red, classConfidence);
            ProcessObjects.AddObject(answer);
            return answer;
        }


        protected override void ProcessFlightLegEnd(ProcessScope scope, int legId)
        {
            try
            {
                if (Drone.UseFlightLegs)
                {
                    ProcessObjList legObjs = new();

                    if (LegFrameFeatures.Count > 0)
                    {
                        // During the leg, we found features using Yolo image detection,
                        // and defined objects using IoU overlap in successive frames.

                        double yMovePerTimeSlice = 10; // Estimate of number of y pixels moved per frame  // PQR TODO

                        var objectsSeen = YoloTracker.CalculateObjectsInLeg(yMovePerTimeSlice, LegFrameFeatures);

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

                    EnsureObjectsNamed(legObjs, null);

                    // For process robustness, we want to process each leg independently.
                    LegFrameFeatures.Clear();
                }

                // Post process the objects found in the leg & maybe set FlightLegs.FixAltM 
                base.ProcessFlightLegEnd(scope, legId);

                foreach (var feature in ProcessFeatures)
                    feature.Value.Significant = true;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloProcess.FlightLegEnd" +
                    "(Leg=" + legId.ToString() +
                    ",CurrBlock = " + scope.PSM.CurrBlockId +
                    ",LastBlock=" + scope.PSM.LastBlockId + ")", ex);
            }
        }


        // Process the image using YOLO to find features. 
        // For each feature found, if it overlaps an object (from previous frame), object claims the feature.
        // Else create a new object to own the feature.
        public virtual int ProcessBlock(
            ProcessScope scope,
            in Image<Bgr, byte> imgOriginal, // read-only
            in Image<Gray, byte> imgThreshold, // read-only
            List<ObjectDetection>? results)
        {
            int Phase = 0;
            int blockID;

            try
            {
                if (results == null)
                    return 0;
                var numSig = results.Count();
                if (numSig == 0)
                    return 0;

                Phase = 1;
                var currBlock = Blocks.LastBlock;
                blockID = currBlock.BlockId;

                // Convert Yolo Bounding Boxes to YoloFeatures
                Phase = 2;
                ProcessFeatureList featuresInBlock = new(this.ProcessConfig);
                foreach (var result in results)
                {
                    // We have found a new feature/object
                    var newFeature = new YoloFeature(this, blockID, result);
                    Phase = 3;
                    newFeature.CalculateHeat(imgOriginal, imgThreshold);
                    Phase = 4;
                    featuresInBlock.AddFeature(newFeature);
                    Phase = 5;
                    LegFrameFeatures.Add(new YoloFeatureSeen { BlockId = blockID, Box = newFeature.PixelBox, FeatureId = newFeature.FeatureId });
                }

                Phase = 6;
                currBlock.AddFeatureList(featuresInBlock);
                Phase = 7;
                ProcessFeatures.AddFeatureList(featuresInBlock);

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
    };
}
