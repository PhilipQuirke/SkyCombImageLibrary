// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using Compunet.YoloV8.Data;
using SkyCombGround.GroundLogic;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold all Yolo feature and block data associated with a video
    // Ignores thermal threshold.
    public class YoloProcess : ProcessAll
    {
        // YOLO (You only look once) V8 image processing
        public YoloDetect YoloDetect;

        // List of features detected in each frame in a leg by YoloDetect 
        public YoloFeatureSeenList LegFrameFeatures;

        // If UseFlightLegs, how many significant objects have been found in this FlightLeg?
        public int FlightLeg_SigObjects { get; set; }
        public int FlightLeg_StartObjects { get; set; }



        public YoloProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, string yoloDirectory) : base(ground, video, drone, config)
        {
            YoloDetect = new YoloDetect(yoloDirectory, config.YoloDetectConfidence, config.YoloIoU);
            LegFrameFeatures = new();
            FlightLeg_SigObjects = 0;
            FlightLeg_StartObjects = 0;
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        protected override void ProcessStart()
        {
            LegFrameFeatures.Clear();
            FlightLeg_SigObjects = 0;
            FlightLeg_StartObjects = 0;

            base.ProcessStart();
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public override void EnsureObjectsNamed()
        {
            ProcessObjects.EnsureObjectsNamed();
        }


        // For process robustness, we want to process each leg independently.
        protected override void ProcessFlightLegStart(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                ProcessObjects.StopTracking();
                LegFrameFeatures.Clear();
                FlightLeg_SigObjects = 0;
                FlightLeg_StartObjects = ProcessObjects.Count;
            }
        }


        public YoloObject AddYoloObject(ProcessScope scope, int legId, YoloFeature firstFeature)
        {
            BaseConstants.Assert(firstFeature != null, "YoloObjectList.AddObject: No firstFeature");

            string className = firstFeature.Class != null? firstFeature.Class.Name : "??";
            float classConfidence = firstFeature.Confidence;
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
            if (Drone.UseFlightLegs)
            {
                FlightLeg_SigObjects = 0;
                ProcessObjList legObjs = new();

                if (LegFrameFeatures.Count > 0)
                {
                    // During the leg, we found features using Yolo image detection,
                    // and defined objects using IoU overlap in successive frames.

                    double yMovePerTimeSlice = 10; // Estiamte of number of y pixels per frame  // PQR TODO

                    var objectsSeen = YoloTracker.CalculateObjectsInLeg(yMovePerTimeSlice, LegFrameFeatures);

                    // For each ObjectSeen, create a YoloObject
                    foreach (var objSeen in objectsSeen)
                    {
                        Assert(objSeen.Features.Any(), "YoloObject has no features");

                        var firstFeature = ProcessFeatures[ objSeen.Features[0].FeatureId ];
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

                FlightLeg_SigObjects = EnsureObjectsNamed(0, legObjs, null);

                // For process robustness, we want to process each leg independently.
                LegFrameFeatures.Clear();
            }

            // Post process the objects found in the leg & maybe set FlightLegs.FixAltM 
            base.ProcessFlightLegEnd(scope, legId);

            foreach (var feature in ProcessFeatures)
                feature.Value.Significant = true;
        }


        // Process the image using YOLO to find features. 
        // For each feature found, if it overlaps an object (from previous frame), object claims the feature.
        // Else create a new object to own the feature.
        public int ProcessBlock(
            ProcessScope scope,
            Image<Gray, byte> currGray,
            Image<Bgr, byte> imgOriginal,
            Image<Gray, byte> imgThreshold,
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
                    var newFeature = new YoloFeature(this, blockID, box);
                    newFeature.CalculateHeat(imgOriginal, imgThreshold);
                    featuresInBlock.AddFeature(newFeature);
                    LegFrameFeatures.Add(new YoloFeatureSeen { BlockId = blockID, Box = newFeature.PixelBox, FeatureId = newFeature.FeatureId });
                }

                Phase = 5;
                currBlock.AddFeatureList(featuresInBlock);
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
