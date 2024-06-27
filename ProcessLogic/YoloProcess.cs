// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using System.Drawing;
using Compunet.YoloV8.Data;



namespace SkyCombImage.ProcessModel
{


    // A Yolo feature 
    public class YoloFeature : ProcessFeatureModel
    {
        // Static data shared by all Yolo features
        private static YoloProcess YoloProcess;


        public YoloFeature(YoloProcess yoloProcess, int blockId, Rectangle imagePixelBox) : base(blockId, CombFeatureTypeEnum.Real)
        {
            ResetMemberData();
            PixelBox = imagePixelBox;
            Significant = true;
            YoloProcess = yoloProcess;
        }


        // Does this Feature's PixleBox and the specified object's rectangle overlap significantly?
        public bool SignificantPixelBoxIntersection(Rectangle objectExpectedPixelBox)
        {
            return base.SignificantPixelBoxIntersection(objectExpectedPixelBox, YoloProcess.ProcessConfig.FeatureMinOverlapPerc);
        }
    };


    public class YoloFeatureList : List<YoloFeature>
    {
        private YoloProcess YoloProcess;


        public YoloFeatureList(YoloProcess yoloProcess)
        {
            YoloProcess = yoloProcess;
        }


        public YoloFeature AddFeature(int blockId, Rectangle imagePixelBox)
        {
            var answer = new YoloFeature(YoloProcess, blockId, imagePixelBox);
            this.Add(answer);
            return answer;
        }
    };


    // A class to hold a Yolo object - layer over a sequence of Yolo features.
    public class YoloObject : ProcessObject
    {
        public string ClassName { get; set; }
        public Color ClassColor { get; set; }
        public float ClassConfidence { get; set; }
   
        // First feature claimed by this object
        public YoloFeature FirstFeature { get; set; }
        // Last feature claimed by this object
        public YoloFeature LastFeature { get; set; }


        public YoloObject(YoloProcess yoloProcess, ProcessScope scope, YoloFeature firstFeature, string className, Color classColor, float classConfidence) : base(yoloProcess.ProcessConfig, scope)
        {
            ClassName = className;
            ClassColor = classColor;
            ClassConfidence = classConfidence;
            FirstFeature = firstFeature;
            LastFeature = firstFeature;
            Significant = true;
        }


        // This object claims this feature
        public void ClaimFeature(YoloFeature theFeature)
        {
            Assert(theFeature.ObjectId <= 0, "YoloObject.ClaimFeature: Feature is already owned.");
            Assert(FirstFeature != null, "YoloObject.ClaimFeature: FirstFeature is null.");

            LastFeature = theFeature;

            NumSigBlocks = LastFeature.BlockId - FirstFeature.BlockId + 1;
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        override public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Object", ObjectId },
                { "FromS", RunFromVideoS, SecondsNdp },
                { "ToS", RunToVideoS, SecondsNdp },
                { "Attributes", Attributes },
                { "Significant", Significant },
                { "#SigBlocks", NumSigBlocks },
                { "FirstFeat", (FirstFeature == null) ? -1 : FirstFeature.FeatureId },
                { "LastFeat", (LastFeature == null) ? -1 : LastFeature.FeatureId },
            };
        }
    };


    public class YoloObjectList : List<YoloObject>
    {
        private YoloProcess YoloProcess;


        // We do not process objects below this index in the object array.
        public int LegFirstIndex;


        public YoloObjectList(YoloProcess yoloProcess)
        {
            YoloProcess = yoloProcess;
        }


        public YoloObject AddObject(ProcessScope scope, YoloFeature firstFeature, string className, Color classColor, float classConfidence)
        {
            var answer = new YoloObject(YoloProcess, scope, firstFeature, className, classColor, classConfidence);
            Add(answer);
            return answer;
        }


        // No existing objects should be live at the start of a new leg
        public void ProcessLegStart()
        {
            foreach (var theObject in this)
                theObject.Significant = false;

            LegFirstIndex = Count;
        }
    };



    // A class to hold all Yolo feature and block data associated with a video
    public class YoloProcess : ProcessAll
    {
        public ProcessBlockList YoloBlocks;
        public YoloObjectList YoloObjects;
        public YoloFeatureList YoloFeatures;

        // YOLO (You only look once) V8 image processing
        public YoloDetect YoloDetect;


        public YoloProcess(ProcessConfigModel config, Drone drone, string yoloDirectory) : base(config, drone.InputVideo, drone)
        {
            YoloBlocks = new();
            YoloObjects = new(this);
            YoloFeatures = new(this);
            YoloObjects.LegFirstIndex = 0;

            YoloDetect = new YoloDetect(yoloDirectory);
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public override void ResetModel()
        {
            YoloBlocks.Clear();
            YoloFeatures.Clear();
            YoloObjects.Clear();

            YoloObjects.LegFirstIndex = 0;

            base.ResetModel();
        }


        // For process robustness, we want to process each leg independently.
        public override void ProcessFlightLegStart(int legId)
        {
            // No existing objects should be live at the start of a new leg
            YoloObjects.ProcessLegStart();
        }


        // Create a Yolo feature and add it to the Yolo object. Update the block totals
        public void ObjectClaimsNewFeature(ProcessBlock block, YoloObject theObject, YoloFeature newFeature)
        {
            try
            {
                block.AddFeature(newFeature);
                theObject.ClaimFeature(newFeature);
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloProcessModel.ObjectClaimsNewFeature", ex);
            }
        }


        public int ProcessBlock(
            ProcessScope scope,
            Image<Gray, byte> prevGray,
            Image<Gray, byte> currGray,
            DetectionResult? result)
        {
            try
            {
                var numSig = 0;

                if(result != null) 
                {
                    numSig = result.Boxes.Count();

                    var thisBlock = YoloBlocks.LastBlock;

                    // Convert Boxes to YoloObjects
                    foreach (var box in result.Boxes)
                    {
                        // We have found a new feature/object
                        var imagePixelBox = new Rectangle(box.Bounds.Left, box.Bounds.Top, box.Bounds.Width, box.Bounds.Height);
                        var newFeature = this.YoloFeatures.AddFeature(thisBlock.BlockId, imagePixelBox);
                        var newObject = this.YoloObjects.AddObject(scope, newFeature,
                            box.Class.Name, System.Drawing.Color.Red, box.Confidence);
                        this.ObjectClaimsNewFeature(thisBlock, newObject, newFeature);

                        /*
                        newObject.Size = new(box.Bounds.Width, box.Bounds.Height);
                        newObject.Confidence = box.Confidence;
                        //newObject.Class = box.Class.Name;
                        newObject.ClassId = box.Class.Id;
                        //newObject.ClassColor = box.Class.Color;
                        newObject.ClassType = box.Class.Type;
                        newObject.ClassDescription = box.Class.Description;
                        */
                    }
                }


                return numSig;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloProcessModel.ProcessBlock" +
                    "(CurrBlockId=" + scope.PSM.CurrBlockId +
                    ",LastBlockId=" + scope.PSM.LastBlockId + ")", ex);
            }
        }


        public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "# Blocks", YoloBlocks.Count },
                { "# Features", YoloFeatures.Count},
                { "# Objects", YoloObjects.Count},
            };
        }

    };

}
