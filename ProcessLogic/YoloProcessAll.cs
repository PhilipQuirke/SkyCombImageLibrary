// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using System.Drawing;


namespace SkyCombImage.ProcessModel
{


    // A Yolo feature 
    public class YoloFeature : ProcessFeatureModel
    {
        public YoloFeature(int blockId, Point location) : base(blockId, CombFeatureTypeEnum.Real)
        {
            ResetMemberData();
            LocationM = new DroneLocation(location.Y,location.X);
            Significant = true;
        }
    };


    public class YoloFeatureList : List<YoloFeature>
    {
        public static ProcessConfigModel Config;

        public YoloFeature AddFeature(int blockId, Point location)
        {
            var answer = new YoloFeature(blockId, location);
            this.Add(answer);
            return answer;
        }
    };


    // A class to hold a Yolo object - layer over a sequence of Yolo features.
    public class YoloObject : ProcessObject
    {

        // First feature claimed by this object
        public YoloFeature FirstFeature { get; set; }
        // Last feature claimed by this object
        public YoloFeature LastFeature { get; set; }


        public YoloObject(ProcessScope scope, YoloFeature firstFeature) : base(scope)
        {
            FirstFeature = firstFeature;
            LastFeature = firstFeature;
            Significant = true;
        }


        // Does the last feature occupy the same block & location as the params?
        public bool LastFeatureIntersects(int blockId, int theY, int theX)
        {
            if (LastFeature.BlockId < blockId)
                return false;

            // Euclidian distance test without use of slow square root function.
            var lastEntry = LastFeature.LocationM;
            return
                Math.Pow(lastEntry.EastingM - theX, 2) +
                Math.Pow(lastEntry.NorthingM - theY, 2) <
                Config.GfttMinDistance * Config.GfttMinDistance;
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
        public static ProcessConfigModel Config;


        // We do not process objects below this index in the object array.
        public int LegFirstIndex;


        public YoloObject AddObject(ProcessScope scope, YoloFeature firstFeature)
        {
            var answer = new YoloObject(scope, firstFeature);
            Add(answer);
            return answer;
        }


        // Return the list of features that are significant
        public int SignificantCount()
        {
            int answer = 0;

            foreach (var theObject in this)
                if (theObject.Significant)
                    answer++;

            return answer;
        }


        // No existing objects should be live at the start of a new leg
        public void ProcessLegStart()
        {
            foreach (var theObject in this)
                theObject.Significant = false;

            LegFirstIndex = Count;
        }


        // Return the list of features that are currently significant
        public YoloObjectList SignificantList()
        {
            YoloObjectList answer = new();

            for (int index = LegFirstIndex; index < Count; index++)
            {
                var theObject = this[index];
                if (theObject.Significant)
                    answer.Add(theObject);
            }

            return answer;
        }
    };



    // A class to hold all Yolo feature and block data associated with a video
    public class YoloProcessAll : ProcessAll
    {
        public ProcessBlockList YoloBlocks;
        public YoloObjectList YoloObjects;
        public YoloFeatureList YoloFeatures;

        // YOLO (You only look once) V8 image processing
        public YoloV8 YoloModel;


        public YoloProcessAll(ProcessConfigModel config, Drone drone, string modelDirectory) : base(config, drone.InputVideo, drone)
        {
            YoloObject.Config = config;
            YoloObjectList.Config = config;
            YoloFeatureList.Config = config;

            YoloBlocks = new();
            YoloObjects = new();
            YoloFeatures = new();

            YoloObjects.LegFirstIndex = 0;

            YoloModel = new YoloV8(modelDirectory);
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
            Image<Gray, byte> PrevGray,
            Image<Gray, byte> CurrGray)
        {
            try
            {
                var numSignificantObjects = 0;
  
                // PQR TODO


                return numSignificantObjects;
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
