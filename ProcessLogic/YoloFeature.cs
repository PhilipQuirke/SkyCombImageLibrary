// Copyright SkyComb Limited 2024. All rights reserved. 
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
    public class YoloFeature : ProcessFeature
    {
        // Results of YoloDewtect image processing
        public BoundingBox? BoundingBox { get; set; } = null;


        public YoloFeature(YoloProcess yoloProcess, int blockId, Rectangle imagePixelBox, BoundingBox? boundingBox) : base(yoloProcess, blockId, FeatureTypeEnum.Real)
        {
            ResetMemberData();
            PixelBox = imagePixelBox;
            Significant = true;
            BoundingBox = boundingBox;
        }


        // Does this Feature's PixleBox and the specified object's rectangle overlap significantly?
        public bool SignificantPixelBoxIntersection(Rectangle objectExpectedPixelBox)
        {
            return base.SignificantPixelBoxIntersection(objectExpectedPixelBox, ProcessAll.ProcessConfig.FeatureMinOverlapPerc);
        }
    };


    public class YoloFeatureList : SortedList<int, YoloFeature>
    {
        private YoloProcess YoloProcess;


        public YoloFeatureList(YoloProcess yoloProcess)
        {
            YoloProcess = yoloProcess;
        }


        public YoloFeature AddFeature(YoloFeature feature)
        {
            BaseConstants.Assert(feature.FeatureId > 0, "AddFeature: No Id");
            this.Add(feature.FeatureId, feature);
            return feature;
        }
        public YoloFeature AddFeature(int blockId, Rectangle imagePixelBox, BoundingBox? boundingBox)
        {
            return AddFeature(new YoloFeature(YoloProcess, blockId, imagePixelBox, boundingBox));
        }


        // Add the feature list for a new block into this YoloFeatureListList 
        public void AddFeatureList(YoloFeatureList featuresToAdd)
        {
            if (featuresToAdd != null)
                foreach (var feature in featuresToAdd)
                    AddFeature(feature.Value);
        }


        public YoloFeatureList Clone()
        {
            var answer = new YoloFeatureList(YoloProcess);

            foreach (var feature in this)
                answer.AddFeature(feature.Value);

            return answer;
        }
    };

}
