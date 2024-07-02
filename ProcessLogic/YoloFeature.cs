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


    public class YoloFeatureList : ProcessFeatureList
    {
        public YoloFeatureList(ProcessConfigModel config) : base(config)
        {
        }


        protected override ProcessFeatureList Create(ProcessConfigModel config)
        {
            return new YoloFeatureList(config);
        }

        public YoloFeature AddFeature(YoloProcess yoloProcess, int blockId, Rectangle imagePixelBox, BoundingBox? boundingBox)
        {
            var feature = new YoloFeature(yoloProcess, blockId, imagePixelBox, boundingBox);
            AddFeature(feature);
            return feature;
        }
    };

}
