// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;
using Compunet.YoloV8.Data;



namespace SkyCombImage.ProcessLogic
{
    // A Yolo feature 
    public class YoloFeature : ProcessFeature
    {
        // Results of YoloDetect image processing
        public BoundingBox? BoundingBox { get; set; } = null;


        public YoloFeature(YoloProcess yoloProcess, int blockId, Rectangle imagePixelBox, BoundingBox? boundingBox) : base(yoloProcess, blockId, FeatureTypeEnum.Real)
        {
            ResetMemberData();
            PixelBox = imagePixelBox;
            Significant = true;
            BoundingBox = boundingBox;
        }
    };

}
