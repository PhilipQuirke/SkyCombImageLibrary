// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessModel;
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
            ResetCalcedMemberData();
            PixelBox = imagePixelBox;
            Significant = true;
            BoundingBox = boundingBox;
        }


        // Constructor used when loaded objects from the datastore
        public YoloFeature(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, settings)
        {
        }
    };

}
