// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessModel;
using System.Drawing;
using Compunet.YoloV8.Data;
using Compunet.YoloV8.Metadata;


namespace SkyCombImage.ProcessLogic
{
    // A Yolo feature 
    public class YoloFeature : ProcessFeature
    {
        // Results of YoloDetect image processing
        public YoloV8Class? Class { get; init; } = null;
        public float Confidence { get; init; } = 0f;


        public YoloFeature(YoloProcess yoloProcess, int blockId, BoundingBox? box) : base(yoloProcess, blockId, FeatureTypeEnum.Real)
        {
            ResetCalcedMemberData();

            PixelBox = new System.Drawing.Rectangle(box.Bounds.Left, box.Bounds.Top, box.Bounds.Width, box.Bounds.Height);
            Significant = true;
            Class = box.Class;
            Confidence = box.Confidence;
        }


        // Constructor used when loaded objects from the datastore
        public YoloFeature(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, settings)
        {
            Class = null;
            Confidence = 0f;
        }
    };

}
