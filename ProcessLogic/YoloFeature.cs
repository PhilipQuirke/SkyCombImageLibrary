// Copyright SkyComb Limited 2024. All rights reserved. 
using Compunet.YoloV8.Data;
using Compunet.YoloV8.Metadata;
using Emgu.CV.Structure;
using Emgu.CV;
using SkyCombImage.ProcessModel;


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


        // Evaluate the MinHeat, MaxHeat and PixelSize of this feature
        public void CalculateHeat(
            Image<Bgr, byte> imgOriginal,
            Image<Gray, byte> imgThreshold)
        {
            MinHeat = 255 + 255 + 255;
            MaxHeat = 0;
            NumHotPixels = 0;

            // Test each pixel in the bounding box. If the image pixel exceeds the threshold, update the MaxHeat, MinHeat and pixelCount
            for (int y = PixelBox.Top; y < PixelBox.Bottom; y++)
                for (int x = PixelBox.Left; x < PixelBox.Right; x++)
                    {
                        if (imgThreshold.Data[y, x, 0] > 0)
                        {
                            // Evaluate the heat from the original image (not the smooth / threshold image)
                            int currHeat = (
                                imgOriginal.Data[y, x, 0] +
                                imgOriginal.Data[y, x, 1] +
                                imgOriginal.Data[y, x, 2]) / 3;

                            if (currHeat < MinHeat)
                                MinHeat = currHeat;
                            if (currHeat > MaxHeat)
                                MaxHeat = currHeat;
                            NumHotPixels++;
                        }
                    }

            // Is this feature significant?
            Significant = (NumHotPixels >= ProcessAll.ProcessConfig.FeatureMinPixels);
            IsTracked = Significant;
        }
    };
}
