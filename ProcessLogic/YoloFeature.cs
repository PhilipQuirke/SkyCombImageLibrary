// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using YoloDotNet.Models;


namespace SkyCombImage.ProcessLogic
{
    // A Yolo feature 
    public class YoloFeature : ProcessFeature
    {
        // Results of YoloDetect image processing
        public LabelModel? Label { get; init; } = null;
        public double Confidence { get; init; } = 0f;


        public YoloFeature(YoloProcess yoloProcess, int blockId, ObjectDetection result) : base(yoloProcess, blockId, FeatureTypeEnum.Real)
        {
            ResetCalcedMemberData();

            PixelBox = new System.Drawing.Rectangle(result.BoundingBox.Left, result.BoundingBox.Top, result.BoundingBox.Width, result.BoundingBox.Height);
            Significant = true;
            Label = result.Label;
            Confidence = result.Confidence;
        }


        // Constructor used when loaded objects from the datastore
        public YoloFeature(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, settings)
        {
            Label = null;
            Confidence = 0f;
        }


        // Evaluate the MinHeat, MaxHeat and PixelSize of this feature
        public void CalculateHeat(
           in Image<Bgr, byte> imgOriginal,
           in Image<Gray, byte> imgThreshold)
        {
            MinHeat = 255 * 3;
            MaxHeat = 0;
            NumHotPixels = 0;

            int left = Math.Max(PixelBox.Left, 0);
            int top = Math.Max(PixelBox.Top, 0);
            int right = Math.Min(PixelBox.Right, imgOriginal.Width);
            int bottom = Math.Min(PixelBox.Bottom, imgOriginal.Height);

            for (int y = top; y < bottom; y++)
            {
                for (int x = left; x < right; x++)
                {
                    if (imgThreshold[y, x].Intensity > 0)
                    {
                        Bgr color = imgOriginal[y, x];
                        int currHeat = (int)((color.Blue + color.Green + color.Red) / 3);
                        MinHeat = Math.Min(MinHeat, currHeat);
                        MaxHeat = Math.Max(MaxHeat, currHeat);
                        NumHotPixels++;
                    }
                }
            }

            Significant = (NumHotPixels >= ProcessAll.ProcessConfig.FeatureMinPixels);
            IsTracked = Significant;
        }
    };
}
