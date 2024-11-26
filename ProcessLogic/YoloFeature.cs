// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using System.Drawing;
using YoloDotNet.Models;


namespace SkyCombImage.ProcessLogic
{
    // A Yolo feature 
    public class YoloFeature : ProcessFeature
    {
        // Results of YoloDetect image processing
        public LabelModel? Label { get; init; } = null;
        public double Confidence { get; init; } = 0f;


        public YoloFeature(YoloProcess yoloProcess, int blockId, ObjectDetection? result, FeatureTypeEnum type = FeatureTypeEnum.Real) : base(yoloProcess, blockId, type)
        {
            ResetCalcedMemberData();
            Significant = true;

            if (result != null)
            {
                PixelBox = new System.Drawing.Rectangle(result.BoundingBox.Left, result.BoundingBox.Top, result.BoundingBox.Width, result.BoundingBox.Height);
                Label = result.Label;
                Confidence = result.Confidence;
            }
            else
                PixelBox = new System.Drawing.Rectangle(0, 0, 0, 0);

        }


        // Constructor used when loaded objects from the datastore
        public YoloFeature(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, settings)
        {
            Label = null;
            Confidence = 0f;
        }


        // Shrink the result.BoundingBox to a smaller bounding box tight around the hot pixels.
        // Evaluate the MinHeat, MaxHeat, NumHotPixels and PixelBox of this feature
        public void CalculateHeat_ShrinkBox(
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

            // Initialize variables to find the tight bounding box
            int minX = right;
            int maxX = left;
            int minY = bottom;
            int maxY = top;

            for (int y = top; y < bottom; y++)
            {
                for (int x = left; x < right; x++)
                {
                    if (imgThreshold[y, x].Intensity > 0)
                    {
                        // Update the tight bounding box coordinates
                        if (x < minX) minX = x;
                        if (x > maxX) maxX = x;
                        if (y < minY) minY = y;
                        if (y > maxY) maxY = y;

                        // Evaluate the heat from the original image (not the threshold image)
                        Bgr orgColor = imgOriginal[y, x];
                        AddHotPixel(y, x, orgColor);
                    }
                }
            }

            if (NumHotPixels > 0)
                // Set (shrink) PixelBox to the tight bounding box around hot pixels
                PixelBox = new Rectangle(minX, minY, maxX - minX + 1, maxY - minY + 1);

            Significant = (NumHotPixels >= ProcessConfigModel.FeatureMinPixels);
            IsTracked = Significant;
        }
    };
}
