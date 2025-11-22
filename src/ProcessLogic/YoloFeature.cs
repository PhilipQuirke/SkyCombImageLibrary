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

            if (result != null)
            {
                PixelBox = new System.Drawing.Rectangle(result.BoundingBox.Left, result.BoundingBox.Top, result.BoundingBox.Width, result.BoundingBox.Height);
                Label = result.Label;
                Confidence = result.Confidence;

                // Check if this feature overlaps with exclusion zones
                var processConfig = yoloProcess.ProcessConfig;
                if (processConfig.ExcludeBottomRightCorner)
                {
                    // Get image dimensions from the process (we'll need to get this from the block or video data)
                    var block = yoloProcess.Blocks[blockId];
                    var imageWidth = yoloProcess.VideoData.ImageWidth;
                    var imageHeight = yoloProcess.VideoData.ImageHeight;

                    var (rightBoundary, bottomBoundary) = 
                        processConfig.GetExclusionBoundaries(imageWidth, imageHeight);

                    // Check if feature overlaps with excluded area
                    var excludedArea = new Rectangle(rightBoundary, bottomBoundary, 
                        imageWidth - rightBoundary, imageHeight - bottomBoundary);
                    
                    if (PixelBox.IntersectsWith(excludedArea))
                    {
                        // Feature overlaps with exclusion zone - mark as insignificant
                        Significant = false;
                        IsTracked = false;
                    }
                }
            }
            else
                PixelBox = new System.Drawing.Rectangle(0, 0, 0, 0);

            // Is this feature significant?
            Calculate_Significant(ProcessAll.ProcessConfig);
        }


        // Constructor used when loaded objects from the datastore
        public YoloFeature(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, settings)
        {
            Label = null;
            Confidence = 0f;
        }


        // Override pixel regeneration for YOLO features to use the existing CalculateHeat_ShrinkBox method
        public override void RegeneratePixelData(in Image<Gray, byte> imgOriginal, in Image<Gray, byte> imgThreshold)
        {
            if (Pixels != null)
                return; // Already have pixel data

            // For YOLO features, use the existing CalculateHeat_ShrinkBox method which includes
            // the logic for shrinking the bounding box to fit tightly around hot pixels
            CalculateHeat_ShrinkBox(imgOriginal, imgThreshold);
        }

        // Shrink the result.BoundingBox to a smaller bounding box tight around the hot pixels.
        // Evaluate the MinHeat, MaxHeat, NumHotPixels, SumHotPixels and PixelBox of this feature
        public void CalculateHeat_ShrinkBox(
            in Image<Gray, byte> imgOriginal,
            in Image<Gray, byte> imgThreshold)
        {
            ClearHotPixelData();
            Pixels = new();

            var processConfig = ProcessAll.ProcessConfig;
            int imageWidth = imgOriginal.Width;
            int imageHeight = imgOriginal.Height;

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
                    // Check if pixel should be processed (not in exclusion zone)
                    if (!processConfig.ShouldProcessPixel(x, y, imageWidth, imageHeight))
                        continue;

                    if (imgThreshold[y, x].Intensity > 0)
                    {
                        // Update the tight bounding box coordinates
                        if (x < minX) minX = x;
                        if (x > maxX) maxX = x;
                        if (y < minY) minY = y;
                        if (y > maxY) maxY = y;

                        // Evaluate the heat from the original image (not the threshold image)
                        var orgColor = imgOriginal[y, x];
                        AddHotPixel(y, x, orgColor);
                    }
                }
            }

            if (NumHotPixels > 0)
                // Set (shrink) PixelBox to the tight bounding box around hot pixels
                PixelBox = new Rectangle(minX, minY, maxX - minX + 1, maxY - minY + 1);

            Calculate_HotPixelData();
            Calculate_Significant(ProcessAll.ProcessConfig);
            IsTracked = Significant;
        }
    };
}
