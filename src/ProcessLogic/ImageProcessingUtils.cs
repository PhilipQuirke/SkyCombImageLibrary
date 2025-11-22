// Copyright SkyComb Limited 2025. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using System.Drawing;

namespace SkyCombImage.ProcessLogic
{
    /// <summary>
    /// Utility class for common image processing operations used across different processing methods
    /// </summary>
    public static class ImageProcessingUtils
    {
        /// <summary>
        /// Creates a threshold image from the original image using the standard processing pipeline:
        /// 1. Convert to grayscale
        /// 2. Apply Gaussian blur for smoothing
        /// 3. Apply binary threshold
        /// </summary>
        /// <param name="originalImage">The original color image</param>
        /// <param name="processConfig">Configuration containing threshold value</param>
        /// <returns>A binary threshold image where hot pixels are white (255) and others are black (0)</returns>
        public static Image<Gray, byte> CreateThresholdImage(in Image<Gray, byte> originalImage, ProcessConfigModel processConfig)
        {
            // Create a threshold image from the current frame using the same process as during detection
            using var smoothedImage = new Image<Gray, byte>(originalImage.Size);
            
            // Apply the same smoothing as in the original processing
            CvInvoke.GaussianBlur(originalImage, smoothedImage, new Size(3, 3), 0);
            
            // Apply threshold
            var thresholdImage = new Image<Gray, byte>(smoothedImage.Size);
            int thresholdValue = processConfig.HeatThresholdValue;
            CvInvoke.Threshold(smoothedImage, thresholdImage, thresholdValue, 255, ThresholdType.Binary);
            
            return thresholdImage;
        }

        /// <summary>
        /// Regenerate pixel data for features in a given block if their pixel data was cleared for memory management.
        /// This is the main entry point for pixel regeneration when displaying objects in the UI.
        /// </summary>
        /// <param name="block">The block containing features to regenerate</param>
        /// <param name="originalImage">The original color image</param>
        /// <param name="processAll">The process data containing features</param>
        /// <param name="processConfig">Configuration containing threshold value</param>
        public static void RegeneratePixelDataForBlock(ProcessBlock block, Image<Gray, byte> originalImage, ProcessAll processAll, ProcessConfigModel processConfig)
        {
            if (block == null || originalImage == null)
                return;

            // Create threshold image using the standard processing pipeline
            using var thresholdImage = CreateThresholdImage(originalImage, processConfig);

            // Check all features in this block and regenerate pixel data if needed
            for (int featureId = block.MinFeatureId; featureId <= block.MaxFeatureId; featureId++)
            {
                if (processAll.ProcessFeatures.ContainsKey(featureId))
                {
                    var feature = processAll.ProcessFeatures[featureId];
                    if (feature.BlockId == block.BlockId && feature.Pixels == null)
                    {
                        // Regenerate pixel data for this feature using the feature's own method
                        // This will handle the specific logic for different feature types (Comb, Yolo, etc.)
                        feature.RegeneratePixelData(originalImage, thresholdImage);
                    }
                }
            }
        }

        /// <summary>
        /// Common pixel processing logic used by threshold-based methods (Comb, Threshold).
        /// This extracts the common pattern found in multiple places.
        /// </summary>
        /// <param name="feature">The feature to regenerate pixels for</param>
        /// <param name="imgOriginal">The original color image</param>
        /// <param name="imgThreshold">The binary threshold image</param>
        /// <param name="processConfig">Configuration for exclusion zones</param>
        public static void RegeneratePixelsInBoundingBox(ProcessFeature feature, in Image<Gray, byte> imgOriginal, in Image<Gray, byte> imgThreshold, ProcessConfigModel processConfig)
        {
            feature.ClearHotPixelData();
            feature.Pixels = new();

            int imageWidth = imgOriginal.Width;
            int imageHeight = imgOriginal.Height;

            int left = Math.Max(feature.PixelBox.Left, 0);
            int top = Math.Max(feature.PixelBox.Top, 0);
            int right = Math.Min(feature.PixelBox.Right, imageWidth);
            int bottom = Math.Min(feature.PixelBox.Bottom, imageHeight);

            // Regenerate hot pixels within the stored PixelBox bounds
            for (int y = top; y < bottom; y++)
            {
                for (int x = left; x < right; x++)
                {
                    // Check if pixel should be processed (not in exclusion zone)
                    if (!processConfig.ShouldProcessPixel(x, y, imageWidth, imageHeight))
                        continue;

                    // Check if this pixel is hot (above threshold)
                    if (imgThreshold.Data[y, x, 0] >= processConfig.HeatThresholdValue)
                    {
                        // Add this hot pixel back to our collection
                        var orgColor = imgOriginal[y, x];
                        feature.AddHotPixel(y, x, orgColor);
                    }
                }
            }

            feature.Calculate_HotPixelData();
        }
    }
}