// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;


namespace SkyCombImage.DrawSpace
{
    // Static class to perform image manipulation
    public class DrawImage
    {
        // Grayscale
        // Convert color to gray image
        public static Image<Gray, byte> ToGrayScale(Image<Bgr, byte> imgInput)
        {
            return imgInput.Convert<Gray, byte>();
        }


        // Threshold
        // Can generate new pixel colors not in original image.
        public static void Threshold(ProcessConfigModel config, ref Image<Gray, byte> imgInput)
        {
            imgInput = imgInput.ThresholdBinary(new Gray(config.HeatThresholdValue), new Gray(255));
        }


        // Process a single image and returns an image.
        public static void Draw(
            RunProcessEnum runProcess, ProcessConfigModel config, DrawImageConfig drawConfig,
            ref Image<Bgr, byte> imgInput)
        {
            if (runProcess == RunProcessEnum.Threshold)
            {
                // For Threshold processing, we want to show the original thermal image 
                // with hot pixels highlighted in thermal colors (orange/red)
                // This should NOT show bounding rectangles - those are handled by DrawRunProcess in ProcessDrawImage
                ApplyThresholdVisualization(config, ref imgInput);
            }
        }

        
        // Apply threshold visualization while preserving the original thermal image
        private static void ApplyThresholdVisualization(ProcessConfigModel config, ref Image<Bgr, byte> imgInput)
        {
            // Convert to grayscale for threshold analysis
            var grayImage = ToGrayScale(imgInput);
            
            // Apply threshold to identify hot pixels
            var thresholdImage = grayImage.ThresholdBinary(new Gray(config.HeatThresholdValue), new Gray(255));
            
            int imageWidth = imgInput.Width;
            int imageHeight = imgInput.Height;

            // Color the hot pixels with thermal colors (8 colors from yellow to red)
            int numColors = 8;
            var theShades = GetColorShades(
                Color.FromArgb(255, 255, 255, 0),  // Yellow
                Color.FromArgb(255, 255, 0, 0),    // Red
                numColors);

            // Use a linear threshold from the config.HeatThresholdValue to 255
            int thresholdStep = Math.Max(1, (255 - config.HeatThresholdValue) / numColors);
            int[] thresholds = new int[numColors];
            for (int i = 0; i < numColors; i++)
                thresholds[i] = config.HeatThresholdValue + i * thresholdStep;

            // Apply thermal coloring to hot pixels
            for (int y = 0; y < imageHeight; y++)
            {
                for (int x = 0; x < imageWidth; x++)
                {
                    // Check if pixel should be processed (not in exclusion zone)
                    if (!config.ShouldProcessPixel(x, y, imageWidth, imageHeight))
                        continue; // Skip pixels in exclusion zone

                    // If this pixel is above threshold (hot)
                    if (thresholdImage.Data[y, x, 0] > 0)
                    {
                        // Get the original pixel heat value
                        byte originalHeat = grayImage.Data[y, x, 0];
                        
                        // Determine which thermal color to use based on heat intensity
                        int colorIndex = 0;
                        for (int i = 0; i < numColors; i++)
                        {
                            if (originalHeat >= thresholds[i])
                                colorIndex = i;
                        }

                        // Apply the thermal color to this hot pixel
                        var thermalColor = theShades[colorIndex];
                        imgInput.Data[y, x, 0] = thermalColor.B; // Blue
                        imgInput.Data[y, x, 1] = thermalColor.G; // Green
                        imgInput.Data[y, x, 2] = thermalColor.R; // Red
                    }
                    // If not a hot pixel, leave the original thermal image unchanged
                }
            }

            // Clean up temporary images
            grayImage.Dispose();
            thresholdImage.Dispose();
        }

        
        // Helper method to get color shades from one color to another
        private static List<Color> GetColorShades(Color startColor, Color endColor, int numShades)
        {
            var colors = new List<Color>();
            
            if (numShades <= 1)
            {
                colors.Add(startColor);
                return colors;
            }

            for (int i = 0; i < numShades; i++)
            {
                float ratio = (float)i / (numShades - 1);
                
                int r = (int)(startColor.R + ratio * (endColor.R - startColor.R));
                int g = (int)(startColor.G + ratio * (endColor.G - startColor.G));
                int b = (int)(startColor.B + ratio * (endColor.B - startColor.B));
                
                colors.Add(Color.FromArgb(255, r, g, b));
            }
            
            return colors;
        }


        // We need to resize the theImage by a factor
        public static Mat ResizeImage(Image<Bgr, byte> theImage, double factor)
        {
            var frameResized = new Mat();
            CvInvoke.Resize(theImage, frameResized, new Size(0, 0), factor, factor, Inter.Linear);

            return frameResized;
        }


        // Store an image in a PictureBox
        public static void StoreImageInPicture(Image<Bgr, byte> theImage, PictureBox thePicture)
        {
            // Early return if either parameter is null
            if (thePicture == null || theImage == null)
            {
                if (thePicture != null && thePicture.Image != null)
                {
                    thePicture.Image.Dispose(); // Clean up existing image
                    thePicture.Image = null;
                }
                return;
            }

            try
            {
                // Cache dimensions to avoid multiple property accesses
                double oldWidth = theImage.Width;
                double oldHeight = theImage.Height;
                int boxWidth = thePicture.Width;
                int boxHeight = thePicture.Height;

                // Avoid division by zero
                if (oldWidth <= 0 || oldHeight <= 0 || boxWidth <= 0 || boxHeight <= 0)
                {
                    return;
                }

                // Calculate resize factor
                var widthRatio = boxWidth / oldWidth;
                var heightRatio = boxHeight / oldHeight;
                var factor = Math.Min(widthRatio, heightRatio);

                // Dispose of existing image before assigning new one
                if (thePicture.Image != null)
                {
                    thePicture.Image.Dispose();
                }

                // Only resize if necessary
                if (Math.Abs(factor - 1.0) < 0.001)
                {
                    thePicture.Image = theImage.ToBitmap();
                }
                else
                {
                    // We need to resize the image to match PictureBox dimensions
                    using (var resizedImage = DrawImage.ResizeImage(theImage, factor))
                    {
                        thePicture.Image = resizedImage.ToBitmap();
                    }
                }
            }
            catch (Exception ex)
            {
                // Clean up on error
                if (thePicture.Image != null)
                {
                    thePicture.Image.Dispose();
                    thePicture.Image = null;
                }
                // Optionally rethrow or handle the exception as needed
                throw new Exception("Error processing image", ex);
            }
        }


        // Store a bitmap in a PictureBox with proper resizing
        public static void StoreBitmapInPicture(Bitmap? theImage, PictureBox? thePicture)
        {
            if (thePicture == null)
                return;

            if (theImage != null)
            {
                double oldWidth = theImage.Width;
                double oldHeight = theImage.Height;

                // if it fits, don't muck around with it - to stop fuzziness
                if ((oldHeight > thePicture.Height) && (oldWidth > thePicture.Width))
                {
                    thePicture.Image?.Dispose();
                    thePicture.Image = theImage;
                    return;
                }
                // Calculate resize factors
                var widthRatio = thePicture.Width / oldWidth;
                var heightRatio = thePicture.Height / oldHeight;
                var factor = Math.Min(widthRatio, heightRatio);

                // Calculate new dimensions
                int newWidth = (int)(oldWidth * factor);
                int newHeight = (int)(oldHeight * factor);

                // Create resized bitmap
                var resized = new Bitmap(newWidth, newHeight);

                using (Graphics g = Graphics.FromImage(resized))
                {
                    // Configure graphics for high quality resizing
                    g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                    g.SmoothingMode = SmoothingMode.HighQuality;
                    g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                    g.CompositingQuality = CompositingQuality.HighQuality;

                    // Draw the resized image
                    g.DrawImage(theImage, 0, 0, newWidth, newHeight);
                }

                // Dispose of old image if PictureBox already had one
                thePicture.Image?.Dispose();

                // Set the new image
                thePicture.Image = resized;
            }
            else
            {
                // Clear and dispose of any existing image
                thePicture.Image?.Dispose();
                thePicture.Image = null;
            }
        }
    }
}