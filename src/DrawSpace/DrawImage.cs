// Copyright SkyComb Limited 2025. All rights reserved.
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
    /// Written by ChatGPT for NQ 25/10/25
    public static class ImageCropper
    {
        // ---- BAKED-IN CAMERA PARAMETERS ----
        // Thermal: DJI M4T approx
        private const double ThermalFovDeg = 45.0;           // Horizontal DFOV (thermal)
        private const int ThermalWidthPx = 1280;          // Thermal native width
        private const int ThermalHeightPx = 1024;          // Thermal native height
        private static double ThermalAspect => (double)ThermalWidthPx / ThermalHeightPx; // 1.25 (5:4)

        // Optical wide FOV at "zoom = 1×"
        private const double OpticalWideFovDeg = 82.0;       // Horizontal DFOV for the wide lens at 1×

        // Simple inverse model; good up to moderate zooms and for cropping alignment.
        // If later you want a more precise mapping, replace OpticalFovFromZoom with:
        // (a) geometric model using sensor width & base focal length, or
        // (b) piecewise interpolation using a DJI zoom→FOV table.
        private static double OpticalFovFromZoom(double zoom) => OpticalWideFovDeg / zoom;

        /// <summary>
        /// Crop the optical image to match the thermal camera's ground coverage at a given zoom,
        /// and enforce the thermal aspect ratio (width/height). Center-cropped.
        /// </summary>
        /// <typeparam name="TColor">e.g., Bgr</typeparam>
        /// <typeparam name="TDepth">e.g., byte</typeparam>
        /// <param name="optical">Input optical image.</param>
        /// <param name="zoom">Optical zoom factor (e.g., 1.13, 2, 4...).</param>
        /// <param name="offsetXpx">Optional horizontal pixel offset (+right, -left) to account for boresight.</param>
        /// <param name="offsetYpx">Optional vertical pixel offset (+down, -up) to account for boresight.</param>
        /// <returns>New Image cropped to thermal FOV and aspect.</returns>
        public static Image<TColor, TDepth> CropOpticalToThermal<TColor, TDepth>(
            Image<TColor, TDepth> optical,
            double zoom,
            int offsetXpx = 0,
            int offsetYpx = 0
        )
            where TColor : struct, IColor
            where TDepth : new()
        {
            if (optical == null) throw new ArgumentNullException(nameof(optical));
            if (zoom <= 0) throw new ArgumentOutOfRangeException(nameof(zoom), "Zoom must be > 0.");

            // --- 1) Estimate optical horizontal FOV at this zoom ---
            // Simple, robust model: FOV ≈ FOV_wide / zoom
            double opticalFovDeg = OpticalFovFromZoom(zoom);

            // --- 2) FOV scale needed so optical matches thermal coverage ---
            // scale = fraction of optical width/height to keep (assuming same projection)
            double scaleFov = Math.Tan(DegToRad(ThermalFovDeg / 2.0)) / Math.Tan(DegToRad(opticalFovDeg / 2.0));
            // If thermal FOV were wider than optical (rare here), bail out with a clone.
            if (scaleFov >= 1.0)
                return optical.Clone();

            int W = optical.Width;
            int H = optical.Height;

            // Limits imposed by the FOV: cannot exceed these
            double wLimit = W * scaleFov;
            double hLimit = H * scaleFov;

            // --- 3) Enforce thermal aspect WHILE staying within the FOV limits ---
            // Two candidates:
            //   A) Height-limited: height = hLimit, width = hLimit * aspect
            //   B) Width-limited : width  = wLimit, height = wLimit / aspect
            double aspect = ThermalAspect;

            double candW_A = hLimit * aspect;
            double candH_A = hLimit;

            double candW_B = wLimit;
            double candH_B = wLimit / aspect;

            double finalW, finalH;

            // Choose the largest rectangle that fits inside BOTH FOV limits and target aspect
            if (candW_A <= wLimit)
            {
                // Height-limited fits
                finalW = candW_A;
                finalH = candH_A;
            }
            else
            {
                // Fall back to width-limited
                finalW = candW_B;
                finalH = candH_B;
            }

            int newW = Math.Max(1, (int)Math.Round(finalW));
            int newH = Math.Max(1, (int)Math.Round(finalH));

            // --- 4) Center crop (with optional boresight offsets), clamped to bounds ---
            int x = (W - newW) / 2 + offsetXpx;
            int y = (H - newH) / 2 + offsetYpx;

            // Clamp to image bounds
            x = Math.Max(0, Math.Min(x, W - newW));
            y = Math.Max(0, Math.Min(y, H - newH));

            Rectangle roi = new Rectangle(x, y, newW, newH);
            return optical.Copy(roi);
        }

        /// <summary>
        /// Crop optical Image<TColor,TDepth> to match the thermal camera's FOV.
        /// Returns a new Image with the central crop.
        /// </summary>
        /// <typeparam name="TColor">e.g., Bgr</typeparam>
        /// <typeparam name="TDepth">e.g., byte</typeparam>
        /// <param name="optical">Optical image (Emgu.CV.Image).</param>
        /// <param name="opticalFovDeg">Optical horizontal FOV in degrees at current zoom.</param>
        /// <param name="thermalFovDeg">Thermal horizontal FOV in degrees (≈45° on M4T).</param>
        public static Image<TColor, TDepth> CropToThermalFovPQ<TColor, TDepth>(
            Image<TColor, TDepth> optical,
            double opticalFovDeg,
            double thermalFovDeg = 45.0
        )
            where TColor : struct, IColor
            where TDepth : new()
        {
            // Scale = fraction of optical width/height that matches thermal coverage
            double scale = Math.Tan(DegToRad(thermalFovDeg / 2.0)) / Math.Tan(DegToRad(opticalFovDeg / 2.0));

            // If thermal FOV is wider than optical (scale >= 1), no crop needed.
            if (scale >= 1.0)
                return optical.Clone();

            int newW = Math.Max(1, (int)Math.Round(optical.Width * scale));
            int newH = Math.Max(1, (int)Math.Round(optical.Height * scale));

            int x = Math.Max(0, (optical.Width - newW) / 2);
            int y = Math.Max(0, (optical.Height - newH) / 2);

            // Ensure ROI stays inside bounds (paranoia)
            if (x + newW > optical.Width) newW = optical.Width - x;
            if (y + newH > optical.Height) newH = optical.Height - y;

            Rectangle roi = new Rectangle(x, y, newW, newH);

            // Image<TColor,TDepth>.Copy(Rectangle) returns a new cropped Image
            return optical.Copy(roi);
        }

        private static double DegToRad(double deg) => deg * Math.PI / 180.0;
    }
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
        public static void ApplyThresholdVisualization(ProcessConfigModel config, ref Image<Bgr, byte> imgInput)
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
        public static Mat ResizeImage(Image<Bgr, byte> theImage, double factor, Inter inter = Inter.Linear)
        {
            var frameResized = new Mat();
            CvInvoke.Resize(theImage, frameResized, new Size(0, 0), factor, factor, inter);

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