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


        // Return altered input image using a OpenCV Threshold feature.
        public static Image<Gray, byte> DrawThreshold(ProcessConfigModel config, ref Image<Bgr, byte> imgInput)
        {
            var answer = ToGrayScale(imgInput);

            Threshold(config, ref answer);

            return answer;
        }


        // Process a single image and returns an image.
        public static void Draw(
            RunProcessEnum runProcess, ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            ref Image<Bgr, byte> imgInput)
        {
            if (runProcess == RunProcessEnum.Threshold)
                DrawThreshold(processConfig, ref imgInput);
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
        public static void StoreBitmapInPicture(Bitmap theImage, PictureBox thePicture)
        {
            if (thePicture == null) return;

            if (theImage != null)
            {
                double oldWidth = theImage.Width;
                double oldHeight = theImage.Height;

                // Calculate resize factors
                var widthRatio = (double)thePicture.Width / oldWidth;
                var heightRatio = (double)thePicture.Height / oldHeight;
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
                if (thePicture.Image != null)
                {
                    thePicture.Image.Dispose();
                }

                // Set the new image
                thePicture.Image = resized;
            }
            else
            {
                // Clear and dispose of any existing image
                if (thePicture.Image != null)
                {
                    thePicture.Image.Dispose();
                    thePicture.Image = null;
                }
            }
        }
    }
}