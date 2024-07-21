// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using SkyCombDrone.DrawSpace;
using SkyCombGround.CommonSpace;
using System.Drawing;
using SkyCombImageLibrary.RunSpace;


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


        // Smooth
        // Refer https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html for explanation of methods
        // Can generate new pixel colors not in original image.
        public static void Smooth(ProcessConfigModel config, ref Image<Bgr, byte> imgInput)
        {
            switch(config.SmoothProcess)
            {
                case SmoothProcessEnum.Blur: 
                    imgInput = imgInput.SmoothBlur(config.SmoothPixels, config.SmoothPixels); 
                    break;
                case SmoothProcessEnum.Gaussian: 
                    imgInput = imgInput.SmoothGaussian(config.SmoothPixels); 
                    break;
                case SmoothProcessEnum.Median: 
                    imgInput = imgInput.SmoothMedian(config.SmoothPixels); 
                    break;
            }
        }


        // Threshold
        // Can generate new pixel colors not in original image.
        public static void Threshold(ProcessConfigModel config, ref Image<Gray, byte> imgInput)
        {
            switch( config.ThresholdProcess )
            {
                case ThresholdProcessEnum.Binary: 
                    imgInput = imgInput.ThresholdBinary(new Gray(config.HeatThresholdValue), new Gray(255)); 
                    break;
                case ThresholdProcessEnum.BinaryInv: 
                    imgInput = imgInput.ThresholdBinaryInv(new Gray(config.HeatThresholdValue), new Gray(255)); 
                    break;
                case ThresholdProcessEnum.ToZero: 
                    imgInput = imgInput.ThresholdToZero(new Gray(config.HeatThresholdValue)); 
                    break;
                case ThresholdProcessEnum.ToZeroInv: 
                    imgInput = imgInput.ThresholdToZeroInv(new Gray(config.HeatThresholdValue)); 
                    break;
                case ThresholdProcessEnum.Trunc: 
                    imgInput = imgInput.ThresholdTrunc(new Gray(config.HeatThresholdValue)); 
                    break;
            }
        }


        // Apply background color palette (if any) to imgInput to give imgOutput
        public static void Palette(DrawImageConfig config, ref Image<Bgr, byte> imgInput)
        {
            if (config.Palette == "none")
                return;

            Image<Bgr, byte> outputImg = imgInput.Clone();
            CvInvoke.ApplyColorMap(imgInput, outputImg, config.DrawPaletteToEnum());
            imgInput = outputImg;
        }


        private static string DistanceTransformNotes(Mat distanceTransform)
        {
            distanceTransform.MinMax(out double[] minValues, out double[] maxValues, out Point[] minLocations, out Point[] maxLocations);

            return string.Format(
                "distanceTransform Cols={0}, Rows={1}, minValue={2}, maxValue={3}",
                distanceTransform.Cols, distanceTransform.Rows,
                minValues[0], maxValues[0]);
        }


        // Return altered input image using a OpenCV Threshold feature.
        public static Image<Gray, byte> DrawThreshold(ProcessConfigModel config, ref Image<Bgr, byte> imgInput)
        {
            Smooth(config, ref imgInput);

            var answer = ToGrayScale(imgInput);

            Threshold(config, ref answer);  

            return answer;
        }


        // Detect GFTT points
        public static MKeyPoint[] Detect_GFTT(ProcessConfigModel config, Image<Gray, byte> imgInput, int maxCorners = -1)
        {
            var detector = new GFTTDetector(
                maxCorners > 0 ? maxCorners : config.GfttMaxCorners,
                config.GfttQualityLevel,
                config.GfttMinDistance,
                config.GfttBlockSize,
                config.GfttUseHarris,
                config.GfttK);

            return detector.Detect(imgInput);
        }


        // Analyse input image using OpenCV "Good Features To Track" (aka GFTT) feature
        // No smoothing or thresholding on input. Just grayscale input before detecting corners.  
        public static void DrawGftt(
            ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            ref Image<Bgr, byte> imgInput)
        {
            Image<Gray, byte> imgInputGray = ToGrayScale(imgInput);

            // Detect GFTT points on a grayscale image
            var keypoints = Detect_GFTT(processConfig, imgInputGray);

            var imgOutColor = imgInput.Clone();
            foreach (MKeyPoint p in keypoints)
            {
                SkyCombDrone.DrawSpace.Draw.Circle(drawConfig, ref imgOutColor, Point.Round(p.Point));

                // Debugging - from original code.
                if (float.IsNaN(p.Point.X) || float.IsNaN(p.Point.Y))
                    throw BaseConstants.ThrowException("Process.Image.Process_GFTT: IsNaN");
            }

            imgInput = imgOutColor;
        }


        // We need to resize the theImage by a factor
        public static Mat ResizeImage(Image<Bgr, byte> theImage, double factor)
        {
            var frameResized = new Mat();
            CvInvoke.Resize(theImage, frameResized, new Size(0, 0), factor, factor, Inter.Linear);

            return frameResized;
        }


        // Process (analyse) a single image (using any one ProcessName) and returns an image.
        // The output image shows hot pixel in green, with red rectangles bounding significant features.
        public static void Draw(
            RunProcessEnum runProcess, ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            ref Image<Bgr, byte> imgInput)
        {
            switch (runProcess)
            {
                case RunProcessEnum.GFTT: 
                    DrawGftt(processConfig, drawConfig, ref imgInput);
                    break;

                case RunProcessEnum.Threshold: 
                    DrawThreshold(processConfig, ref imgInput);
                    break;
            }
        }
    }
}