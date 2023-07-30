// Copyright SkyComb Limited 2023. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using SkyCombDrone.DrawSpace;
using SkyCombGround.CommonSpace;
using System.Drawing;


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
        public static Image<Bgr, byte> Smooth(ProcessConfigModel config, Image<Bgr, byte> imgInput)
        {
            return config.SmoothProcess switch
            {
                SmoothProcessEnum.Blur => imgInput.SmoothBlur(config.SmoothPixels, config.SmoothPixels),
                SmoothProcessEnum.Gaussian => imgInput.SmoothGaussian(config.SmoothPixels),
                SmoothProcessEnum.Median => imgInput.SmoothMedian(config.SmoothPixels),
                SmoothProcessEnum.None => imgInput.Clone(),
                _ => throw BaseConstants.ThrowException("Image.Smooth: Unknown smooth process"),
            };
        }


        // Threshold
        // Can generate new pixel colors not in original image.
        public static Image<Gray, byte> Threshold(ProcessConfigModel config, Image<Gray, byte> imgInput)
        {
            return config.ThresholdProcess switch
            {
                ThresholdProcessEnum.Binary => imgInput.ThresholdBinary(new Gray(config.ThresholdValue), new Gray(255)),
                ThresholdProcessEnum.BinaryInv => imgInput.ThresholdBinaryInv(new Gray(config.ThresholdValue), new Gray(255)),
                ThresholdProcessEnum.ToZero => imgInput.ThresholdToZero(new Gray(config.ThresholdValue)),
                ThresholdProcessEnum.ToZeroInv => imgInput.ThresholdToZeroInv(new Gray(config.ThresholdValue)),
                ThresholdProcessEnum.Trunc => imgInput.ThresholdTrunc(new Gray(config.ThresholdValue)),
                ThresholdProcessEnum.None => imgInput.Clone(),
                _ => throw BaseConstants.ThrowException("Image.Threshold: Unknown threshold process"),
            };
        }


        // Apply background color palette (if any) to imgInput to give imgOutput
        public static void Palette(DrawImageConfig config, ref Image<Bgr, byte> outputImg)
        {
            if (config.Palette == "none")
                return;

            Image<Bgr, byte> temp = outputImg.Clone();
            CvInvoke.ApplyColorMap(outputImg, temp, config.DrawPaletteToEnum());
            outputImg = temp.Clone();
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
        public static (Image<Bgr, byte>, string) DrawThreshold(ProcessConfigModel config, Image<Bgr, byte> imgInput)
        {
            // Create a mask: Pixels above threshold are white. Background is black.
            Image<Gray, byte> imgThreshold = Threshold(config, ToGrayScale(Smooth(config, imgInput)));

            return (imgThreshold.Convert<Bgr, byte>(), "");
        }


        // Return altered input image using a OpenCV Trunc feature.
        public static (Image<Bgr, byte>, string) DrawTrunc(ProcessConfigModel config, Image<Bgr, byte> imgInput)
        {
            // Create a mask: Pixels above threshold are white. Background is black.
            Image<Gray, byte> imgThreshold = Threshold(config, ToGrayScale(Smooth(config, imgInput)));

            return (imgThreshold.Convert<Bgr, byte>(), "");
        }


        // Return altered input image using OpenCV "DistanceTransform" feature.
        // Doesn't work. DistanceTransform is not useful. Used params:
        //      DistanceMinGray      value="1"
        //      DistanceMaskSize     value="3"      Must be 3 or 5 or 0 (precise)
        public static (Image<Bgr, byte>, string) DrawDistance(ProcessConfigModel config, Image<Bgr, byte> imgInput)
        {
            var imgOut = imgInput.Clone();
            string notes = "";

            // Create a mask: Pixels above threshold are black. Background is white.
            var mask = Threshold(config, ToGrayScale(Smooth(config, imgInput)));

            // Calculate DistanceTransform matrix as distance from white pixel to the nearest black pixel.
            // Pixels in center of an object have a higher "distance" from white pixels.
            Mat distanceTransform = new();
            CvInvoke.DistanceTransform(mask, distanceTransform, null,
                    DistType.L2,
                    config.DistanceMaskSize); // N by N mask size 
            // Scale (640x512) matrix distances to be in 0 to 255 range (so can visualise for debugging)
            CvInvoke.Normalize(distanceTransform, distanceTransform,
                    0, 255, NormType.MinMax);
            notes = DistanceTransformNotes(distanceTransform);

            // Debugging - Show distanceTransform. Looks useless (for ThermalMaskSize = 0, 3, 5 )
            // return (distanceTransform.ToImage<Bgr, byte>(), notes);

            // Calculate Markers matrix, based on ThermalMinDistance threshold, with all pixels 0 or 255.
            var markers = distanceTransform.ToImage<Gray, byte>()
                .ThresholdBinary(new Gray(config.DistanceMinGray), new Gray(255));
            CvInvoke.ConnectedComponents(markers, markers);
            var finalMarkers = markers.Convert<Gray, int>();

            CvInvoke.Watershed(imgOut, finalMarkers);

            Image<Gray, byte> boundaries = finalMarkers.Convert(delegate (int x)
            {
                return (byte)(x == -1 ? 255 : 0);
            });

            boundaries._Dilate(1);
            imgOut.SetValue(new Bgr(0, 255, 0), boundaries);

            var imgOutColor = imgOut.Convert<Bgr, byte>();

            return (imgOutColor, notes);
        }


        // Analyse input image using OpenCV "Contour" feature. 
        public static Emgu.CV.Util.VectorOfVectorOfPoint AnalyseImageContours(ProcessConfigModel config, Image<Bgr, byte> imgInput)
        {
            var imgInputGray = Threshold(config, ToGrayScale(Smooth(config, imgInput)));

            // Find contours 
            var contours = new Emgu.CV.Util.VectorOfVectorOfPoint();
            Mat hier = new();
            CvInvoke.FindContours(imgInputGray, contours, hier,
                RetrType.External, // Exclude "hole" contours inside other contours.
                ChainApproxMethod.ChainApproxNone); // Hot images are rare, so return all pixels in contour.

            return contours;
        }


        // Analyse input image using OpenCV "Contour" feature
        // Return copy of input image, with hot pixel contours in green, with red bounding rects
        // using OpenCV "Contour" feature. 
        public static (Image<Bgr, byte>, string) DrawContour(
            ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            Image<Bgr, byte> imgInput)
        {
            var contours = AnalyseImageContours(processConfig, imgInput);

            var imgOutColor = imgInput.Clone();

            var numFilteredContours = SkyCombDrone.DrawSpace.Draw.ContoursAndPixels(
                drawConfig, processConfig.ContourMinArea, ref imgOutColor, contours);

            string notes = string.Format("NumContours={0}, FilteredContours={1}, BoundingRects={2}",
                contours.Size,
                numFilteredContours,
                numFilteredContours);

            return (imgOutColor, notes);
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
        public static (Image<Bgr, byte>, string) DrawGftt(
            ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            Image<Bgr, byte> imgInput)
        {
            Image<Gray, byte> imgInputGray = ToGrayScale(imgInput);

            // Detect GFTT points on a grayscale image
            var keypoints = Detect_GFTT(processConfig, imgInputGray);
            string notes = "KeyPoints=" + keypoints.Length;

            var imgOutColor = imgInput.Clone();
            foreach (MKeyPoint p in keypoints)
            {
                SkyCombDrone.DrawSpace.Draw.Circle(drawConfig, ref imgOutColor, Point.Round(p.Point));

                // Debugging - from original code.
                if (float.IsNaN(p.Point.X) || float.IsNaN(p.Point.Y))
                    throw BaseConstants.ThrowException("Process.Image.Process_GFTT: IsNaN");
            }

            return (imgOutColor, notes);
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
        public static (Image<Bgr, byte>, string) Draw(
            RunProcessEnum runProcess, ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            Image<Bgr, byte> imgInput)
        {
            Image<Bgr, byte> image = null;
            string notes = "";

            switch (runProcess)
            {
                case RunProcessEnum.Distance: (image, notes) = DrawDistance(processConfig, imgInput); break;
                case RunProcessEnum.Contour: (image, notes) = DrawContour(processConfig, drawConfig, imgInput); break;
                case RunProcessEnum.GFTT: (image, notes) = DrawGftt(processConfig, drawConfig, imgInput); break;
                case RunProcessEnum.Flow: image = imgInput.Clone(); break; // Do nothing. Flow is a video-specific process.  
                case RunProcessEnum.Comb: image = imgInput.Clone(); break; // Do nothing. Comb is a video-specific process. 
                case RunProcessEnum.Threshold: (image, notes) = DrawThreshold(processConfig, imgInput); break;
                case RunProcessEnum.None: image = imgInput.Clone(); break; // Do nothing. Useful to creating a shorter video clip, or testing the "Save" functionality.
                default: throw BaseConstants.ThrowException("Process.Image.DrawImage: Illegal ImageProcess value:" + runProcess);
            }

            return (image, notes);
        }
    }
}