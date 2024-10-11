// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
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


        // We need to resize the theImage by a factor
        public static Mat ResizeImage(Image<Bgr, byte> theImage, double factor)
        {
            var frameResized = new Mat();
            CvInvoke.Resize(theImage, frameResized, new Size(0, 0), factor, factor, Inter.Linear);

            return frameResized;
        }


        // Process a single image and returns an image.
        public static void Draw(
            RunProcessEnum runProcess, ProcessConfigModel processConfig, DrawImageConfig drawConfig,
            ref Image<Bgr, byte> imgInput)
        {
            if (runProcess == RunProcessEnum.Threshold)
                DrawThreshold(processConfig, ref imgInput);
        }
    }
}