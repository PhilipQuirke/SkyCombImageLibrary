// Refer https://github.com/dme-compunet/YOLOv8
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using Compunet.YoloV8.Plotting;
using Emgu.CV;
using Emgu.CV.Structure;

// Refer https://sixlabors.com/products/imagesharp/
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Jpeg;
using SixLabors.ImageSharp.PixelFormats;

using SkyCombGround.CommonSpace;
using System.Configuration;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // YOLO (You only look once) V8 image object detector.
    // Uses a SkyComb-specific pre-trained model to detect objects in an image.
    public class YoloDetect : BaseConstants
    {
        YoloV8Predictor? Detector = null;


        public YoloDetect(string yoloDirectory)
        {
            Assert(yoloDirectory != "", "yoloDirectory is not specified");

            // If modelDirectory doesnt end in ".onnx" append "\model.onnx" or "model.onnx"
            if (!yoloDirectory.EndsWith(".onnx"))
            {
                if (yoloDirectory.EndsWith("\\") || yoloDirectory.EndsWith("/"))
                    yoloDirectory += "yolo_v8_model.onnx";
                else
                    yoloDirectory += "\\yolyolo_v8_modelo_model.onnx";
            }

            try
            {
                // Load the model. Takes a few seconds.
                Detector = YoloV8Predictor.Create(yoloDirectory);
            }
            catch (Exception ex)
            {
                Detector = null;
                throw ThrowException("YoloDetect.Constructor failed: " + ex.Message);
            }
        }


        public static Image<Rgba32> ConvertToImageSharp(System.Drawing.Image drawingImage)
        {
            using (var memoryStream = new MemoryStream())
            {
                // Save the System.Drawing.Image to a memory stream
                drawingImage.Save(memoryStream, System.Drawing.Imaging.ImageFormat.Png);

                // Reset the position of the stream to the beginning
                memoryStream.Position = 0;

                // Load the stream into an ImageSharp Image
                return SixLabors.ImageSharp.Image.Load<Rgba32>(memoryStream);
            }
        }


        async Task<SixLabors.ImageSharp.Image?> AsyncGetImage(DetectionResult result, System.Drawing.Image raw_image)
        {
            if (Detector == null)
                return null;

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                using var plotted = await result.PlotImageAsync(image);

                image.Dispose();

                return plotted;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.AsyncGetImage failed: " + ex.Message);
            }
        }


        SixLabors.ImageSharp.Image? GetImage(DetectionResult result, System.Drawing.Image raw_image)
        {
            if (Detector == null)
                return null;

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                using var plotted = result.PlotImage(image);

                image.Dispose();

                return plotted;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.GetImage failed: " + ex.Message);
            }
        }


        public async Task<DetectionResult?> AsyncDetect(System.Drawing.Image raw_image)
        {
            if (Detector == null)
                return null;

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                DetectionResult? result = await Detector.DetectAsync(image);

                return result;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.AsyncDetect failed: " + ex.Message);
            }
        }


        public DetectionResult? Detect(System.Drawing.Image raw_image)
        {
            if (Detector == null)
                return null;

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                return Detector.Detect(image);
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.Detect failed: " + ex.Message);
            }
        }
    }
}
