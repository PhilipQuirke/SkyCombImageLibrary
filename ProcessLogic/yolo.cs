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
    public class YoloResult
    {
        public DetectionResult? Result { get; }

        public SixLabors.ImageSharp.Image? Image { get; }

        public YoloResult(DetectionResult? result = null, SixLabors.ImageSharp.Image? image = null)
        {
            Result = result;
            Image = image;
        }
    }


    // YOLO (You only look once) V8 image processing.
    // Uses a SkyComb-specific pre-trained model to detect objects in an image.
    public class YoloV8 : BaseConstants
    {
        YoloV8Predictor? DetectPredictor = null;


        public YoloV8(string modelDirectory)
        {
            Assert(modelDirectory != "", "modelDirectory is not specified");

            // If modelDirectory doesnt end in ".onnx" append "\model.onnx" or "model.onnx"
            if (!modelDirectory.EndsWith(".onnx"))
            {
                if (modelDirectory.EndsWith("\\") || modelDirectory.EndsWith("/"))
                    modelDirectory += "model.onnx";
                else
                    modelDirectory += "\\model.onnx";
            }

            try
            {
                // Load the model
                DetectPredictor = YoloV8Predictor.Create(modelDirectory);
            }
            catch (Exception ex)
            {
                DetectPredictor = null;
                throw ThrowException("YoloV8.Constructor failed: " + ex.Message);
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
            if (DetectPredictor == null)
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
                throw ThrowException("YoloV8.AsyncGetImage failed: " + ex.Message);
            }
        }


        SixLabors.ImageSharp.Image? GetImage(DetectionResult result, System.Drawing.Image raw_image)
        {
            if (DetectPredictor == null)
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
                throw ThrowException("YoloV8.GetImage failed: " + ex.Message);
            }
        }


        public async Task<YoloResult> AsyncDetect(System.Drawing.Image raw_image)
        {
            if (DetectPredictor == null)
                return new YoloResult();

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                DetectionResult? result = await DetectPredictor.DetectAsync(image);
                if (result is null)
                    return new YoloResult();

                Console.WriteLine($"Task:   {DetectPredictor.Metadata.Task}");
                Console.WriteLine($"Image:  {image}");
                Console.WriteLine($"Result: {result}");
                Console.WriteLine($"Speed:  {result.Speed}");

                using var plotted = await AsyncGetImage(result, raw_image);

                return new YoloResult( result, plotted );
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloV8.AsyncDetect failed: " + ex.Message);
            }
        }


        public YoloResult Detect(System.Drawing.Image raw_image)
        {
            if (DetectPredictor == null)
                return new YoloResult();

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                DetectionResult? result = DetectPredictor.Detect(image);
                if (result is null)
                    return new YoloResult();

                Console.WriteLine($"Task:   {DetectPredictor.Metadata.Task}");
                Console.WriteLine($"Image:  {image}");
                Console.WriteLine($"Result: {result}");
                Console.WriteLine($"Speed:  {result.Speed}");

                using var plotted = GetImage(result, raw_image);

                return new YoloResult(result, plotted);
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloV8.Detect failed: " + ex.Message);
            }
        }
    }
}
