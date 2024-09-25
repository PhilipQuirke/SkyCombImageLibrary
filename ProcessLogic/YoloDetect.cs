// Copyright SkyComb Limited 2024. All rights reserved. 

// Refer https://github.com/dme-compunet/YOLOv8
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using Compunet.YoloV8.Plotting;
using Emgu.CV;

// Refer https://sixlabors.com/products/imagesharp/
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

using SkyCombGround.CommonSpace;



namespace SkyCombImage.ProcessLogic
{
    // YOLO (You only look once) V8 image object detector.
    // Uses a SkyComb-specific pre-trained model to detect objects in an image.
    public class YoloDetect : BaseConstants
    {
        YoloPredictor? Detector = null;
        YoloConfiguration DetectorConfig;


        public YoloDetect(string yoloDirectory, float confidence, float iou)
        {
            Assert(yoloDirectory != "", "yoloDirectory is not specified");

            // If modelDirectory doesnt end in ".onnx" append suffix
            if (!yoloDirectory.EndsWith(".onnx"))
            {
                // First SkyComb-specific model was trained in Supervisely in June 2024 by fine-tuning a YOLO COCO V8 "medium" model
                if (yoloDirectory.EndsWith("\\") || yoloDirectory.EndsWith("/"))
                    yoloDirectory += "yolo_v8_s_e100.onnx";
                else
                    yoloDirectory += "\\yolo_v8_s_e100.onnx";
            }

            try
            {
                // Load the model. Takes a few seconds.
                Detector = new YoloPredictor(yoloDirectory);
                // Detector.ToDevice( 'cuda'); PQR
            }
            catch (Exception ex)
            {
                Detector = null;
                throw ThrowException("YoloDetect.Constructor failed: " + ex.Message);
            }

            DetectorConfig = new YoloConfiguration();
            DetectorConfig.Confidence = confidence;
            DetectorConfig.IoU = iou; 
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


        async Task<SixLabors.ImageSharp.Image?> AsyncGetImage(YoloResult<Detection> result, System.Drawing.Image raw_image)
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


        SixLabors.ImageSharp.Image? GetImage(YoloResult<Detection> result, System.Drawing.Image raw_image)
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


        public async Task<YoloResult<Detection>?> AsyncDetect(System.Drawing.Image raw_image)
        {
            if (Detector == null)
                return null;

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                YoloResult<Detection>? result = await Detector.DetectAsync(image);

                return result;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.AsyncDetect failed: " + ex.Message);
            }
        }


        public YoloResult<Detection>? Detect(System.Drawing.Image raw_image)
        {
            if (Detector == null)
                return null;

            try
            {
                using SixLabors.ImageSharp.Image image = ConvertToImageSharp(raw_image);

                return Detector.Detect(image, DetectorConfig);
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.Detect failed: " + ex.Message);
            }
        }
    }
}
