// Copyright SkyComb Limited 2024. All rights reserved. 

// Refer https://github.com/dme-compunet/YOLOv8
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using Emgu.CV;

// Refer https://sixlabors.com/products/imagesharp/
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


        public YoloResult<Detection>? Detect(System.Drawing.Image raw_image)
        {
            YoloResult<Detection>? answer = null;

            try
            {
                if (Detector != null)
                {
                    using (var memoryStream = new MemoryStream())
                    {
                        raw_image.Save(memoryStream, System.Drawing.Imaging.ImageFormat.Bmp);

                        memoryStream.Position = 0;

                        using (var image = SixLabors.ImageSharp.Image.Load<Rgba32>(memoryStream))
                        {
                            answer = Detector.Detect(image, DetectorConfig);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloDetect.Detect failed: " + ex.Message);
            }

            return answer;
        }

    }
}
