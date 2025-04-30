// Copyright SkyComb Limited 2025. All rights reserved. 

// https://github.com/NickSwardh/YoloDotNet
using SkiaSharp;
using SkyCombGround.CommonSpace;
using YoloDotNet;
using YoloDotNet.Enums;
using YoloDotNet.Models;


namespace SkyCombImage.ProcessLogic
{
    // YOLO (You only look once) V8 image object detector.
    // Uses a SkyComb-specific pre-trained model to detect objects in an image.
    public class YoloDetect : BaseConstants
    {
        readonly float Confidence = UnknownValue;
        readonly float IoU = UnknownValue;
        readonly string YoloPath;

        Yolo? YoloTool = null;
        public Dictionary<int, List<ObjectDetection>> Results;


        // Processing requires GPU and CUDA libraries.
        // Defers loading the model until processign starts.
        // Allows SkyCombAnalyst to be used to view (but not re-process) an existing datastore
        public void EnsureModelLoaded()
        {
            if (YoloTool == null)
                try
                {
                    // Load the model. Takes a few seconds.
                    YoloTool = new Yolo(new YoloOptions
                    {
                        OnnxModel = YoloPath,                   // Your Yolov8 or Yolov10 model in onnx format
                        ModelType = ModelType.ObjectDetection,  // Model type
                        Cuda = true,                            // Use CPU or CUDA for GPU accelerated inference. Default = true
                        GpuId = 0,                              // Select Gpu by id. Default = 0
                        PrimeGpu = false,                       // Pre-allocate GPU before first. Default = false
                    });
                }
                catch (Exception ex)
                {
                    YoloTool = null;
                    throw ThrowException("YoloDetect.LoadModel failed: " + ex.Message);
                }
        }


        public YoloDetect(string yoloPath, float confidence, float iou)
        {
            Confidence = confidence;
            IoU = iou;

            Assert(yoloPath != "", "yoloPath is not specified");
            YoloPath = yoloPath;
            if (!YoloPath.EndsWith(".onnx"))
                // The SkyComb Yolo models were generated in and exported from Supervisely.
                // More details in D:\SkyComb\Data_Yolo\YoloV8_14Oct\ModelTrainingDetails.docx
                YoloPath = Path.Combine(YoloPath, "SkyCombYoloV8.onnx");

            Results = new();
        }


        // Run Yolo object detection on a single frame (image)
        public List<ObjectDetection>? DetectFrame(System.Drawing.Image raw_image)
        {
            List<ObjectDetection>? answer = null;

            EnsureModelLoaded();

            try
            {
                if (YoloTool != null)
                {
                    using (MemoryStream memoryStream = new())
                    {
                        raw_image.Save(memoryStream, System.Drawing.Imaging.ImageFormat.Bmp);
                        memoryStream.Position = 0;

                        using (var skData = SKData.Create(memoryStream))
                        using (var the_image = SkiaSharp.SKImage.FromEncodedData(skData))
                        {
                            answer = YoloTool.RunObjectDetection(the_image, confidence: Confidence, iou: IoU);
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
