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
        // Singleton instance
        private static YoloDetect? _instance = null;
        // Lock object for thread safety
        private static readonly object _lock = new object();


        // Yolo model parameters
        private static float Confidence = UnknownValue;
        private static float IoU = UnknownValue;
        private static string YoloPath = "";
        private static Yolo? YoloTool = null;


        // Private constructor to prevent instantiation from outside
        private YoloDetect(string yoloPath, float confidence, float iou)
        {
            Assert(yoloPath != "", "yoloPath is not specified");

            Confidence = confidence;
            IoU = iou;
            YoloPath = yoloPath;
            if (!YoloPath.EndsWith(".onnx"))
                // The SkyComb Yolo models were generated in and exported from Supervisely.
                // More details in D:\SkyComb\Data_Yolo\YoloV8_14Oct\ModelTrainingDetails.docx
                YoloPath = Path.Combine(YoloPath, "SkyCombYoloV8.onnx");
        }


        // Public method to get the singleton instance. Means SkyCombFlights will only load the model once.
        public static YoloDetect GetInstance(string yoloPath, float confidence, float iou)
        {
            // Double-check locking pattern for thread safety
            if (_instance == null)
            {
                lock (_lock)
                {
                    if (_instance == null)
                    {
                        _instance = new YoloDetect(yoloPath, confidence, iou);
                    }
                }
            }
            else
            {
                YoloDetect.Confidence = confidence;
                YoloDetect.IoU = iou;
            }
            return _instance;
        }

        // Processing requires GPU and CUDA libraries.
        // Defers loading the model until processing starts.
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
