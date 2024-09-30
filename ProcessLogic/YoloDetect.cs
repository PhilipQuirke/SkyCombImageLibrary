// Copyright SkyComb Limited 2024. All rights reserved. 

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
        Yolo? YoloTool = null;
        float Confidence = UnknownValue;
        float IoU = UnknownValue;


        public YoloDetect(string yoloPath, float confidence, float iou)
        {
            Assert(yoloPath != "", "yoloPath is not specified");

            // If modelDirectory doesnt end in ".onnx" append suffix
            if (!yoloPath.EndsWith(".onnx"))
            {
                // First SkyComb-specific model was trained in Supervisely in June 2024 by fine-tuning a YOLO COCO V8 "medium" model
                if (yoloPath.EndsWith("\\") || yoloPath.EndsWith("/"))
                    yoloPath += "YoloV8_s_e100.onnx";
                else
                    yoloPath += "\\YoloV8_s_e100.onnx";
            }

            try
            {
                // Load the model. Takes a few seconds.
                YoloTool = new Yolo(new YoloOptions
                {
                    OnnxModel = yoloPath,                   // Your Yolov8 or Yolov10 model in onnx format
                    ModelType = ModelType.ObjectDetection,  // Model type
                    Cuda = true,                           // Use CPU or CUDA for GPU accelerated inference. Default = true
                    GpuId = 0,                              // Select Gpu by id. Default = 0
                    PrimeGpu = false,                       // Pre-allocate GPU before first. Default = false
                });
            }
            catch (Exception ex)
            {
                YoloTool = null;
                throw ThrowException("YoloDetect.Constructor failed: " + ex.Message);
            }

            Confidence = confidence;
            IoU = iou;
        }


        public List<ObjectDetection>? DetectFrame(System.Drawing.Image raw_image)
        {
            List<ObjectDetection>? answer = null;

            try
            {
                if (YoloTool != null)
                {
                    using (var memoryStream = new MemoryStream())
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


        public Dictionary<int,List<ObjectDetection>>? DetectVideo(string videoFileName)
        {
            // Set video options
            var options = new VideoOptions
            {
                VideoFile = videoFileName,
                OutputDir = "",
                GenerateVideo = false,
                DrawLabels = false,
                //FPS = 30,
                //Width = 640,  // Resize video...
                //Height = -2,  // -2 automatically calculate dimensions to keep proportions
                //Quality = 28,
                DrawConfidence = false,
                KeepAudio = false,
                //KeepFrames = false,
                //DrawSegment = DrawSegment.Default,
                //PoseOptions = MyPoseMarkerConfiguration // Your own pose marker configuration...
            };

            // Run inference on video
            var results = YoloTool?.RunObjectDetection(options, Confidence, IoU);

            return results;
        }
    }
}
