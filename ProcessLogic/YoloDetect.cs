// Copyright SkyComb Limited 2024. All rights reserved. 

// https://github.com/NickSwardh/YoloDotNet
using SkiaSharp;
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using System.Diagnostics;
using YoloDotNet;
using YoloDotNet.Enums;
using YoloDotNet.Models;


namespace SkyCombImage.ProcessLogic
{
    public enum YoloProcessMode
    {
        ByFrame, // Process frame by frame. Most accurate but slowest
        TimeRange, // Process leg by leg
        FullVideo // Fastest processing per frame
    }


    // YOLO (You only look once) V8 image object detector.
    // Uses a SkyComb-specific pre-trained model to detect objects in an image.
    public class YoloDetect : BaseConstants
    {
        readonly float Confidence = UnknownValue;
        readonly float IoU = UnknownValue;
        readonly string YoloPath;

        Yolo ? YoloTool = null;
        public YoloProcessMode ProcessMode;
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

            ProcessMode = YoloProcessMode.FullVideo;
            Results = new();
        }


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


        // Run Yolo detection over the entire video file
        private Dictionary<int, List<ObjectDetection>>? DetectVideo(string videoFileName)
        {
            EnsureModelLoaded();

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
                //FromFrame = -1,
                //ToFrame = -1,
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


        public Dictionary<int, List<ObjectDetection>>? DetectTimeRange(string videoFileName, int fromSecond, int toSecond)
        {
            EnsureModelLoaded();

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
                FromSecond = fromSecond,
                ToSecond = toSecond,
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


        public void ProcessScope(ProcessScope scope, VideoData inputVideo)
        {
            // Yolo processing frame by frame takes approximately twice as long per frame as processing the whole video.
            // Process all frames if user has specified a time range that is >= 50% of the video duration.
            if (scope.PSM.InputVideoDurationMs >= inputVideo.DurationMs / 2)
            {
                // Process the entire video file, using YOLO and GPU. Do not create an output file yet.
                ProcessMode = YoloProcessMode.FullVideo;
                Results = DetectVideo(inputVideo.FileName);
            }
            else
            {
                ProcessMode = YoloProcessMode.TimeRange;
                Results = DetectTimeRange(inputVideo.FileName, scope.PSM.FirstVideoFrameMs / 1000, scope.PSM.LastVideoFrameMs / 1000 + 1);
            }
        }


        // Run both full-video and time-section detection tests on a single video.
        public void UnitTest()
        {
            EnsureModelLoaded();

            string videoFileName = "D:\\SkyComb\\Data_Input\\CC\\2023-05-D\\DJI_20230531190425_0001_S.mp4";
            double time1 = 0;
            double time2 = 0;
            double time3 = 0;
            double time4 = 0;
            Dictionary<int, List<ObjectDetection>>? results1 = null;
            Dictionary<int, List<ObjectDetection>>? results2 = null;
            Dictionary<int, List<ObjectDetection>>? results3 = null;
            Dictionary<int, List<ObjectDetection>>? results4 = null;

            {
                Stopwatch stopwatch = new Stopwatch();
                stopwatch.Start();
                results1 = this.DetectVideo(videoFileName); // Full video
                stopwatch.Stop();
                time1 = stopwatch.Elapsed.TotalMilliseconds;
                System.Threading.Thread.Sleep(500);
            }
            {
                Stopwatch stopwatch = new Stopwatch();
                stopwatch.Start();
                results2 = this.DetectTimeRange(videoFileName, 30, 50); // 20 secs
                stopwatch.Stop();
                time2 = stopwatch.Elapsed.TotalMilliseconds;
                System.Threading.Thread.Sleep(500);
            }
            {
                Stopwatch stopwatch = new Stopwatch();
                stopwatch.Start();
                results3 = this.DetectTimeRange(videoFileName, 60, 70); // 10 secs
                stopwatch.Stop();
                time3 = stopwatch.Elapsed.TotalMilliseconds;
                System.Threading.Thread.Sleep(500);
            }
            {
                Stopwatch stopwatch = new Stopwatch();
                stopwatch.Start();
                results4 = this.DetectTimeRange(videoFileName, 80, 90); // 10 secs
                stopwatch.Stop();
                time4 = stopwatch.Elapsed.TotalMilliseconds;
                System.Threading.Thread.Sleep(500);
            }

            var time = time1 + time2 + time3 + time4;
            var results = results1.Count + results2.Count + results3.Count + results4.Count;
        }
    }
}
