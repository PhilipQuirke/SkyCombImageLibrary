// Copyright SkyComb Limited 2025. All rights reserved.
using SkyCombImage.Interfaces;
using SkyCombImage.Services;
using SkyCombDrone.Interfaces;
using SkyCombDrone.Services;
using SkyCombImage.Exceptions;

namespace SkyCombImage.Examples
{
    /// <summary>
    /// Basic examples showing how to use the SkyComb Image Library
    /// </summary>
    public static class BasicUsageExamples
    {
        /// <summary>
        /// Example: Basic object detection on a drone video using Comb algorithm
        /// </summary>
        public static async Task BasicCombDetectionExample()
        {
            Console.WriteLine("=== Basic Comb Object Detection Example ===");

            // Step 1: Load drone data (video with flight log)
            var droneService = DroneDataService.Create();
            string videoPath = @"C:\DroneVideos\thermal_flight.mp4";
            string groundDataPath = @"C:\GroundData";

            using var droneData = await droneService.LoadVideoDataAsync(videoPath, groundDataPath);
            Console.WriteLine($"Loaded flight: Duration {droneData.FlightSummary.Duration:hh\\:mm\\:ss}, Camera: {droneData.FlightSummary.CameraType}");

            // Step 2: Configure image processing
            var imageService = ImageProcessingService.Create();
            var options = new ImageProcessingOptions
            {
                Algorithm = DetectionAlgorithm.Comb,
                HeatThreshold = 235,  // For thermal imagery
                SaveAnnotatedVideo = true,
                SaveObjectData = ObjectDataSaveMode.Significant
            };

            // Step 3: Process video with progress reporting
            var progress = new Progress<ImageProcessingProgress>(p =>
                Console.WriteLine($"  {p.Stage}: {p.PercentComplete}% ({p.CurrentFrame}/{p.TotalFrames} frames) - {p.ObjectsDetectedSoFar} objects"));

            using var result = await imageService.ProcessVideoAsync(droneData, options, progress);

            // Step 4: Display results
            Console.WriteLine($"\nResults:");
            Console.WriteLine($"  Algorithm: {result.AlgorithmUsed}");
            Console.WriteLine($"  Total objects detected: {result.TotalObjectsDetected}");
            Console.WriteLine($"  Significant objects: {result.SignificantObjectsDetected}");
            Console.WriteLine($"  Processing time: {result.ProcessingDuration:hh\\:mm\\:ss}");
            Console.WriteLine($"  Annotated video: {result.AnnotatedVideoPath}");
            Console.WriteLine($"  Data file: {result.DataFilePath}");

            // Step 5: Examine detected objects
            var significantObjects = result.GetSignificantObjects();
            foreach (var obj in significantObjects.Take(5)) // Show first 5
            {
                Console.WriteLine($"  Object {obj.Name}: Size {obj.SizeCM2:F0}cm², Height {obj.HeightAboveGroundM:F1}m, Duration {obj.TrackingDuration.TotalSeconds:F1}s");
            }
        }

        /// <summary>
        /// Example: YOLO-based object detection with GPU acceleration
        /// </summary>
        public static async Task YoloDetectionExample()
        {
            Console.WriteLine("\n=== YOLO Object Detection Example ===");

            var droneService = DroneDataService.Create();
            var imageService = ImageProcessingService.Create();

            string videoPath = @"C:\DroneVideos\thermal_survey.mp4";
            string groundDataPath = @"C:\GroundData";
            string yoloModelsPath = @"C:\YoloModels";

            try
            {
                // Load drone data
                using var droneData = await droneService.LoadVideoDataAsync(videoPath, groundDataPath);
                Console.WriteLine($"Loaded video: {droneData.FlightSummary.Duration:hh\\:mm\\:ss}");

                // Configure YOLO processing
                var options = new ImageProcessingOptions
                {
                    Algorithm = DetectionAlgorithm.Yolo,
                    YoloConfidence = 0.25f,
                    YoloIoU = 0.45f,
                    YoloModelDirectory = yoloModelsPath,
                    SaveAnnotatedVideo = true,
                    SaveObjectData = ObjectDataSaveMode.All
                };

                // Get processing preview first
                Console.WriteLine("Getting processing preview...");
                var preview = await imageService.GetProcessingPreviewAsync(droneData, options, maxFramesToSample: 20);
                Console.WriteLine($"Preview: ~{preview.EstimatedObjectCount} objects estimated, ~{preview.EstimatedProcessingTime:hh\\:mm\\:ss} processing time");

                // Process with progress reporting
                using var result = await imageService.ProcessVideoAsync(droneData, options, 
                    new Progress<ImageProcessingProgress>(p => Console.WriteLine($"  {p.PercentComplete}% - {p.Stage}")));

                Console.WriteLine($"YOLO Results: {result.SignificantObjectsDetected} significant objects detected");

                // Group objects by flight leg
                if (droneData.HasFlightLegs)
                {
                    foreach (var leg in droneData.GetFlightLegs())
                    {
                        var objectsInLeg = result.GetObjectsInLeg(leg.LegId);
                        Console.WriteLine($"  Leg {leg.LegName}: {objectsInLeg.Count} objects");
                    }
                }
            }
            catch (YoloModelNotFoundException ex)
            {
                Console.WriteLine($"Error: YOLO models not found at {ex.ModelPath}");
                Console.WriteLine("Please ensure YOLO model files are available and GPU drivers are installed.");
            }
        }

        /// <summary>
        /// Example: Processing multiple videos in batch
        /// </summary>
        public static async Task BatchProcessingExample()
        {
            Console.WriteLine("\n=== Batch Processing Example ===");

            var droneService = DroneDataService.Create();
            var imageService = ImageProcessingService.Create();

            string[] videoPaths = {
                @"C:\DroneVideos\flight_001.mp4",
                @"C:\DroneVideos\flight_002.mp4",
                @"C:\DroneVideos\flight_003.mp4"
            };

            string groundDataPath = @"C:\GroundData";

            var options = new ImageProcessingOptions
            {
                Algorithm = DetectionAlgorithm.Comb,
                HeatThreshold = 250,
                SaveObjectData = ObjectDataSaveMode.Significant
            };

            var allResults = new List<(string VideoPath, IImageProcessingResult Result)>();

            foreach (string videoPath in videoPaths)
            {
                if (!File.Exists(videoPath))
                {
                    Console.WriteLine($"Skipping missing file: {Path.GetFileName(videoPath)}");
                    continue;
                }

                try
                {
                    Console.WriteLine($"Processing: {Path.GetFileName(videoPath)}");
                    
                    using var droneData = await droneService.LoadVideoDataAsync(videoPath, groundDataPath);
                    var result = await imageService.ProcessVideoAsync(droneData, options);
                    
                    allResults.Add((videoPath, result));
                    
                    Console.WriteLine($"  Completed: {result.SignificantObjectsDetected} objects found in {result.ProcessingDuration:mm\\:ss}");
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"  Error processing {Path.GetFileName(videoPath)}: {ex.Message}");
                }
            }

            // Summary
            Console.WriteLine($"\nBatch Summary:");
            Console.WriteLine($"  Videos processed: {allResults.Count}");
            Console.WriteLine($"  Total objects: {allResults.Sum(r => r.Result.TotalObjectsDetected)}");
            Console.WriteLine($"  Total significant: {allResults.Sum(r => r.Result.SignificantObjectsDetected)}");
            Console.WriteLine($"  Total processing time: {TimeSpan.FromMilliseconds(allResults.Sum(r => r.Result.ProcessingDuration.TotalMilliseconds)):hh\\:mm\\:ss}");

            // Cleanup
            foreach (var (_, result) in allResults)
            {
                result.Dispose();
            }
        }

        /// <summary>
        /// Example: Error handling and troubleshooting
        /// </summary>
        public static async Task ErrorHandlingExample()
        {
            Console.WriteLine("\n=== Error Handling Example ===");

            var droneService = DroneDataService.Create();
            var imageService = ImageProcessingService.Create();

            string videoPath = @"C:\DroneVideos\test_flight.mp4";
            string groundDataPath = @"C:\GroundData";

            try
            {
                using var droneData = await droneService.LoadVideoDataAsync(videoPath, groundDataPath);
                
                var options = new ImageProcessingOptions
                {
                    Algorithm = DetectionAlgorithm.Yolo,
                    YoloModelDirectory = @"C:\InvalidPath"  // This will cause an error
                };

                using var result = await imageService.ProcessVideoAsync(droneData, options);
            }
            catch (FileNotFoundException ex)
            {
                Console.WriteLine($"File not found: {ex.Message}");
                Console.WriteLine("Solution: Check that the video file path is correct.");
            }
            catch (YoloModelNotFoundException ex)
            {
                Console.WriteLine($"YOLO model error: {ex.Message}");
                Console.WriteLine($"Solution: Ensure YOLO model files are present in: {ex.ModelPath}");
            }
            catch (GpuRequirementsException ex)
            {
                Console.WriteLine($"GPU requirement error: {ex.Message}");
                Console.WriteLine("Solution: Install CUDA drivers and ensure GPU compatibility.");
            }
            catch (InvalidVideoDataException ex)
            {
                Console.WriteLine($"Video data error: {ex.Message}");
                Console.WriteLine("Solution: Ensure video has associated flight log (SRT file).");
            }
            catch (ObjectDetectionException ex)
            {
                Console.WriteLine($"Detection error: {ex.Message}");
                if (ex.FrameNumber.HasValue)
                    Console.WriteLine($"Error occurred at frame: {ex.FrameNumber}");
                Console.WriteLine("Solution: Check video quality and processing parameters.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Unexpected error: {ex.Message}");
                Console.WriteLine("Solution: Contact support with error details.");
            }
        }

        /// <summary>
        /// Example: Advanced configuration and customization
        /// </summary>
        public static async Task AdvancedConfigurationExample()
        {
            Console.WriteLine("\n=== Advanced Configuration Example ===");

            // Custom drone data loading options
            var droneOptions = new DroneDataOptions
            {
                FullDataLoad = true,
                AutoDetectLegs = true,
                BufferDistanceM = 100  // Larger buffer for ground data
            };

            var droneService = DroneDataService.Create(droneOptions);
            var imageService = ImageProcessingService.Create();

            string videoPath = @"C:\DroneVideos\complex_flight.mp4";
            string groundDataPath = @"C:\GroundData";

            using var droneData = await droneService.LoadVideoDataAsync(videoPath, groundDataPath);

            // Fine-tuned processing options
            var processingOptions = new ImageProcessingOptions
            {
                Algorithm = DetectionAlgorithm.Comb,
                HeatThreshold = 180,  // Lower threshold for cooler objects
                SaveAnnotatedVideo = true,
                SaveObjectData = ObjectDataSaveMode.All  // Save everything for analysis
            };

            // Custom progress reporting
            var detailedProgress = new Progress<ImageProcessingProgress>(progress =>
            {
                Console.WriteLine($"[{DateTime.Now:HH:mm:ss}] {progress.Stage}");
                Console.WriteLine($"  Progress: {progress.PercentComplete}% ({progress.CurrentFrame}/{progress.TotalFrames})");
                Console.WriteLine($"  Objects found so far: {progress.ObjectsDetectedSoFar}");
                
                if (progress.EstimatedTimeRemaining.HasValue)
                    Console.WriteLine($"  Estimated time remaining: {progress.EstimatedTimeRemaining:hh\\:mm\\:ss}");
            });

            using var result = await imageService.ProcessVideoAsync(droneData, processingOptions, detailedProgress);

            // Detailed analysis
            Console.WriteLine($"\nDetailed Results Analysis:");
            Console.WriteLine($"Video processed: {result.VideoInfo.InputVideoPath}");
            Console.WriteLine($"Frames processed: {result.VideoInfo.FramesProcessed} at {result.VideoInfo.FrameRate:F1} fps");
            Console.WriteLine($"Processing efficiency: {result.VideoInfo.FramesProcessed / result.ProcessingDuration.TotalSeconds:F1} frames/second");

            // Object size distribution
            var allObjects = result.GetDetectedObjects();
            var sizeGroups = allObjects.GroupBy(obj => 
                obj.SizeCM2 switch
                {
                    < 100 => "Small (< 100cm²)",
                    < 500 => "Medium (100-500cm²)",
                    < 1000 => "Large (500-1000cm²)",
                    _ => "Very Large (> 1000cm²)"
                });

            Console.WriteLine($"Object size distribution:");
            foreach (var group in sizeGroups.OrderBy(g => g.Key))
            {
                Console.WriteLine($"  {group.Key}: {group.Count()} objects");
            }
        }
    }
}