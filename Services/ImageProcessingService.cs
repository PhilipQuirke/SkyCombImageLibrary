// Copyright SkyComb Limited 2025. All rights reserved.
using SkyCombImage.Interfaces;
using SkyCombImage.RunSpace;
using SkyCombImage.Exceptions;
using SkyCombDrone.Interfaces;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using System.Diagnostics;
using System.Drawing;

namespace SkyCombImage.Services
{
    /// <summary>
    /// Service implementation for processing drone imagery to detect objects
    /// </summary>
    public class ImageProcessingService : IImageProcessingService
    {
        /// <summary>
        /// Creates a new instance of the image processing service
        /// </summary>
        /// <returns>A new image processing service instance</returns>
        public static IImageProcessingService Create()
        {
            return new ImageProcessingService();
        }

        /// <summary>
        /// Processes drone video data to detect objects
        /// </summary>
        public async Task<IImageProcessingResult> ProcessVideoAsync(IDroneData droneData, ImageProcessingOptions options, CancellationToken cancellationToken = default)
        {
            return await ProcessVideoAsync(droneData, options, null, cancellationToken);
        }

        /// <summary>
        /// Processes drone video data with progress reporting
        /// </summary>
        public async Task<IImageProcessingResult> ProcessVideoAsync(IDroneData droneData, ImageProcessingOptions options, IProgress<ImageProcessingProgress>? progress, CancellationToken cancellationToken = default)
        {
            // Validation
            if (droneData == null)
                throw new ArgumentNullException(nameof(droneData));
            if (options == null)
                throw new ArgumentNullException(nameof(options));
            if (!droneData.HasVideoData)
                throw new InvalidVideoDataException("Drone data does not contain video information required for image processing");

            // Validate YOLO requirements
            if (options.Algorithm == DetectionAlgorithm.Yolo)
            {
                ValidateYoloRequirements(options);
            }

            var stopwatch = Stopwatch.StartNew();

            try
            {
                // Report initial progress
                progress?.Report(new ImageProcessingProgress
                {
                    Stage = "Initializing image processing",
                    PercentComplete = 0,
                    CurrentFrame = 0,
                    TotalFrames = 0
                });

                // Create internal processing configuration
                var runConfig = CreateRunConfig(droneData, options);
                var dataStore = CreateDataStore(droneData, runConfig);
                var drone = CreateDroneFromData(droneData);
                
                // Create progress reporting wrapper
                var progressWrapper = progress != null ? new ProgressReportingUserInterface(progress) : new BasicUserInterface();

                // Create internal processing worker
                using var runWorker = RunWorkerFactory.Create(progressWrapper, runConfig, dataStore, drone, null, null);

                // Execute processing
                progress?.Report(new ImageProcessingProgress
                {
                    Stage = "Processing video frames",
                    PercentComplete = 1,
                    CurrentFrame = 0,
                    TotalFrames = runWorker.PSM?.LastBlockId ?? 0
                });

                var numSignificantObjects = await Task.Run(() =>
                {
                    return runWorker.Run();
                }, cancellationToken);

                stopwatch.Stop();

                // Create result wrapper
                var result = new ImageProcessingResultWrapper(
                    runWorker,
                    options.Algorithm,
                    stopwatch.Elapsed,
                    droneData
                );

                progress?.Report(new ImageProcessingProgress
                {
                    Stage = "Processing complete",
                    PercentComplete = 100,
                    ObjectsDetectedSoFar = result.SignificantObjectsDetected,
                    EstimatedTimeRemaining = TimeSpan.Zero
                });

                return result;
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex) when (!(ex is ImageProcessingException))
            {
                throw new ObjectDetectionException($"Failed to process drone video: {ex.Message}", ex);
            }
        }

        /// <summary>
        /// Gets a quick preview of what would be detected without full processing
        /// </summary>
        public async Task<ImageProcessingPreview> GetProcessingPreviewAsync(IDroneData droneData, ImageProcessingOptions options, int maxFramesToSample = 10, CancellationToken cancellationToken = default)
        {
            if (droneData == null)
                throw new ArgumentNullException(nameof(droneData));
            if (options == null)
                throw new ArgumentNullException(nameof(options));
            if (!droneData.HasVideoData)
                throw new InvalidVideoDataException("Drone data does not contain video information");

            // For now, return estimated values based on flight duration and algorithm
            await Task.Delay(100, cancellationToken); // Placeholder for actual preview logic

            var estimatedObjects = EstimateObjectCount(droneData, options);
            var estimatedTime = EstimateProcessingTime(droneData, options);

            return new ImageProcessingPreview
            {
                EstimatedObjectCount = estimatedObjects,
                EstimatedProcessingTime = estimatedTime,
                SampleDetections = new List<DetectedObject>(),
                FramesSampled = Math.Min(maxFramesToSample, 10)
            };
        }

        private static void ValidateYoloRequirements(ImageProcessingOptions options)
        {
            if (string.IsNullOrEmpty(options.YoloModelDirectory))
                throw new InvalidProcessingConfigurationException("YoloModelDirectory must be specified when using YOLO algorithm");
            
            if (!Directory.Exists(options.YoloModelDirectory))
                throw new YoloModelNotFoundException(options.YoloModelDirectory);

            // Check for required YOLO model files
            var requiredFiles = new[] { "best.onnx", "best.pt" };
            var hasModelFile = requiredFiles.Any(file => File.Exists(Path.Combine(options.YoloModelDirectory, file)));
            
            if (!hasModelFile)
                throw new YoloModelNotFoundException(options.YoloModelDirectory, 
                    new FileNotFoundException($"No YOLO model files found. Expected: {string.Join(", ", requiredFiles)}"));
        }

        private static RunConfig CreateRunConfig(IDroneData droneData, ImageProcessingOptions options)
        {
            var runConfig = new RunConfig();
            
            // Set algorithm
            runConfig.RunProcess = options.Algorithm switch
            {
                DetectionAlgorithm.Comb => RunProcessEnum.Comb,
                DetectionAlgorithm.Yolo => RunProcessEnum.Yolo,
                DetectionAlgorithm.Threshold => RunProcessEnum.Threshold,
                _ => RunProcessEnum.Comb
            };

            // Configure processing parameters
            if (runConfig.ProcessConfig != null)
            {
                runConfig.ProcessConfig.HeatThresholdValue = options.HeatThreshold;
                runConfig.ProcessConfig.YoloDetectConfidence = options.YoloConfidence;
                runConfig.ProcessConfig.YoloIoU = options.YoloIoU;
                runConfig.ProcessConfig.SaveAnnotatedVideo = options.SaveAnnotatedVideo;
                runConfig.ProcessConfig.SaveObjectData = options.SaveObjectData switch
                {
                    ObjectDataSaveMode.None => SaveObjectDataEnum.None,
                    ObjectDataSaveMode.Significant => SaveObjectDataEnum.Significant,
                    ObjectDataSaveMode.All => SaveObjectDataEnum.All,
                    _ => SaveObjectDataEnum.Significant
                };
            }

            if (!string.IsNullOrEmpty(options.YoloModelDirectory))
            {
                runConfig.YoloDirectory = options.YoloModelDirectory;
            }

            return runConfig;
        }

        private static DroneDataStore? CreateDataStore(IDroneData droneData, RunConfig runConfig)
        {
            // Create or open a data store based on the drone data
            try
            {
                // This would need to be implemented based on the actual IDroneData interface
                // For now, return null as a placeholder
                return null;
            }
            catch (Exception ex)
            {
                throw new InvalidVideoDataException($"Failed to create data store: {ex.Message}", ex);
            }
        }

        private static Drone? CreateDroneFromData(IDroneData droneData)
        {
            // Convert IDroneData to internal Drone object
            try
            {
                // This would need to be implemented based on the actual IDroneData interface
                // For now, return null as a placeholder
                return null;
            }
            catch (Exception ex)
            {
                throw new InvalidVideoDataException($"Failed to create drone object: {ex.Message}", ex);
            }
        }

        private static int EstimateObjectCount(IDroneData droneData, ImageProcessingOptions options)
        {
            // Rough estimation based on flight duration and algorithm
            var durationMinutes = droneData.FlightSummary.Duration.TotalMinutes;
            
            return options.Algorithm switch
            {
                DetectionAlgorithm.Yolo => (int)(durationMinutes * 2.5), // YOLO finds more objects
                DetectionAlgorithm.Comb => (int)(durationMinutes * 1.8), // Comb is more selective
                DetectionAlgorithm.Threshold => (int)(durationMinutes * 1.2), // Simple threshold
                _ => (int)(durationMinutes * 1.5)
            };
        }

        private static TimeSpan EstimateProcessingTime(IDroneData droneData, ImageProcessingOptions options)
        {
            var durationMinutes = droneData.FlightSummary.Duration.TotalMinutes;
            
            var processingMinutes = options.Algorithm switch
            {
                DetectionAlgorithm.Yolo => durationMinutes * 0.8, // YOLO is slower due to GPU processing
                DetectionAlgorithm.Comb => durationMinutes * 0.4, // Comb is faster
                DetectionAlgorithm.Threshold => durationMinutes * 0.2, // Threshold is fastest
                _ => durationMinutes * 0.5
            };

            return TimeSpan.FromMinutes(Math.Max(processingMinutes, 0.5)); // Minimum 30 seconds
        }
    }

    /// <summary>
    /// Basic user interface implementation for non-interactive processing
    /// </summary>
    internal class BasicUserInterface : RunUserInterface
    {
        public override void RefreshAll() { }
        public override void DrawUI(RunWorker runWorker) { }
        public override void DrawObjectGrid(RunWorker runWorker, bool showObjectGrid) { }
        public override (string legs, string sizes, string heights) GetMainFilters() => ("", "", "");
        public override List<Image> GetSizeImages() => new();
        public override void ShowStepProgress(RunWorker runWorker, int intervalCount, int stepCount) { }
        public override void BadDuration(RunWorker runWorker) { }
        public override void ShowRunSummary(string summary) { }
        public override RunConfig? RunConfig() => null;
        public override void LegsForm_CopyMainFormButtonState(object theForm) { }
    }

    /// <summary>
    /// User interface implementation that reports progress to an IProgress callback
    /// </summary>
    internal class ProgressReportingUserInterface : BasicUserInterface
    {
        private readonly IProgress<ImageProcessingProgress> _progress;
        private DateTime _lastProgressReport = DateTime.MinValue;

        public ProgressReportingUserInterface(IProgress<ImageProcessingProgress> progress)
        {
            _progress = progress;
        }

        public override void ShowStepProgress(RunWorker runWorker, int intervalCount, int stepCount)
        {
            // Throttle progress reports to avoid overwhelming the UI (max once per second)
            if (DateTime.Now - _lastProgressReport < TimeSpan.FromSeconds(1))
                return;

            if (runWorker.PSM != null)
            {
                var percentComplete = runWorker.PSM.LastBlockId > 0 
                    ? Math.Min(100, (int)(100.0 * runWorker.PSM.CurrBlockId / runWorker.PSM.LastBlockId))
                    : 0;

                var objectsFound = runWorker.ProcessAll?.ProcessObjects?.NumSignificantObjects ?? 0;
                var stage = objectsFound > 0 
                    ? $"Processing frames - {objectsFound} objects detected"
                    : "Processing frames - analyzing video";

                _progress.Report(new ImageProcessingProgress
                {
                    Stage = stage,
                    PercentComplete = percentComplete,
                    CurrentFrame = runWorker.PSM.CurrBlockId,
                    TotalFrames = runWorker.PSM.LastBlockId,
                    ObjectsDetectedSoFar = objectsFound,
                    EstimatedTimeRemaining = EstimateTimeRemaining(runWorker)
                });

                _lastProgressReport = DateTime.Now;
            }
        }

        private static TimeSpan? EstimateTimeRemaining(RunWorker runWorker)
        {
            if (runWorker.PSM == null || runWorker.PSM.CurrBlockId <= 0 || runWorker.PSM.LastBlockId <= 0)
                return null;

            var elapsed = DateTime.Now - _startTime;
            var progress = (double)runWorker.PSM.CurrBlockId / runWorker.PSM.LastBlockId;
            
            if (progress <= 0.01) // Less than 1% complete
                return null;

            var totalEstimated = elapsed.TotalSeconds / progress;
            var remaining = totalEstimated - elapsed.TotalSeconds;
            
            return remaining > 0 ? TimeSpan.FromSeconds(remaining) : TimeSpan.Zero;
        }

        private static readonly DateTime _startTime = DateTime.Now;
    }

    /// <summary>
    /// Wrapper that provides a clean interface to the internal processing results
    /// </summary>
    internal class ImageProcessingResultWrapper : IImageProcessingResult
    {
        private readonly RunWorker _runWorker;
        private readonly DetectionAlgorithm _algorithm;
        private readonly TimeSpan _processingDuration;
        private readonly IDroneData _droneData;
        private bool _disposed = false;

        public ImageProcessingResultWrapper(RunWorker runWorker, DetectionAlgorithm algorithm, TimeSpan processingDuration, IDroneData droneData)
        {
            _runWorker = runWorker ?? throw new ArgumentNullException(nameof(runWorker));
            _algorithm = algorithm;
            _processingDuration = processingDuration;
            _droneData = droneData ?? throw new ArgumentNullException(nameof(droneData));
        }

        public DetectionAlgorithm AlgorithmUsed => _algorithm;

        public int TotalObjectsDetected => _runWorker.ProcessAll?.ProcessObjects?.Count ?? 0;

        public int SignificantObjectsDetected => _runWorker.ProcessAll?.ProcessObjects?.NumSignificantObjects ?? 0;

        public TimeSpan ProcessingDuration => _processingDuration;

        public VideoProcessingInfo VideoInfo => new VideoProcessingInfo
        {
            InputVideoPath = _runWorker.InputVideoFileName(),
            Duration = _droneData.FlightSummary.Duration,
            FramesProcessed = _runWorker.PSM?.CurrBlockId ?? 0,
            FrameRate = GetFrameRate(),
            CameraType = _droneData.FlightSummary.CameraType
        };

        public string? AnnotatedVideoPath 
        {
            get
            {
                if (_runWorker.RunConfig?.ProcessConfig?.SaveAnnotatedVideo != true)
                    return null;

                var inputPath = _runWorker.InputVideoFileName();
                if (string.IsNullOrEmpty(inputPath))
                    return null;

                return $"{Path.ChangeExtension(inputPath, null)}_annotated.mp4";
            }
        }

        public string DataFilePath => _runWorker.DataStore?.DataStoreFileName ?? string.Empty;

        public IReadOnlyCollection<DetectedObject> GetDetectedObjects()
        {
            if (_runWorker.ProcessAll?.ProcessObjects == null)
                return Array.Empty<DetectedObject>();

            return _runWorker.ProcessAll.ProcessObjects.Values
                .Select(ConvertToDetectedObject)
                .ToList()
                .AsReadOnly();
        }

        public IReadOnlyCollection<DetectedObject> GetSignificantObjects()
        {
            if (_runWorker.ProcessAll?.ProcessObjects == null)
                return Array.Empty<DetectedObject>();

            return _runWorker.ProcessAll.ProcessObjects.Values
                .Where(obj => obj.Significant)
                .Select(ConvertToDetectedObject)
                .ToList()
                .AsReadOnly();
        }

        public IReadOnlyCollection<DetectedObject> GetObjectsInLeg(int legId)
        {
            if (_runWorker.ProcessAll?.ProcessObjects == null)
                return Array.Empty<DetectedObject>();

            return _runWorker.ProcessAll.ProcessObjects.Values
                .Where(obj => obj.FlightLegId == legId)
                .Select(ConvertToDetectedObject)
                .ToList()
                .AsReadOnly();
        }

        private float GetFrameRate()
        {
            // Try to get frame rate from drone data or use default
            return (float)(_runWorker.Drone?.InputVideo?.Fps ?? 30.0);
        }

        private DetectedObject ConvertToDetectedObject(ProcessObject processObject)
        {
            return new DetectedObject
            {
                ObjectId = processObject.ObjectId,
                Name = processObject.Name,
                IsSignificant = processObject.Significant,
                LegId = processObject.FlightLegId,
                Location = ConvertLocation(processObject.LocationM),
                HeightAboveGroundM = processObject.HeightM,
                SizeCM2 = processObject.SizeCM2,
                FirstSeenMs = processObject.FirstFeature?.Block?.FlightStep?.SumTimeMs ?? 0,
                LastSeenMs = processObject.LastRealFeature?.Block?.FlightStep?.SumTimeMs ?? 0,
                TrackingDuration = CalculateTrackingDuration(processObject),
                MaxHeatValue = processObject.MaxHeat,
                Category = GetObjectCategory(processObject)
            };
        }

        private GlobalLocation? ConvertLocation(DroneLocation? droneLocation)
        {
            if (droneLocation == null)
                return null;

            // TODO: Implement proper conversion from drone relative coordinates to global coordinates
            // This would require access to the drone's origin point and coordinate system
            return new GlobalLocation(0, 0); // Placeholder
        }

        private static TimeSpan CalculateTrackingDuration(ProcessObject processObject)
        {
            var firstMs = processObject.FirstFeature?.Block?.FlightStep?.SumTimeMs ?? 0;
            var lastMs = processObject.LastRealFeature?.Block?.FlightStep?.SumTimeMs ?? 0;
            
            return TimeSpan.FromMilliseconds(Math.Max(0, lastMs - firstMs));
        }

        private string? GetObjectCategory(ProcessObject processObject)
        {
            // TODO: Implement category retrieval from object annotations
            // This would require access to the category system
            return null;
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _runWorker?.Dispose();
                _disposed = true;
            }
        }
    }
}