// Copyright SkyComb Limited 2025. All rights reserved.
using SkyCombGround.CommonSpace;
using SkyCombDrone.Interfaces;

namespace SkyCombImage.Interfaces
{
    /// <summary>
    /// Defines the types of object detection algorithms available
    /// </summary>
    public enum DetectionAlgorithm
    {
        /// <summary>
        /// SkyComb-specific Comb detection method for thermal imagery
        /// </summary>
        Comb,
        
        /// <summary>
        /// YOLO v8 deep learning object detection
        /// </summary>
        Yolo,
        
        /// <summary>
        /// Simple threshold-based detection
        /// </summary>
        Threshold
    }

    /// <summary>
    /// Configuration options for image processing
    /// </summary>
    public class ImageProcessingOptions
    {
        /// <summary>
        /// Gets or sets the object detection algorithm to use
        /// </summary>
        public DetectionAlgorithm Algorithm { get; set; } = DetectionAlgorithm.Comb;

        /// <summary>
        /// Gets or sets the heat threshold value (0-255) for thermal detection
        /// </summary>
        public int HeatThreshold { get; set; } = 235;

        /// <summary>
        /// Gets or sets the confidence threshold for YOLO detection (0.0-1.0)
        /// </summary>
        public float YoloConfidence { get; set; } = 0.1f;

        /// <summary>
        /// Gets or sets the IoU threshold for YOLO object tracking (0.0-1.0)
        /// </summary>
        public float YoloIoU { get; set; } = 0.25f;

        /// <summary>
        /// Gets or sets whether to save the annotated output video
        /// </summary>
        public bool SaveAnnotatedVideo { get; set; } = false;

        /// <summary>
        /// Gets or sets what object data to save
        /// </summary>
        public ObjectDataSaveMode SaveObjectData { get; set; } = ObjectDataSaveMode.Significant;

        /// <summary>
        /// Gets or sets the directory containing YOLO model files (required for YOLO algorithm)
        /// </summary>
        public string? YoloModelDirectory { get; set; }
    }

    /// <summary>
    /// Defines what object data should be saved
    /// </summary>
    public enum ObjectDataSaveMode
    {
        /// <summary>
        /// Don't save any object data
        /// </summary>
        None,
        
        /// <summary>
        /// Save only significant/important objects
        /// </summary>
        Significant,
        
        /// <summary>
        /// Save all detected objects
        /// </summary>
        All
    }

    /// <summary>
    /// Represents the results of image processing on drone data
    /// </summary>
    public interface IImageProcessingResult : IDisposable
    {
        /// <summary>
        /// Gets the detection algorithm that was used
        /// </summary>
        DetectionAlgorithm AlgorithmUsed { get; }

        /// <summary>
        /// Gets the total number of objects detected
        /// </summary>
        int TotalObjectsDetected { get; }

        /// <summary>
        /// Gets the number of significant objects detected
        /// </summary>
        int SignificantObjectsDetected { get; }

        /// <summary>
        /// Gets the processing duration
        /// </summary>
        TimeSpan ProcessingDuration { get; }

        /// <summary>
        /// Gets information about the video that was processed
        /// </summary>
        VideoProcessingInfo VideoInfo { get; }

        /// <summary>
        /// Gets all detected objects
        /// </summary>
        /// <returns>Collection of detected objects</returns>
        IReadOnlyCollection<DetectedObject> GetDetectedObjects();

        /// <summary>
        /// Gets significant detected objects only
        /// </summary>
        /// <returns>Collection of significant objects</returns>
        IReadOnlyCollection<DetectedObject> GetSignificantObjects();

        /// <summary>
        /// Gets objects detected in a specific flight leg
        /// </summary>
        /// <param name="legId">The flight leg identifier</param>
        /// <returns>Collection of objects in the specified leg</returns>
        IReadOnlyCollection<DetectedObject> GetObjectsInLeg(int legId);

        /// <summary>
        /// Gets the path to the annotated output video (if created)
        /// </summary>
        string? AnnotatedVideoPath { get; }

        /// <summary>
        /// Gets the path to the Excel data file containing detailed results
        /// </summary>
        string DataFilePath { get; }
    }

    /// <summary>
    /// Information about the video that was processed
    /// </summary>
    public class VideoProcessingInfo
    {
        /// <summary>
        /// Gets or sets the input video file path
        /// </summary>
        public string InputVideoPath { get; set; } = string.Empty;

        /// <summary>
        /// Gets or sets the video duration
        /// </summary>
        public TimeSpan Duration { get; set; }

        /// <summary>
        /// Gets or sets the number of frames processed
        /// </summary>
        public int FramesProcessed { get; set; }

        /// <summary>
        /// Gets or sets the video frame rate
        /// </summary>
        public float FrameRate { get; set; }

        /// <summary>
        /// Gets or sets the camera type detected
        /// </summary>
        public string CameraType { get; set; } = string.Empty;
    }

    /// <summary>
    /// Represents a detected object in the processed imagery
    /// </summary>
    public class DetectedObject
    {
        /// <summary>
        /// Gets or sets the unique object identifier
        /// </summary>
        public int ObjectId { get; set; }

        /// <summary>
        /// Gets or sets the object name (e.g., "A1", "B3")
        /// </summary>
        public string Name { get; set; } = string.Empty;

        /// <summary>
        /// Gets or sets whether this object is considered significant
        /// </summary>
        public bool IsSignificant { get; set; }

        /// <summary>
        /// Gets or sets the flight leg where this object was detected
        /// </summary>
        public int LegId { get; set; }

        /// <summary>
        /// Gets or sets the geographical location of the object
        /// </summary>
        public GlobalLocation? Location { get; set; }

        /// <summary>
        /// Gets or sets the object's height above ground in meters
        /// </summary>
        public float HeightAboveGroundM { get; set; }

        /// <summary>
        /// Gets or sets the estimated size of the object in square centimeters
        /// </summary>
        public float SizeCM2 { get; set; }

        /// <summary>
        /// Gets or sets the first timestamp where the object was detected
        /// </summary>
        public int FirstSeenMs { get; set; }

        /// <summary>
        /// Gets or sets the last timestamp where the object was detected
        /// </summary>
        public int LastSeenMs { get; set; }

        /// <summary>
        /// Gets or sets the duration for which the object was tracked
        /// </summary>
        public TimeSpan TrackingDuration { get; set; }

        /// <summary>
        /// Gets or sets the maximum heat/temperature value detected for this object
        /// </summary>
        public float MaxHeatValue { get; set; }

        /// <summary>
        /// Gets or sets the object category (if manually classified)
        /// </summary>
        public string? Category { get; set; }
    }

    /// <summary>
    /// Service for processing drone imagery to detect objects
    /// </summary>
    public interface IImageProcessingService
    {
        /// <summary>
        /// Processes drone video data to detect objects
        /// </summary>
        /// <param name="droneData">The drone flight data containing video</param>
        /// <param name="options">Processing configuration options</param>
        /// <param name="cancellationToken">Cancellation token</param>
        /// <returns>Processing results containing detected objects</returns>
        /// <exception cref="ArgumentNullException">Thrown when droneData or options is null</exception>
        /// <exception cref="InvalidOperationException">Thrown when drone data doesn't contain video</exception>
        /// <exception cref="YoloModelNotFoundException">Thrown when YOLO model files are not found</exception>
        Task<IImageProcessingResult> ProcessVideoAsync(IDroneData droneData, ImageProcessingOptions options, CancellationToken cancellationToken = default);

        /// <summary>
        /// Processes drone video data with progress reporting
        /// </summary>
        /// <param name="droneData">The drone flight data containing video</param>
        /// <param name="options">Processing configuration options</param>
        /// <param name="progress">Progress reporting callback</param>
        /// <param name="cancellationToken">Cancellation token</param>
        /// <returns>Processing results containing detected objects</returns>
        Task<IImageProcessingResult> ProcessVideoAsync(IDroneData droneData, ImageProcessingOptions options, IProgress<ImageProcessingProgress>? progress, CancellationToken cancellationToken = default);

        /// <summary>
        /// Gets a quick preview of what would be detected without full processing
        /// </summary>
        /// <param name="droneData">The drone flight data containing video</param>
        /// <param name="options">Processing configuration options</param>
        /// <param name="maxFramesToSample">Maximum number of frames to sample for preview</param>
        /// <param name="cancellationToken">Cancellation token</param>
        /// <returns>Preview results</returns>
        Task<ImageProcessingPreview> GetProcessingPreviewAsync(IDroneData droneData, ImageProcessingOptions options, int maxFramesToSample = 10, CancellationToken cancellationToken = default);
    }

    /// <summary>
    /// Progress information for image processing operations
    /// </summary>
    public class ImageProcessingProgress
    {
        /// <summary>
        /// Gets or sets the current processing stage
        /// </summary>
        public string Stage { get; set; } = string.Empty;

        /// <summary>
        /// Gets or sets the percentage complete (0-100)
        /// </summary>
        public int PercentComplete { get; set; }

        /// <summary>
        /// Gets or sets the current frame being processed
        /// </summary>
        public int CurrentFrame { get; set; }

        /// <summary>
        /// Gets or sets the total number of frames to process
        /// </summary>
        public int TotalFrames { get; set; }

        /// <summary>
        /// Gets or sets the number of objects detected so far
        /// </summary>
        public int ObjectsDetectedSoFar { get; set; }

        /// <summary>
        /// Gets or sets the estimated time remaining
        /// </summary>
        public TimeSpan? EstimatedTimeRemaining { get; set; }
    }

    /// <summary>
    /// Preview information about what would be detected
    /// </summary>
    public class ImageProcessingPreview
    {
        /// <summary>
        /// Gets or sets the estimated number of objects that would be detected
        /// </summary>
        public int EstimatedObjectCount { get; set; }

        /// <summary>
        /// Gets or sets the estimated processing duration
        /// </summary>
        public TimeSpan EstimatedProcessingTime { get; set; }

        /// <summary>
        /// Gets or sets sample detections from the preview frames
        /// </summary>
        public IReadOnlyCollection<DetectedObject> SampleDetections { get; set; } = new List<DetectedObject>();

        /// <summary>
        /// Gets or sets the number of frames that were sampled
        /// </summary>
        public int FramesSampled { get; set; }
    }
}