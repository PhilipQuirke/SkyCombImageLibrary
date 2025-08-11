// Copyright SkyComb Limited 2025. All rights reserved.

namespace SkyCombImage.Exceptions
{
    /// <summary>
    /// Base exception for all SkyComb Image processing errors
    /// </summary>
    public abstract class ImageProcessingException : Exception
    {
        protected ImageProcessingException(string message) : base(message) { }
        protected ImageProcessingException(string message, Exception innerException) : base(message, innerException) { }
    }

    /// <summary>
    /// Exception thrown when YOLO model files cannot be found or loaded
    /// </summary>
    public class YoloModelNotFoundException : ImageProcessingException
    {
        /// <summary>
        /// Gets the path where YOLO model files were expected
        /// </summary>
        public string ModelPath { get; }

        public YoloModelNotFoundException(string modelPath) 
            : base($"YOLO model files not found at: {modelPath}")
        {
            ModelPath = modelPath;
        }

        public YoloModelNotFoundException(string modelPath, Exception innerException) 
            : base($"YOLO model files not found at: {modelPath}", innerException)
        {
            ModelPath = modelPath;
        }
    }

    /// <summary>
    /// Exception thrown when GPU/CUDA requirements are not met for YOLO processing
    /// </summary>
    public class GpuRequirementsException : ImageProcessingException
    {
        public GpuRequirementsException(string message) : base(message) { }
        public GpuRequirementsException(string message, Exception innerException) : base(message, innerException) { }
    }

    /// <summary>
    /// Exception thrown when video data is missing or invalid for image processing
    /// </summary>
    public class InvalidVideoDataException : ImageProcessingException
    {
        public InvalidVideoDataException(string message) : base(message) { }
        public InvalidVideoDataException(string message, Exception innerException) : base(message, innerException) { }
    }

    /// <summary>
    /// Exception thrown when processing configuration is invalid
    /// </summary>
    public class InvalidProcessingConfigurationException : ImageProcessingException
    {
        public InvalidProcessingConfigurationException(string message) : base(message) { }
        public InvalidProcessingConfigurationException(string message, Exception innerException) : base(message, innerException) { }
    }

    /// <summary>
    /// Exception thrown when object detection processing fails
    /// </summary>
    public class ObjectDetectionException : ImageProcessingException
    {
        /// <summary>
        /// Gets the frame number where the error occurred (if applicable)
        /// </summary>
        public int? FrameNumber { get; }

        public ObjectDetectionException(string message) : base(message) 
        { 
            FrameNumber = null;
        }

        public ObjectDetectionException(string message, int frameNumber) : base(message) 
        { 
            FrameNumber = frameNumber;
        }

        public ObjectDetectionException(string message, Exception innerException) : base(message, innerException) 
        { 
            FrameNumber = null;
        }

        public ObjectDetectionException(string message, int frameNumber, Exception innerException) : base(message, innerException) 
        { 
            FrameNumber = frameNumber;
        }
    }

    /// <summary>
    /// Exception thrown when insufficient memory is available for processing
    /// </summary>
    public class InsufficientMemoryException : ImageProcessingException
    {
        public InsufficientMemoryException(string message) : base(message) { }
        public InsufficientMemoryException(string message, Exception innerException) : base(message, innerException) { }
    }
}