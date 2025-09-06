# SkyCombImageLibrary

SkyComb Image Library is a modern .NET library for processing drone thermal video to detect and track objects of interest. It provides a clean, user-friendly API for integrating computer vision capabilities into drone analysis applications.

## Features

- **Multiple Detection Algorithms** - Comb (thermal-optimized), YOLO v8 (deep learning), and threshold-based detection
- **Drone Integration** - Seamless integration with SkyComb Drone Library for flight data
- **Thermal Processing** - Specialized algorithms for thermal imagery analysis
- **YOLO Support** - GPU-accelerated YOLO v8 object detection on hardware with CUDA support
- **Rich Results** - Detailed object information including location, size, duration, and confidence
- **Image and Videos** - Per run processes either a folder of images or a single video file
- **Video Annotation** - When processing vidoes, generates annotated videos showing detected objects
- **Progress Reporting** - Real-time progress updates during processing
- **Extensible Design** - Clean interfaces for custom algorithm implementations

## Quick Start

### Installation

```bash
# Add project reference
dotnet add reference ../SkyCombImageLibrary/SkyCombImageLibrary.csproj

# Or add package reference (when published)
dotnet add package SkyCombImageLibrary
```

### Basic Usage

```csharp
using SkyCombImage.Interfaces;
using SkyCombImage.Services;
using SkyCombDrone.Services;

// Step 1: Load drone data (video + flight log)
var droneService = DroneDataService.Create();
using var droneData = await droneService.LoadVideoDataAsync(
    @"C:\DroneVideos\thermal_flight.mp4", 
    @"C:\GroundData"
);

// Step 2: Configure image processing
var imageService = ImageProcessingService.Create();
var options = new ImageProcessingOptions
{
    Algorithm = DetectionAlgorithm.Comb,
    HeatThreshold = 235,
    SaveObjectData = ObjectDataSaveMode.Significant
};

// Step 3: Process with progress reporting
var progress = new Progress<ImageProcessingProgress>(p =>
    Console.WriteLine($"{p.PercentComplete}% - {p.Stage} - {p.ObjectsDetectedSoFar} objects"));

using var result = await imageService.ProcessVideoAsync(droneData, options, progress);

// Step 4: Analyze results
Console.WriteLine($"Found {result.SignificantObjectsDetected} significant objects");
Console.WriteLine($"Processing took: {result.ProcessingDuration:hh\\:mm\\:ss}");

foreach (var obj in result.GetSignificantObjects().Take(5))
{
    Console.WriteLine($"Object {obj.Name}: {obj.SizeCM2:F0}cm², {obj.HeightAboveGroundM:F1}m high");
}
```

### YOLO Detection Example

```csharp
var options = new ImageProcessingOptions
{
    Algorithm = DetectionAlgorithm.Yolo,
    YoloConfidence = 0.25f,
    YoloIoU = 0.45f,
    YoloModelDirectory = @"C:\YoloModels",
    SaveAnnotatedVideo = true
};

// Get preview first
var preview = await imageService.GetProcessingPreviewAsync(droneData, options);
Console.WriteLine($"Estimated: {preview.EstimatedObjectCount} objects, {preview.EstimatedProcessingTime:hh\\:mm\\:ss} processing time");

// Process with full detection
using var result = await imageService.ProcessVideoAsync(droneData, options, progress);
```

## API Reference

### Core Interfaces

- **`IImageProcessingService`** - Main service for object detection
- **`IImageProcessingResult`** - Access to processing results and detected objects
- **`ImageProcessingOptions`** - Configuration for detection algorithms
- **`DetectedObject`** - Information about individual detected objects

### Detection Algorithms
SkyComb Image Library supports three detection algorithms:

**Threshold**
The **threshold** algorithm simply covert all pixels below a certain threshold (HeatThresholdValue, e.g. 235) to black (i.e. 0).
There is a default threshold, this can be manually adjusted per input set to better segment the animals from the background. 

Background: Thermal images are grey-scale with pixels in the range 0 to 255.
The interpretation of 0 to 255 depends on the temperature range chosen (out of 3 say 3 predefined options) in the drone and to a lesser degree the environment (e.g. with too much light a thermal can be swamped and become inaccurate).  
For images taken during the day (not recommended) rocks can be the hottest objects.
For images taken at night, after rocks have had time to cool, animals are the generally the hotest objects.
But a range of trees, rocks, sap, water may score say 0 to 100 on the 0-255 scale.

**YOLO**
The **YOLO** (You Only Look Once) algorithm is an AI-based, state-of-the-art object detection system. 
Starting with images of known objects (e.g. possums and birds) manually annotated by humans, a neural network is trained to recognize those objects in new images.
The model is best at detecting those objects it has been trained on, but can also detect other objects with similar attributes (e.g. size, heat, shape).
One such model is available for free with this library, trained on possums and birds in thermal imagery.
To run this algorithm, your laptop must have a CUDA-capable GPU chip and the YOLO model stored on your hard-disk.

**Comb**
The **Comb** algorithm is a custom object detection algorithm optimized for thermal imagery.
It combines the thresholding approach with "hot pixel" clustering techniques, to detect small hot objects in thermal images.

### Processing Options

```csharp
var options = new ImageProcessingOptions
{
    Algorithm = DetectionAlgorithm.Comb,       // Detection method
    HeatThreshold = 235,                       // Thermal threshold (0-255)
    YoloConfidence = 0.25f,                    // YOLO confidence (0.0-1.0)
    YoloIoU = 0.45f,                           // YOLO IoU threshold
    SaveAnnotatedVideo = true,                 // Generate annotated MP4
    SaveObjectData = ObjectDataSaveMode.Significant, // What to save
    YoloModelDirectory = @"C:\YoloModels"      // YOLO model path
};
```

## GPU and YOLO Setup

For YOLO processing with GPU acceleration:

1. **Install CUDA Toolkit** - [CUDA 12.6](https://developer.nvidia.com/cuda-downloads)
2. **Install cuDNN** - [cuDNN for Windows](https://developer.nvidia.com/cudnn)
3. **Update PATH variables:**
   ```
   C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin
   C:\Program Files\NVIDIA\CUDNN\v9.4\bin\12.6
   ```
4. **Set environment variables:**
   ```
   CUDA_HOME=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
   CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
   CUDA_VISIBLE_DEVICES=0
   ```

## Error Handling

The library provides specific exception types:

```csharp
try
{
    using var result = await imageService.ProcessVideoAsync(droneData, options);
}
catch (YoloModelNotFoundException ex)
{
    Console.WriteLine($"YOLO models not found: {ex.ModelPath}");
}
catch (GpuRequirementsException ex)
{
    Console.WriteLine($"GPU setup issue: {ex.Message}");
}
catch (InvalidVideoDataException ex)
{
    Console.WriteLine($"Video data problem: {ex.Message}");
}
catch (ObjectDetectionException ex)
{
    Console.WriteLine($"Detection failed: {ex.Message}");
    if (ex.FrameNumber.HasValue)
        Console.WriteLine($"Error at frame: {ex.FrameNumber}");
}
```

## Advanced Usage

### Batch Processing

```csharp
var videoPaths = Directory.GetFiles(@"C:\DroneVideos", "*.mp4");

foreach (var videoPath in videoPaths)
{
    using var droneData = await droneService.LoadVideoDataAsync(videoPath, groundDataPath);
    using var result = await imageService.ProcessVideoAsync(droneData, options);
    
    Console.WriteLine($"{Path.GetFileName(videoPath)}: {result.SignificantObjectsDetected} objects");
}
```

### Custom Progress Reporting

```csharp
var detailedProgress = new Progress<ImageProcessingProgress>(progress =>
{
    Console.WriteLine($"[{DateTime.Now:HH:mm:ss}] {progress.Stage}");
    Console.WriteLine($"  {progress.PercentComplete}% ({progress.CurrentFrame}/{progress.TotalFrames})");
    Console.WriteLine($"  Objects: {progress.ObjectsDetectedSoFar}");
    
    if (progress.EstimatedTimeRemaining.HasValue)
        Console.WriteLine($"  ETA: {progress.EstimatedTimeRemaining:hh\\:mm\\:ss}");
});
```

### Object Analysis

```csharp
var significantObjects = result.GetSignificantObjects();

// Group by size
var sizeGroups = significantObjects.GroupBy(obj => obj.SizeCM2 switch
{
    < 100 => "Small",
    < 500 => "Medium", 
    < 1000 => "Large",
    _ => "Very Large"
});

foreach (var group in sizeGroups)
{
    Console.WriteLine($"{group.Key}: {group.Count()} objects");
}

// Filter by flight leg
if (droneData.HasFlightLegs)
{
    foreach (var leg in droneData.GetFlightLegs())
    {
        var legObjects = result.GetObjectsInLeg(leg.LegId);
        Console.WriteLine($"Leg {leg.LegName}: {legObjects.Count} objects");
    }
}
```

## Key Dependencies

- **YoloDotNet (2.2.0)** - YOLO v8 object detection framework
- **SkiaSharp (2.88.8)** - Cross-platform 2D graphics API for image processing  
- **OpenCvSharp4 (4.10.0.*)** - Computer vision and image processing operations
- **Accord.MachineLearning (3.8.0)** - Mathematical optimization algorithms
- **ALGLIB** - Advanced mathematical computation library (native DLL)

## Examples

See the [Examples](Examples/) directory for usage examples.

## Architecture

```
SkyCombImageLibrary/
    Interfaces/        # Public API contracts
    Services/          # Service implementations  
    Examples/          # Usage examples and documentation
    Exceptions/        # Custom exception types
    src/
        ProcessLogic/  # Object detection algorithms
        ProcessModel/  # Data models for detected objects
        PersistModel/  # Data persistence and export
        DrawSpace/     # Video annotation and visualization
        RunSpace/      # Processing workflow coordination
        CategorySpace/ # Object classification system
```

## Performance Considerations

- **Memory Usage**: Large videos require substantial RAM (2-4GB recommended)
- **Processing Speed**: YOLO ~0.8x video duration, Comb ~0.4x, Threshold ~0.2x
- **GPU Requirements**: YOLO requires CUDA-compatible GPU for optimal performance
- **Storage**: Annotated videos and Excel data files are generated during processing

## Related Projects

Part of the SkyComb ecosystem:

- **[SkyComb Analyst](../../SkyCombAnalyst/)** - Complete drone thermal analysis application
- **[SkyComb Flights](../../SkyCombFlights/)** - Batch drone data processing  
- **[SkyComb Drone Library](../../SkyCombDroneLibrary/)** - Drone flight data processing
- **[SkyComb Ground Library](../../SkyCombGroundLibrary/)** - Ground elevation data processing

## License

MIT License - see [LICENSE](LICENSE) for details.

## Support

- **Documentation**: See [Examples](Examples/) directory
- **Issues**: Report via GitHub Issues
- **Contact**: Through GitHub

---

**Note**: This library processes drone video files with associated flight logs. GPU setup is required for optimal YOLO performance. Camera-specific calibration may be needed for new drone models.