# SkyCombImageLibrary

SkyComb Image Library is a library that:
- takes as input the (thermal and/or optical) video files created by a drone during a flight
- leverage data calculated by the SkyComb Ground and Drone libraries
- processes one video & detects interesting objects 
- outputs an mp4 annotated video & updates the datastore (xlsx) with details of the interesting objects  

This "image processing" library is incorporated into the tools:
- [SkyComb Analyst](https://github.com/PhilipQuirke/SkyCombAnalyst/) 
- [SkyComb Flights](https://github.com/PhilipQuirke/SkyCombFlights/) 


## Code
The code folders are:
- **CategorySpace:** Object categories that are manually and automatically applied to objects as labels  
- **ProcessModel:** In-memory representations (models) of objects found
- **ProcessLogic:** Logic on how to process images to detect objects
- **DrawSpace:** Code to annotate videos & annotate graphs with interesting objects
- **PersistModel:** Save/load data from/to the datastore (spreadsheet) including graphs
- **RunSpace:** Code to run the image processing on videos


## Processing using GPU and YOLO frameworks
SkyComb Analyst and Flights use the "You Only Look Once" (YOLO) v8 algorithm to detect objects in the video frames.
This algorithm utilises a GPU to process the calculations quickly.

SkyComb Analyst and Flights can only process input data after the YOLO / GPU set up is complete.
SkyComb Analyst can be used to view pre-processed input data without doing the set up.

Given a Windows PC or laptop that has a GPU, the SkyComb "processing" set up process is:

Manually download and install 2 GPU-related toolkits from:
- [CUDA](https://developer.nvidia.com/cuda-downloads) for "CUDA Toolkit 12.6" 
- [cuDNN](https://developer.nvidia.com/cudnn) for "cuDNN (NVIDIA CUDA Deep Neural Network) Library for Windows 10"

Add to the Windows PATH variable entries like:
- C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin
- C:\Program Files\NVIDIA\CUDNN\v9.4\bin\12.6

Add to the Windows System variables entries like:
- CUDA_HOME C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
- CUDA_PATH C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
- CUDA_PATH_V12_6 C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
- CUDA_VISIBLE_DEVICES 0


## Key Dependencies
This project uses these main libraries:
- **YoloDotNet (2.2.0)** - YOLO v8 object detection framework
- **SkiaSharp (2.88.8)** - Cross-platform 2D graphics API for image processing
- **OpenCvSharp4 (4.10.0.*)** - Computer vision and image processing operations
- **Accord.MachineLearning (3.8.0)** - Mathematical optimization algorithms
- **alglib404gpl_net5.dll** - Advanced mathematical computation library