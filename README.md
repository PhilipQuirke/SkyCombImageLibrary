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


## GPU and YOLO frameworks
For speed, the code assumes the existance of a GPU it can to process the YOLO v8 model calculations.
The setup of a PC with a GPU includes manually installing GPU-related frameworks:
- [CUDA](https://developer.nvidia.com/cuda-downloads)
- [cuDNN](https://developer.nvidia.com/cudnn)
- 
Adding to the Windows PATH variable entries like:
- C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin
- C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\extras\CUPTI\lib64
- C:\Program Files\NVIDIA\CUDNN\v9.4\bin\12.6

This project imports these libraries related to GPU and YOLO computing:
- Accord.MachineLearning
- Compunet.YoloV8
- Microsoft.ML.OnnxRuntime.Gpu
- Microsoft.ML.OnnxRuntime.Gpu.Windows
- SixLabors.ImageSharp
