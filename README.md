# SkyCombImageLibrary


SkyComb Image Library is a library that:
- takes as input the (thermal and/or optical) video files created by a drone during a flight
- leverage data calculated by the SkyComb Ground and Drone libraries
- processes one video & detects interesting objects 
- outputs an mp4 annotated video & updates the datastore (xlsx) with details of the interesting objects  

This "image prcecssing" library is incorporated into the tools:
- [SkyComb Analyst](https://github.com/PhilipQuirke/SkyCombAnalyst/) 

The code folders are:
- **CategorySpace:** Object categories that are manually and automatically applied to objects as labels  
- **ProcessModel:** In-memory representations (models) of objects found
- **ProcessLogic:** Logic on how to process images to detect objects
- **DrawSpace:** Code to annotate videos & annotate graphs with interesting objects
- **PersistModel:** Save/load data from/to the datastore (spreadsheet) including graphs
- **RunSpace:** Code to run the image processing on videos
