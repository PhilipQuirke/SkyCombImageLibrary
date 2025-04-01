// Copyright SkyComb Limited 2024. All rights reserved. 
using OpenCvSharp;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    public enum SaveObjectDataEnum { All, Significant, None };


    // Configuration settings related to processing models.
    //
    // When processing a video for the first time:
    //      - the below member data default values are used, with a few override default values loaded from App.Config
    //      - the member data values are saved to the DataStore (spreadsheet) specific to this video.
    // When processing that same video for a second or subsequent time:
    //      - ALL member data values are loaded from the DataStore specific to this video 
    //      - So if you want to trial different values for the same video, alter the setting in the DATASTORE.
    //      - WARNING: Changing the default values below will have NO effect.
    public class ProcessConfigModel : ConfigBase
    {
        // In legs objects move down the Y axis between frames.
        // When associating objects in successive frames, we sometimes search higher in the image.
        public const int SearchHigherPixels = 20;

        // Single frame Yolo detection confidence
        public const float YoloDetectConfidenceDefault = 0.1f;
        // Successive frame Yolo overlap threshold. Typically 0.2 to 0.4
        public const float YoloIoUDefault = 0.25f;
        // We are clustering features in objects while the drone is flying a leg. We expect very little change in the X values.
        public const int YoloMaxXPixelsDeltaPerCluster = 10;
        // We are clustering features in objects while the drone is flying a leg. We define a maximum Y delta between frames.
        public const int YoloMaxYPixelsDeltaPerFrame = 25;
        // Succcessive features in an object can't be more than 5 frames apart.
        public const int YoloMaxTimeGap = 5;


        // --------------------- Process Techniques --------------------- 

        // Theshold:
        // Pixel gray-scale value that ThresholdProcess uses. Takes values from 50 to 255
        public int HeatThresholdValue { get; set; } = 180;

        // Comb:
        // SkyComb-specific detection method "Comb".
        // Minimum number of hot pixels required inside a bounding rectangle, to draw a red rect in output
        public const int FeatureMinPixels = 8;
        // Maximum feature size (aka length or width) in pixels
        // Sometimes the algorithm generates absurdly large features e.g. 100. This reduces this issue.
        // ToDo: Value should be based on physically realities e.g. the maximum mammal size given standard drone altitude 
        public const int FeatureMaxSize = 100;
        // Minimum overlap percentage between two features that is considered significant
        public const int FeatureMinOverlapPerc = 5;

        // Object characteristics:
        // Duration (in milliseconds) that object must be tracked for before it is highlighted
        public const int ObjectMinDurationMs = 500;
        // Minimum percentage difference between ground speed and object speed for object to be considered above ground.
        public const int ObjectMaxUnrealBlocks = 5;
        // To be highlighted, an object must have this many hot pixels in at least one real step
        public const int ObjectMinPixels = 5;
        // An object detected at long-range must be large and so is not of interest to us.
        public const int ObjectMaxRangeM = 350;


        // YOLO
        // Single frame Yolo detection confidence
        public float YoloDetectConfidence { get; set; } = ProcessConfigModel.YoloDetectConfidenceDefault;
        // Successive frame Yolo overlap threshold
        public float YoloIoU { get; set; } = ProcessConfigModel.YoloIoUDefault; // Typically 0.2 to 0.4



        // --------------------- Saving Output --------------------- 
        // Create annotated video file as output MP4
        public bool SaveAnnotatedVideo { get; set; } = false;
        // Save the objects and features to the datastore. Takes values all, significant, none.
        public SaveObjectDataEnum SaveObjectData { get; set; } = SaveObjectDataEnum.Significant;


        // --------------------- Error Thresholds --------------------- 
        // The maximum +/- inaccuracy (in meters) in an object's estimated location to be considered "good" & colored green
        public const float GoodLocationErrM = 5;
        // The maximum +/- inaccuracy (in meters) in an object's estimated height to be considered "good" & colored green 
        public const float GoodHeightErrM = 5;
        // If height inaccuracy is too great then height calculation is abandoned.
        public const float AbandonHeightErrM = 5;


        // --------------------- Processing Limits --------------------- 

        // The ThresholdValue should be in range 50 to 255.
        // A user may mistake the range for 0.0 to 1.0, so we set the min value to 50
        public void ValidateHeatThresholdValue()
        {
            if (HeatThresholdValue < 50)
                HeatThresholdValue = 50;

            if (HeatThresholdValue > 255)
                HeatThresholdValue = 255;
        }


        public void ValidateYolo()
        {
            if ((YoloDetectConfidence < 0.1f) || (YoloDetectConfidence > 0.9f))
                YoloDetectConfidence = ProcessConfigModel.YoloDetectConfidenceDefault;

            if ((YoloIoU < 0.1f) || (YoloIoU > 0.9f))
                YoloIoU = ProcessConfigModel.YoloIoUDefault;
        }


        // Horizontal field of view in degrees. Differs per manufacturer's camera.
        public void SetCameraSpecifics(string cameraType)
        {
            switch (cameraType)
            {
                case VideoModel.DjiH20N:
                    HeatThresholdValue = 180;
                    break;

                case VideoModel.DjiH20T:
                    break;

                case VideoModel.DjiM3T:
                    HeatThresholdValue = 235;
                    break;
            }
        }


        // Get the class's settings as datapairs (e.g. for saving to a spreadsheet)
        public DataPairList GetModelSettings()
        {
            return new DataPairList
            {
                { "Threshold Value", HeatThresholdValue },
                { "Feature Min Pixels", FeatureMinPixels },
                { "Feature Max Size", FeatureMaxSize },
                { "Feature Min Overlap Perc", FeatureMinOverlapPerc },
                { "Object Min Duration Ms", ObjectMinDurationMs },
                { "Object Max Unreal Blocks", ObjectMaxUnrealBlocks },
                { "Object Min Pixels", ObjectMinPixels },
                { "Object Max Range M", ObjectMaxRangeM },
                { "Yolo Confidence", YoloDetectConfidence, 2 },
                { "Yolo IoU", YoloIoU, 2 },
            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadModelSettings(List<string> settings)
        {
            int i = 0;
            HeatThresholdValue = StringToNonNegInt(settings[i++]);
            i++; // FeatureMinPixels  
            i++; // FeatureMaxSize  
            i++; // FeatureMinOverlapPerc  
            i++; // ObjectMinDurationMs  
            i++; // ObjectMaxUnrealBlocks  
            i++; // ObjectMinPixels  
            i++; // ObjectMaxRangeM  
            YoloDetectConfidence = StringToFloat(settings[i++]);
            YoloIoU = StringToFloat(settings[i++]);
        }


        // Get the class's settings as datapairs (e.g. for saving to a spreadsheet)
        public DataPairList GetOutputSettings()
        {
            return new DataPairList
            {
                { "Save Annotated Video", SaveAnnotatedVideo },
                { "Save Object Data", SaveObjectData.ToString() },

            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadOutputSettings(List<string> settings)
        {
            int i = 0;
            SaveAnnotatedVideo = Convert.ToBoolean(settings[i++]);
            SaveObjectData = (SaveObjectDataEnum)Enum.Parse(typeof(SaveObjectDataEnum), settings[i++]);
        }


        public static string AllSomeOrNone(string value)
        {
            string answer = CleanString(value);

            return answer switch
            {
                "all" or "significant" or "none" => answer,
                "" => "none",
                _ => throw ThrowException("ModelConfig.AllSomeOrNone: Bad value: " + answer),
            };
        }

        // Hardcode the drone camera intrinsic matrix for Lennard Sparks drone camera
//        public static CameraIntrinsic intrinsic = new CameraIntrinsic(9.1, 640, 512, 7.68, 6.144);
        public static CameraIntrinsic intrinsic = new CameraIntrinsic(40, 1280, 1024, 640, 512);
        public static Mat K = intrinsic.K;
    }
    public class CameraIntrinsic
    {
        public Mat K = new Mat(3, 3, MatType.CV_64F);
        public Mat KInv = new Mat(3, 3, MatType.CV_64F);
        public double Cx;
        public double Cy;
        public double Fx;
        public double Fy;
        public double ImageWidth;
        public double ImageHeight;

        public CameraIntrinsic(double focalLength, double imageWidth, double imageHeight, double sensorWidth, double sensorHeight)
        {
            ImageWidth = imageWidth;
            ImageHeight = imageHeight;
            Cx = imageWidth / 2; 
            Cy = imageHeight / 2;
            Fx = focalLength * imageWidth / sensorWidth; // F * pixels per mm = focal length in mm x image width px / sensor width mm
            Fy = focalLength * imageHeight / sensorHeight;
            K.At<double>(0, 0) = Fx;
            K.At<double>(0, 1) = 0;
            K.At<double>(0, 2) = Cx;
            K.At<double>(1, 0) = 0;
            K.At<double>(1, 1) = Fy;
            K.At<double>(1, 2) = Cy;
            K.At<double>(2, 0) = 0;
            K.At<double>(2, 1) = 0;
            K.At<double>(2, 2) = 1;
            KInv = K.Inv();
        }

    }
}