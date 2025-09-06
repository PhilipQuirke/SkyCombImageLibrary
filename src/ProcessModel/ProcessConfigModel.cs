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

        // SkyComb-specific detection method "Comb".
        // Minimum number of hot pixels required inside a bounding rectangle, to draw a red rect in output
        public const int FeatureMinPixels = 8;
        // Maximum feature size (aka length or width) in pixels
        // Sometimes the algorithm generates absurdly large features e.g. 100. This reduces this issue.
        // ToDo: Value should be based on physically realities e.g. the maximum mammal size given standard drone altitude 
        public const int FeatureMaxSize = 100;
        // Minimum overlap percentage between two features that is considered significant
        public const int FeatureMinOverlapPerc = 5;

        // Duration (in milliseconds) that object must be tracked for before it is highlighted
        public const int ObjectMinDurationMs = 500;
        // Maximum number of "unreal" features after a real feature. Applies to videos only.
        public const int ObjectMaxUnrealBlocks = 5;
        // To be significant, an object must have this many hot pixels in at least one real step
        public const int ObjectMinPixels = 5;
        // An object detected at long-range must be large and so is not of interest to us.
        public const int ObjectMaxRangeM = 350;
        // Minimum fraction of pixels in a feature that must be "hot" to be considered significant
        public const float ObjectMinHotDensity = 0.1f;

        // Pixel gray-scale value for hot pixel thresholding. Takes values from 50 to 255
        public int HeatThresholdValue { get; set; } = 220;
        // Single frame Yolo detection confidence
        public float YoloDetectConfidence { get; set; } = ProcessConfigModel.YoloDetectConfidenceDefault;
        // Successive frame Yolo overlap threshold
        public float YoloIoU { get; set; } = ProcessConfigModel.YoloIoUDefault; // Typically 0.2 to 0.4

        // --------------------- Image Exclusion Zones --------------------- 
        // Exclude bottom right corner from processing (used when image includes text overlays e.g. location, altitude)
        public bool ExcludeBottomRightCorner { get; set; } = true;
        // Width of exclusion zone from right edge (in pixels if > 1 else as percentage)
        public float ExclusionZoneRightWidth { get; set; } = 0.4f;
        // Height of exclusion zone from bottom edge  (in pixels if > 1 else as percentage)
        public float ExclusionZoneBottomHeight { get; set; } = 0.1f;


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

        // Calculate exclusion zone boundaries based on image dimensions
        public (int rightBoundary, int bottomBoundary) GetExclusionBoundaries(int imageWidth, int imageHeight)
        {
            if (!ExcludeBottomRightCorner)
                return (imageWidth, imageHeight);

            // Calculate right boundary (exclude from right edge)
            int excludeWidth = ExclusionZoneRightWidth <= 1.0f 
                ? (int)(imageWidth * ExclusionZoneRightWidth) 
                : (int)ExclusionZoneRightWidth;
            int rightBoundary = Math.Max(0, imageWidth - excludeWidth);

            // Calculate bottom boundary (exclude from bottom edge)
            int excludeHeight = ExclusionZoneBottomHeight <= 1.0f 
                ? (int)(imageHeight * ExclusionZoneBottomHeight) 
                : (int)ExclusionZoneBottomHeight;
            int bottomBoundary = Math.Max(0, imageHeight - excludeHeight);

            return (rightBoundary, bottomBoundary);
        }

        // Check if a pixel coordinate should be processed (not in exclusion zone)
        public bool ShouldProcessPixel(int x, int y, int imageWidth, int imageHeight)
        {
            if (!ExcludeBottomRightCorner)
                return true;

            var (rightBoundary, bottomBoundary) = GetExclusionBoundaries(imageWidth, imageHeight);
            
            // Exclude only the bottom-right corner (intersection of right area AND bottom area)
            // Return FALSE only if pixel is BOTH in right area AND bottom area
            bool inRightArea = x >= rightBoundary;
            bool inBottomArea = y >= bottomBoundary;
            bool inExclusionZone = inRightArea && inBottomArea;
            
            return !inExclusionZone; // Process all pixels EXCEPT those in the exclusion zone
        }

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
                { "Exclude Bottom Right", ExcludeBottomRightCorner },
                { "Exclusion Right Width", ExclusionZoneRightWidth, 1 },
                { "Exclusion Bottom Height", ExclusionZoneBottomHeight, 1 },
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
            
            // Handle new exclusion zone settings (backwards compatible)
            if (i < settings.Count)
                ExcludeBottomRightCorner = Convert.ToBoolean(settings[i++]);
            if (i < settings.Count)
                ExclusionZoneRightWidth = StringToFloat(settings[i++]);
            if (i < settings.Count)
                ExclusionZoneBottomHeight = StringToFloat(settings[i++]);
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


        // Used by SpanOptimize
        public static CameraIntrinsic CameraIntrinsic = new CameraIntrinsic(
            -1, // Negate the focal length 
            CameraIntrinsic.DefaultFocalLength,
            CameraIntrinsic.DefaultImageWidth * 2,
            CameraIntrinsic.DefaultImageHeight * 2);
    }


    public class CameraIntrinsic
    {
        // These are default values from a single specific camera. Only use if nothing else is available.
        public const double DefaultFocalLength = 9.1; // mm
        public const double DefaultImageWidth = 640; // px
        public const double DefaultImageHeight = 512; // px
        public const double DefaultSensorWidth = 7.68; // mm
        public const double DefaultSensorHeight = 6.144; // mm


        public double ImageWidth;
        public double ImageHeight;
        public double Cx;
        public double Cy;
        public double Fx;
        public double Fy;
        public Mat K = new Mat(3, 3, MatType.CV_64F);
        public Mat KInv = new Mat(3, 3, MatType.CV_64F);


        // Drone camera intrinsic matrix constructor. Used by SpanOptimize
        public CameraIntrinsic(
            float focal_sign, // +1 or -1
            double focalLength = DefaultFocalLength,
            double imageWidth = DefaultImageWidth,
            double imageHeight = DefaultImageHeight,
            double sensorWidth = DefaultSensorWidth,
            double sensorHeight = DefaultSensorHeight)
        {
            ImageWidth = imageWidth;
            ImageHeight = imageHeight;
            Cx = imageWidth / 2;
            Cy = imageHeight / 2;
            Fx = focal_sign * focalLength * imageWidth / sensorWidth; // F * pixels per mm = focal length in mm x image width px / sensor width mm
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


        // Create a drone camera intrinsic matrix. Used by DroneTargetCalculator
        public static Accord.Math.Matrix3x3 Intrinsic(
            double rawFocalLength = BaseConstants.UnknownValue,
            double rawImageWidth = BaseConstants.UnknownValue,
            double rawImageHeight = BaseConstants.UnknownValue,
            double rawSensorWidth = BaseConstants.UnknownValue,
            double rawSensorHeight = BaseConstants.UnknownValue)
        {
            double focalLength = rawFocalLength > 0 ? rawFocalLength : DefaultFocalLength;
            double imageWidth = rawImageWidth > 0 ? rawImageWidth : DefaultImageWidth;
            double imageHeight = rawImageHeight > 0 ? rawImageHeight : DefaultImageHeight;
            double sensorWidth = rawSensorWidth > 0 ? rawSensorWidth : DefaultSensorWidth;
            double sensorHeight = rawSensorHeight > 0 ? rawSensorHeight : DefaultSensorHeight;

            var Cx = imageWidth / 2;
            var Cy = imageHeight / 2;
            var Fx = focalLength * imageWidth / sensorWidth; // F * pixels per mm = focal length in mm x image width px / sensor width mm
            var Fy = focalLength * imageHeight / sensorHeight;
            Accord.Math.Matrix3x3 K = new();
            K.V00 = (float)Fx;
            K.V01 = 0;
            K.V02 = (float)Cx;
            K.V10 = 0;
            K.V11 = (float)Fy;
            K.V12 = (float)Cy;
            K.V20 = 0;
            K.V21 = 0;
            K.V22 = 1;
            return K;
        }

        // Returns default values from a single specific camera. Try not to use this.
        public static Accord.Math.Matrix3x3 Default3x3()
        {
            return Intrinsic();
        }
    }
}