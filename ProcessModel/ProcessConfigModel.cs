// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    public enum SmoothProcessEnum { Blur, Gaussian, Median, None };

    public enum ThresholdProcessEnum { Binary, BinaryInv, ToZero, ToZeroInv, Trunc, None };

    public enum SaveObjectDataEnum { All, Significant, None };

    public enum SavePixelsEnum { All, Some, None };


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
        // Single frame Yolo detection confidence
        public const float YoloDetectConfidenceDefault = 0.66f;
        // Successive frame Yolo overlap threshold. Typically 0.2 to 0.4
        public const float YoloIoUDefault = 0.25f; 


        // --------------------- Process Techniques --------------------- 

        // Smooth Process
        public SmoothProcessEnum SmoothProcess { get; set; } = SmoothProcessEnum.None;
        public int SmoothPixels { get; set; } = 3;


        // Theshold:
        // Theshold Process. Takes values binary, binaryinv, tozero, tozeroinv, trunc, none. Lowercase
        public ThresholdProcessEnum ThresholdProcess { get; set; } = ThresholdProcessEnum.Binary;
        // Pixel gray-scale value that ThresholdProcess uses. Takes values from 50 to 255
        public int HeatThresholdValue { get; set; } = 235;
        // A setting to allow stupidly hot pixels to (optionally) be ignored.
        public int TruncThresholdValue { get; set; } = UnknownValue;


        // Comb:
        // SkyComb-specific detection method "Comb".
        // Minimum number of hot pixels required inside a bounding rectangle, to draw a red rect in output
        public int FeatureMinPixels { get; set; } = 8;
        // Minimum percentage of hot pixels required inside a bounding rectangle, to create a feature
        public int FeatureMinDensityPerc { get; set; } = 20;
        // Maximum feature size (aka length or width) in pixels
        // Sometimes the algorithm generates absurdly large features e.g. 100. This reduces this issue.
        // ToDo: Value should be based on physically realities e.g. the maximum mammal size given standard drone altitude 
        public int FeatureMaxSize { get; set; } = 100;
        // Minimum overlap percentage between two features that is considered significant
        public int FeatureMinOverlapPerc { get; set; } = 25;


        // Duration (in milliseconds) that object must be tracked for before it is highlighted
        public int ObjectMinDurationMs { get; set; } = 500;
        // Minimum percentage difference between ground speed and object speed for object to be considered above ground.
        public int ObjectMinSpeedDiffPerc { get; set; } = 10;
        // Persist searching for an object (usign unreal features) for this many steps
        public int ObjectMaxUnrealBlocks { get; set; } = 3;
        // To be highlighted, an object must have this many hot pixels per real step
        public int ObjectMinPixelsPerBlock { get; set; } = 5;
        // To be highlighted, an object must have this percentage of hot pixels inside bounding rectangle per step
        public int ObjectMinDensityPerc { get; set; } = 33;


        // Distance process:
        public int DistanceMinGray { get; set; } = 1;
        public int DistanceMaskSize { get; set; } = 0;


        // Contour process:
        public double ContourMinArea { get; set; } = 5;


        // Good Feature To Track (Gftt) process:
        public int GfttMaxCorners { get; set; } = 40; // Maximum number of corners to detect.
        public double GfttQualityLevel { get; set; } = 0.3;
        public int GfttMinDistance { get; set; } = 7;
        public int GfttBlockSize { get; set; } = 3;
        public bool GfttUseHarris { get; set; } = false;
        public double GfttK { get; set; } = 0.04;


        // YOLO
        // Single frame Yolo detection confidence
        public float YoloDetectConfidence { get; set; } = ProcessConfigModel.YoloDetectConfidenceDefault;
        // Successive frame Yolo overlap threshold
        public float YoloIoU { get; set; } = ProcessConfigModel.YoloIoUDefault; // Typically 0.2 to 0.4
        // Yolo merge objects based on similarity threshold
        //public float YoloMergeConfidence { get; set; } = ProcessConfigModel.YoloMergeConfidenceDefault;
        //public float YoloMergeVelocityWeighting { get; set; } = ProcessConfigModel. YoloMergeVelocityWeightingDefault;
        //public float YoloMergePositionWeighting { get; set; } = ProcessConfigModel.YoloMergePositionWeightingDefault;


        // --------------------- Saving Output --------------------- 
        // Create annotated video file as output
        public bool SaveAnnotatedVideo { get; set; } = true;
        // Save the objects and features to the datastore. Takes values all, significant, none.
        public SaveObjectDataEnum SaveObjectData { get; set; } = SaveObjectDataEnum.Significant; 
        // Save the pixels to the datastore (slow & bulky). Takes values all, some, none.
        public SavePixelsEnum SavePixels { get; set; } = SavePixelsEnum.None;


        // --------------------- Error Thresholds --------------------- 
        // The maximum +/- inaccuracy (in meters) in an object's estimated location to be considered "good" 
        public float GoodLocationErrM { get; set; } = 0.5f;
        // The minimum +/- inaccuracy (in meters) in an object's estimated location to be considered "bad" 
        public float BadLocationErrM { get; set; } = 1.0f;
        // The maximum +/- inaccuracy (in meters) in an object's estimated height to be considered "good" 
        public float GoodHeightErrM { get; set; } = 0.5f;
        // The minimum +/- inaccuracy (in meters) in an object's estimated height to be considered "bad" 
        public float BadHeightErrM { get; set; } = 1.0f;


        // --------------------- Processing Limits --------------------- 

        // We set a maximum distance and ignore objects detected beyond that distance.
        // SkyComb Analyst is not designed to detect things in the far distance (as they would need to be very large).
        // Things detected by 
        public int MaxFeatureDistanceM { get; set; } = 150; // Min 50, Max 250


        // The ThresholdValue should be in range 50 to 255.
        // A user may mistake the range for 0.0 to 1.0, so we set the min value to 50
        public void ValidateHeatThresholdValue()
        {
            if (HeatThresholdValue < 50)
                HeatThresholdValue = 50;

            if (HeatThresholdValue > 255)
                HeatThresholdValue = 255;
        }


        public void ValidateMaxFeatureDistanceM()
        {
            if (MaxFeatureDistanceM < 50)
                MaxFeatureDistanceM = 50;

            if (MaxFeatureDistanceM > 250)
                MaxFeatureDistanceM = 250;
        }


        public void ValidateYolo()
        {
            if ((YoloDetectConfidence < 0.1f) || (YoloDetectConfidence > 0.9f))
                YoloDetectConfidence = ProcessConfigModel.YoloDetectConfidenceDefault;

            if ((YoloIoU < 0.1f) || (YoloIoU > 0.9f))
                YoloIoU = ProcessConfigModel.YoloIoUDefault;

            //if ((YoloMergeConfidence < 0.1f) || (YoloMergeConfidence > 0.9f))
            //    YoloMergeConfidence = ProcessConfigModel.YoloMergeConfidenceDefault;
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
                    // PQR TBC. 
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
                { "Smooth Process", SmoothProcess.ToString() },
                { "Smooth Pixels", SmoothPixels },
                { "Threshold Process", ThresholdProcess.ToString() },
                { "Threshold Value", HeatThresholdValue },
                { "Trunc Threshold Value", TruncThresholdValue },
                { "Feature Min Pixels", FeatureMinPixels },
                { "Feature MinDensity Perc", FeatureMinDensityPerc },
                { "Feature Max Size", FeatureMaxSize },
                { "Feature Min Overlap Perc", FeatureMinOverlapPerc },
                { "Object Min Duration Ms", ObjectMinDurationMs },
                { "Object Min Speed Diff Perc", ObjectMinSpeedDiffPerc },
                { "Object Max Unreal Blocks", ObjectMaxUnrealBlocks },
                { "Object Min Pixels Per Block", ObjectMinPixelsPerBlock },
                { "Object Min Density Perc", ObjectMinDensityPerc },
                { "Contour Min Area", ContourMinArea.ToString() },
                { "Distance Min Gray", DistanceMinGray },
                { "Distance Mask Size", DistanceMaskSize },
                { "Gftt Max Corners", GfttMaxCorners },
                { "Gftt Quality Level", GfttQualityLevel.ToString() },
                { "Gftt Min Distance", GfttMinDistance },
                { "Gftt Block Size", GfttBlockSize },
                { "Gftt Use Harris", GfttUseHarris.ToString() },
                { "Gftt K", GfttK.ToString() },
                { "Yolo Confidence", YoloDetectConfidence, 2 },
                { "Yolo IoU", YoloIoU, 2 },
            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadModelSettings(List<string> settings)
        {
            int i = 0;
            SmoothProcess = (SmoothProcessEnum)Enum.Parse(typeof(SmoothProcessEnum), settings[i++]);
            SmoothPixels = StringToNonNegInt(settings[i++]);
            ThresholdProcess = (ThresholdProcessEnum)Enum.Parse(typeof(ThresholdProcessEnum), settings[i++]);
            HeatThresholdValue = StringToNonNegInt(settings[i++]);
            TruncThresholdValue = StringToInt(settings[i++]);
            FeatureMinPixels = StringToNonNegInt(settings[i++]);
            FeatureMinDensityPerc = StringToNonNegInt(settings[i++]);
            FeatureMaxSize = StringToNonNegInt(settings[i++]);
            FeatureMinOverlapPerc = StringToNonNegInt(settings[i++]);
            ObjectMinDurationMs = StringToNonNegInt(settings[i++]);
            ObjectMinSpeedDiffPerc = StringToNonNegInt(settings[i++]);
            ObjectMaxUnrealBlocks = StringToNonNegInt(settings[i++]);
            ObjectMinPixelsPerBlock = StringToNonNegInt(settings[i++]);
            ObjectMinDensityPerc = StringToNonNegInt(settings[i++]);
            ContourMinArea = StringToDouble(settings[i++]);
            DistanceMinGray = StringToNonNegInt(settings[i++]);
            DistanceMaskSize = StringToNonNegInt(settings[i++]);
            GfttMaxCorners = StringToNonNegInt(settings[i++]);
            GfttQualityLevel = StringToDouble(settings[i++]);
            GfttMinDistance = StringToNonNegInt(settings[i++]);
            GfttBlockSize = StringToNonNegInt(settings[i++]);
            GfttUseHarris = StringToBool(settings[i++]);
            GfttK = StringToDouble(settings[i++]);
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
                { "Save Pixels", SavePixels.ToString() },
                { "Good Location Err M", GoodLocationErrM, LocationNdp },
                { "Bad Location Err M", BadLocationErrM, LocationNdp },
                { "Good Height Err M", GoodHeightErrM, HeightNdp },
                { "Bad Height Err M", BadHeightErrM, HeightNdp },
                { "Max Feature Distance M", MaxFeatureDistanceM },
            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadOutputSettings(List<string> settings)
        {
            int i = 0;
            SaveAnnotatedVideo = Convert.ToBoolean(settings[i++]);
            SaveObjectData = (SaveObjectDataEnum)Enum.Parse(typeof(SaveObjectDataEnum), settings[i++]);
            SavePixels = (SavePixelsEnum)Enum.Parse(typeof(SavePixelsEnum), settings[i++]);
            GoodLocationErrM = StringToNonNegFloat(settings[i++]);
            BadLocationErrM = StringToNonNegFloat(settings[i++]);
            GoodHeightErrM = StringToNonNegFloat(settings[i++]);
            BadHeightErrM = StringToNonNegFloat(settings[i++]);
            MaxFeatureDistanceM = StringToNonNegInt(settings[i++]);
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

    }
}