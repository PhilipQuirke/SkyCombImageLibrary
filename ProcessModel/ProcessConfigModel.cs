// Copyright SkyComb Limited 2024. All rights reserved. 
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
        public const float YoloDetectConfidenceDefault = 0.66f;
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
        public int FeatureMinPixels { get; set; } = 8;
        // Maximum feature size (aka length or width) in pixels
        // Sometimes the algorithm generates absurdly large features e.g. 100. This reduces this issue.
        // ToDo: Value should be based on physically realities e.g. the maximum mammal size given standard drone altitude 
        public int FeatureMaxSize { get; set; } = 100;
        // Minimum overlap percentage between two features that is considered significant
        public int FeatureMinOverlapPerc { get; set; } = 5;


        // Duration (in milliseconds) that object must be tracked for before it is highlighted
        public int ObjectMinDurationMs { get; set; } = 500;
        // Minimum percentage difference between ground speed and object speed for object to be considered above ground.
        public int ObjectMaxUnrealBlocks { get; set; } = 5;
        // To be highlighted, an object must have this many hot pixels in at least one real step
        public int ObjectMinPixels { get; set; } = 5;


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
        // The maximum +/- inaccuracy (in meters) in an object's estimated location to be considered "good" 
        public float GoodLocationErrM { get; set; } = 1;
        // The maximum +/- inaccuracy (in meters) in an object's estimated height to be considered "good" 
        public float GoodHeightErrM { get; set; } = 1;



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
                { "Threshold Value", HeatThresholdValue },
                { "Feature Min Pixels", FeatureMinPixels },
                { "Feature Max Size", FeatureMaxSize },
                { "Feature Min Overlap Perc", FeatureMinOverlapPerc },
                { "Object Min Duration Ms", ObjectMinDurationMs },
                { "Object Max Unreal Blocks", ObjectMaxUnrealBlocks },
                { "Object Min Pixels", ObjectMinPixels },
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
            FeatureMinPixels = StringToNonNegInt(settings[i++]);
            FeatureMaxSize = StringToNonNegInt(settings[i++]);
            FeatureMinOverlapPerc = StringToNonNegInt(settings[i++]);
            ObjectMinDurationMs = StringToNonNegInt(settings[i++]);
            ObjectMaxUnrealBlocks = StringToNonNegInt(settings[i++]);
            ObjectMinPixels = StringToNonNegInt(settings[i++]);
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
                { "Good Location Err M", GoodLocationErrM, LocationNdp },
                { "Good Height Err M", GoodHeightErrM, HeightNdp },

            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadOutputSettings(List<string> settings)
        {
            int i = 0;
            SaveAnnotatedVideo = Convert.ToBoolean(settings[i++]);
            SaveObjectData = (SaveObjectDataEnum)Enum.Parse(typeof(SaveObjectDataEnum), settings[i++]);
            GoodLocationErrM = StringToNonNegFloat(settings[i++]);
            GoodHeightErrM = StringToNonNegFloat(settings[i++]);

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