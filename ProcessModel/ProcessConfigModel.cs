// Copyright SkyComb Limited 2023. All rights reserved.
using Emgu.CV.ML;
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
        // --------------------- Debug --------------------- 
        // The ID of the Object that we want to focus on (if any).
        public int FocusObjectId { get; set; } = 0;


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


        // Optical flow - Implements sparse iterative version of Lucas-Kanade optical flow in pyramids ([Bouguet00]). 
        //
        // FlowMaxPyramid:
        // Maximal pyramid level number.If 0, pyramids are not used (single level), if 1, two levels are used, etc.
        public int FlowMaxPyramid { get; set; } = 4;
        // MinEigThreshold:
        // The algorithm calculates the minimum eigen value of a 2x2 normal matrix of optical
        // flow equations (this matrix is called a spatial gradient matrix in [Bouguet00]),
        // divided by number of pixels in a window; if this value is less than minEigThreshold,
        // then a corresponding feature is filtered out and its flow is not processed, so
        // it allows to remove bad points and get a performance boost.
        public double FlowMinEigThreshold { get; set; } = 0.0001;
        public int FlowMaxIterations { get; set; } = 1;
        // Size of the search window of each pyramid level.
        public int FlowSearchWindow { get; set; } = 15;


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

        // If the camera view includes the horizon, then the camera can experience "thermal bloom",
        // giving bad thermal readings, and lots of suprious features.
        // Manual operators of drtones occassionally look at the horizon to make sure the drone is not going to run into anything.
        // We automatically ignore video frames where the camera down angle is greater than MinCameraDownDeg.
        // If MinCameraDownDeg is set to 35, and camera has vertical field of vision (VFOV) of 47.6 degrees,
        // then the highest view the app processes is 35 +/- 24 degrees which is 11 to 49 degrees down from the horizon.
        public int MinCameraDownDeg { get; set; } = 35; // Min 25, Max 90

        // We set a maximum distance and ignore objects detected beyond that distance.
        // SkyComb Analyst is not designed to detect things in the far distance (as they would need to be very large).
        // Things detected by 
        public int MaxFeatureDistanceM { get; set; } = 150; // Min 50, Max 250


        // Describe (summarise) the key process settings.
        public string Describe()
        {
            var answer =
                "Heat Threshold: " + HeatThresholdValue + "\r\n" +
                "Save Annotated Video: " + (SaveAnnotatedVideo ? "Yes" : "No") + "\r\n";

            if (SaveObjectData != SaveObjectDataEnum.None)
                answer += "Save Objects: " + SaveObjectData + "\r\n";

            return answer;
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


        public void ValidateMinCameraDownDeg()
        {
            if (MinCameraDownDeg < 25)
                MinCameraDownDeg = 25;

            if (MinCameraDownDeg > 90)
                MinCameraDownDeg = 90;
        }


        public void ValidateMaxFeatureDistanceM()
        {
            if (MaxFeatureDistanceM < 50)
                MaxFeatureDistanceM = 50;

            if (MaxFeatureDistanceM > 250)
                MaxFeatureDistanceM = 250;
        }


        // Get the class's settings as datapairs (e.g. for saving to a spreadsheet)
        public DataPairList GetModelSettings()
        {
            return new DataPairList
            {
                { "Focus Object Id", FocusObjectId },
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
                { "Flow Max Pyramid", FlowMaxPyramid },
                { "Flow Min Eig Threshold", FlowMinEigThreshold.ToString() },
                { "Flow Max Iterations", FlowMaxIterations },
                { "Flow Search Window", FlowSearchWindow },
            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadModelSettings(List<string> settings)
        {
            int i = 0;
            FocusObjectId = StringToNonNegInt(settings[i++]);
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
            FlowMaxPyramid = StringToNonNegInt(settings[i++]);
            FlowMinEigThreshold = StringToDouble(settings[i++]);
            FlowMaxIterations = StringToNonNegInt(settings[i++]);
            FlowSearchWindow = StringToNonNegInt(settings[i++]);
        }


        // Load this object's settings from strings (loaded from App.Config)
        // This function must align to the above GetSettings function.
        public void LoadSettings(DataPairList settings)
        {
            foreach (var setting in settings)
                switch (setting.Key)
                {
                    case "focusobjectid":
                        FocusObjectId = StringToNonNegInt(setting.Value); break;
                }
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
                { "Min Camera Down Deg", MinCameraDownDeg },
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
            MinCameraDownDeg = StringToNonNegInt(settings[i++]);
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