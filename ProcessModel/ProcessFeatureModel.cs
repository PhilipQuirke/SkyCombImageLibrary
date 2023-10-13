// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombGround.CommonSpace;
using System.Drawing;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // Is the feature:
    // - Real (containing hot pixels), 
    // - Unreal (Persistance search feature, no hot pixels), or
    // - Consumed (was Real once but now eaten by another Real feature in the same block).
    public enum CombFeatureTypeEnum { Real, Unreal, Consumed };


    // A class to hold a significant feature 
    public class ProcessFeatureModel : ConfigBase
    {
        // Static NextFeatureId shared by all Comb features
        public static int NextFeatureId = 0;

        // Height algorithm "success" statuses
        public const string UnrealCopyHeightAlgorithm = "UC"; // Height was copied from a previous feature for an unreal feature.
        public const string BaseLineHeightAlgorithm = "BL"; // Height was calculated from change in camera down angle and the drone base line movement
        public const string LineOfSightHeightAlgorithm = "LOS"; // Height was calculated from camera down angle and land contours.


        // Unique identifier
        public int FeatureId { get; set; }
        // Is this feature actively being tracked now
        public bool IsTracked { get; set; }
        // Is this feature significant?
        public bool Significant { get; set; }
        // Attributes about this feature
        public string Attributes { get; set; } = "";
        // A feature can be associated with an object
        public int ObjectId { get; set; }


        // A Comb feature is associated 1-1 with a Block
        public int BlockId { get; set; } = UnknownValue;

        public int MinHeat { get; set; } = 0;
        public int MaxHeat { get; set; } = 0;

        // Is the feature Real, Unreal or Consumed.
        public CombFeatureTypeEnum Type { get; set; }

        // Rectangular box that bounds the hot pixels. Origin is top (Y=0) left (X=0) of image.
        // Note that y = 0 is the top of the image, but the furtherest pixel from the drone 
        public Rectangle PixelBox { get; set; }

        // Location of this feature inside drone flight box, in meters
        public DroneLocation? LocationM { get; set; }

        // Height of this feature above the ground. 
        public float HeightM { get; set; }
        // Technique used to calculate HeightM 
        public string HeightAlgorithm { get; set; }


        public ProcessFeatureModel(int blockId, CombFeatureTypeEnum type)
        {
            NextFeatureId++;

            FeatureId = NextFeatureId;
            BlockId = blockId;
            Type = type;
            ResetMemberData();
        }


        // Constructor used when loaded objects from the datastore
        public ProcessFeatureModel(List<string> settings)
        {
            ResetMemberData();
            LoadSettings(settings);
        }


        // Reset member data to mirror a newly created feature.
        // Used in experimentation to allow repeated calculation run against this feature.
        public void ResetMemberData()
        {
            IsTracked = true;
            Significant = false;
            Attributes = "";
            ObjectId = 0;
            LocationM = null;
            HeightM = UnknownValue;
            HeightAlgorithm = "";
        }


        // Set the HeightAlgorithm value to an error value - unless it is already set to a success value.
        // There a few height algorithms. If one succeeds, we retain that success value
        public void SetHeightAlgorithmError(string theCase)
        {
            if ((HeightAlgorithm != UnrealCopyHeightAlgorithm) &&
                (HeightAlgorithm != BaseLineHeightAlgorithm) &&
                (HeightAlgorithm != LineOfSightHeightAlgorithm))
                HeightAlgorithm = theCase;
        }


        private int UnknownHeight = -2;

        // One-based settings index values. Must align with GetSettings procedure below     
        public const int FeatureIdSetting = 1;
        public const int IsTrackedSetting = 2;
        public const int SignificantSetting = 3;
        public const int NotesSetting = 4;
        public const int ObjectIdSetting = 5;
        public const int BlockIdSetting = 6;
        public const int TypeSetting = 7;
        public const int NorthingMSetting = 8;
        public const int EastingMSetting = 9;
        public const int HeightMSetting = 10;
        public const int HeightAlgorithmSetting = 11;
        public const int PixelBoxXSetting = 12;
        public const int PixelBoxYSetting = 13;
        public const int PixelBoxWidthSetting = 14;
        public const int PixelBoxHeightSetting = 15;
        public const int MinHeatSetting = 16;
        public const int MaxHeatSetting = 17;
        public const int NumHotPixelsSetting = 18;
        public const int DensityPercSetting = 19;
        public const int PixelDensityGoodSetting = 20;
        public const int LegIdSetting = 21;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        virtual public DataPairList GetSettings()
        {
            // If have blank Attributes, the row will stop at blank, and so not load fully from DataStore
            var theAttributes = (Attributes == null || Attributes == "") ? "No" : Attributes;

            return new DataPairList
            {
                { "Feature", FeatureId },
                { "Is Tracked", IsTracked},
                { "Significant", Significant },
                { "Attributes", theAttributes },
                { "Object", ObjectId },
                { "Block", BlockId },
                { "Type", Type.ToString() },
                { "Northing M", (LocationM != null ? LocationM.NorthingM : 0), LocationNdp },
                { "Easting M", (LocationM != null ? LocationM.EastingM : 0), LocationNdp },
                { "Height M", (HeightM == UnknownValue ? UnknownHeight : HeightM), HeightNdp },
                { "Ht Algorithm", HeightAlgorithm },
                { "Box.X", PixelBox.X },
                { "Box.Y", PixelBox.Y },
                { "Box.Width", PixelBox.Width },
                { "Box.Height", PixelBox.Height },
                { "Min Heat", MinHeat },
                { "Max Heat", MaxHeat },
            };
        }


        public void LoadSettings(List<string> settings)
        {
            FeatureId = StringToNonNegInt(settings[FeatureIdSetting - 1]);
            IsTracked = settings[IsTrackedSetting-1] == "true";
            Significant = settings[SignificantSetting-1] == "true";
            Attributes = settings[NotesSetting-1];
            ObjectId = StringToNonNegInt(settings[ObjectIdSetting-1]);

            BlockId = StringToNonNegInt(settings[BlockIdSetting - 1]);
            Type = (CombFeatureTypeEnum)Enum.Parse(typeof(CombFeatureTypeEnum), settings[TypeSetting - 1]);
            LocationM = new DroneLocation(settings[NorthingMSetting - 1], settings[EastingMSetting - 1]);

            HeightM = StringToFloat(settings[HeightMSetting - 1]);
            if (HeightM == UnknownHeight)
                HeightM = UnknownValue;
            HeightAlgorithm = settings[ObjectIdSetting - 1];

            PixelBox = new Rectangle(
                StringToInt(settings[PixelBoxXSetting - 1]),
                StringToInt(settings[PixelBoxYSetting - 1]),
                StringToInt(settings[PixelBoxWidthSetting - 1]),
                StringToInt(settings[PixelBoxHeightSetting - 1]));
            MinHeat = StringToNonNegInt(settings[MinHeatSetting - 1]);
            MaxHeat = StringToNonNegInt(settings[MaxHeatSetting - 1]);
        }
    }
}
