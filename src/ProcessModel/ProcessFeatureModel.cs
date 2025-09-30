// Copyright SkyComb Limited 2025. All rights reserved. 
using SkyCombGround.CommonSpace;
using System.Drawing;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // Is the feature:
    // - Real (containing hot pixels), 
    // - Unreal (Persistance search feature, no hot pixels), or
    // - Consumed (was Real once but now eaten by another Real feature in the same block).
    public enum FeatureTypeEnum { Real, Unreal, Consumed };


    // A class to hold a significant feature 
    public class ProcessFeatureModel : ConfigBase
    {
        // Static NextFeatureId shared by all Comb features
        public static int NextFeatureId = 0;

        // Height algorithm "success" statuses
        public const string UnrealCopyHeightAlgorithm = "UC"; // Height was copied from a previous feature for an unreal feature.
        public const string LineOfSightHeightAlgorithm = "LOS"; // Height was calculated from camera down angle and land contours.


        // Unique identifier
        public int FeatureId { get; set; }
        // Is this feature actively being tracked now
        public bool IsTracked { get; set; }
        // Is this feature significant?
        public bool Significant { get; set; } = false;
        // A feature can be associated with an object
        public int ObjectId { get; set; }


        // A Comb feature is associated 1-1 with a Block
        public int BlockId { get; set; } = UnknownValue;

        // Is the feature Real, Unreal or Consumed.
        public FeatureTypeEnum Type { get; set; }

        // Rectangular box that bounds the hot pixels. Origin is top (Y=0) left (X=0) of image.
        // Note that y = 0 is the top of the image, but the furtherest pixel from the drone 
        public Rectangle PixelBox { get; set; }

        // Location of this feature inside drone flight box, in meters
        public DroneLocation? LocationM { get; set; }

        // Height of this feature above the ground. 
        public float HeightM { get; set; }
        // Technique used to calculate HeightM 
        public string HeightAlgorithm { get; set; } = "";

        public int MinHeat { get; set; } = UnknownValue;
        public int MaxHeat { get; set; } = UnknownValue;
        // Number of image pixels in the PixelBox above the ProcessConfig min heat threshold
        public int NumHotPixels { get; set; } = 0;
        // Sum of how much hotter than the ProcessConfig min heat threshold the hot pixels are
        public int SumHotPixels { get; set; } = 0;
        // Number of feature pixels with heat 255 (max possible)
        public int NumMaxHeatPixels { get; set; } = 0;


        public ProcessFeatureModel(int blockId, FeatureTypeEnum type)
        {
            NextFeatureId++;

            FeatureId = NextFeatureId;
            BlockId = blockId;
            Type = type;

            ResetCalcedMemberData();
        }


        // Constructor used when loaded objects from the datastore
        public ProcessFeatureModel(List<string> settings)
        {
            ResetCalcedMemberData();
            LoadSettings(settings);
        }


        public void Set_LocationM_HeightM(DroneLocation? location = null, float heightM = BaseConstants.UnknownHeight)
        {
            LocationM = (location == null ? null : location.Clone());
            HeightM = heightM;
        }


        // Reset member data to mirror a newly created feature.
        // Used in experimentation to allow repeated calculation run against this feature.
        public void ResetCalcedMemberData()
        {
            IsTracked = true;
            ObjectId = 0;

            Set_LocationM_HeightM();

            HeightAlgorithm = "";
            // MinHeat
            // MaxHeat
            // NumHotPixels = 0; Derived from image processing. Can't be recalced. So excluded 
            // SumHotPixels = 0; Derived from image processing. Can't be recalced. So excluded
            // Significant = false; Derived from NumHotPixels. Can't be recalced. So excluded
        }


        // Is this feature significant?
        public void Calculate_Significant()
        {
            Significant = (NumHotPixels >= ProcessConfigModel.FeatureMinPixels);
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int FeatureIdSetting = 1;
        public const int IsTrackedSetting = 2;
        public const int SignificantSetting = 3;
        public const int ObjectIdSetting = 4;
        public const int BlockIdSetting = 5;
        public const int TypeSetting = 6;
        public const int NorthingMSetting = 7;
        public const int EastingMSetting = 8;
        public const int HeightMSetting = 9;
        public const int HeightAlgorithmSetting = 10;
        public const int PixelBoxXSetting = 11;
        public const int PixelBoxYSetting = 12;
        public const int PixelBoxWidthSetting = 13;
        public const int PixelBoxHeightSetting = 14;
        public const int MinHeatSetting = 15;
        public const int MaxHeatSetting = 16;
        public const int NumHotPixelsSetting = 17;
        public const int SumHotPixelsSetting = 18;
        public const int NumMaxHotPixelsSetting = 19;
        public const int LegIdSetting = 20;
        public const int RangeMSetting = 21;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public virtual DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Feature", FeatureId },
                { "Is Tracked", IsTracked},
                { "Significant", Significant },
                { "Object", ObjectId },
                { "Block", BlockId },
                { "Type", Type.ToString() },
                { "Northing M", (LocationM != null ? LocationM.NorthingM : 0), LocationNdp },
                { "Easting M", (LocationM != null ? LocationM.EastingM : 0), LocationNdp },
                { "HeightM", (HeightM == UnknownValue ? UnknownHeight : HeightM), HeightNdp },
                { "Ht Algorithm", HeightAlgorithm },
                { "Box.X", PixelBox.X },
                { "Box.Y", PixelBox.Y },
                { "Box.Width", PixelBox.Width },
                { "Box.Height", PixelBox.Height },
                { "Min Heat", MinHeat },
                { "Max Heat", MaxHeat },
                { "Num Hot Pxs", NumHotPixels },
                { "Sum Hot Pxs", SumHotPixels },
                { "Num MaxHot Pxs", NumMaxHeatPixels },
            };
        }


        public virtual void LoadSettings(List<string> settings)
        {
            // Convert int settings in batch for speed  
            var intSettings = new int[12];
            var intIndices = new[] {
                FeatureIdSetting - 1,
                ObjectIdSetting - 1,
                BlockIdSetting - 1,
                PixelBoxXSetting - 1,
                PixelBoxYSetting - 1,
                PixelBoxWidthSetting - 1,
                PixelBoxHeightSetting - 1,
                MinHeatSetting - 1,
                MaxHeatSetting - 1,
                NumHotPixelsSetting - 1,
                SumHotPixelsSetting - 1,
                NumMaxHotPixelsSetting - 1 };
            var intInputs = intIndices.Select(i => settings[i]).ToArray();
            ConfigBase.ConvertStringBatch(intInputs, intSettings);


            FeatureId = intSettings[0];
            IsTracked = settings[IsTrackedSetting - 1] == "true";
            Significant = settings[SignificantSetting - 1] == "true";
            ObjectId = intSettings[1];
            BlockId = intSettings[2];
            Type = (FeatureTypeEnum)Enum.Parse(typeof(FeatureTypeEnum), settings[TypeSetting - 1]);
            LocationM = new DroneLocation(settings[NorthingMSetting - 1], settings[EastingMSetting - 1]);
            HeightM = StringToFloat(settings[HeightMSetting - 1]);
            HeightAlgorithm = settings[HeightAlgorithmSetting - 1];
            PixelBox = new Rectangle(intSettings[3], intSettings[4], intSettings[5], intSettings[6]);
            MinHeat = intSettings[7];
            MaxHeat = intSettings[8];
            NumHotPixels = intSettings[9];
            SumHotPixels = intSettings[10];
            NumMaxHeatPixels = intSettings[11];
        }

        // Unit test to ensure that GetSettings and LoadSettings form a consistent pair.
        public static void TestSettingsPair()
        {
            var rand = new Random();
            var obj = new ProcessFeatureModel(rand.Next(1, 10000), FeatureTypeEnum.Real)
            {
                FeatureId = rand.Next(1, 10000),
                IsTracked = rand.Next(0, 2) == 1,
                Significant = rand.Next(0, 2) == 1,
                ObjectId = rand.Next(1, 10000),
                BlockId = rand.Next(1, 10000),
                Type = FeatureTypeEnum.Real,
                PixelBox = new Rectangle(rand.Next(0, 1000), rand.Next(0, 1000), rand.Next(1, 1000), rand.Next(1, 1000)),
                LocationM = new DroneLocation(rand.Next(-1000, 1000), rand.Next(-1000, 1000)),
                HeightM = (float)rand.NextDouble() * 100,
                HeightAlgorithm = "ALG" + rand.Next(1, 10000),
                MinHeat = rand.Next(0, 255),
                MaxHeat = rand.Next(0, 255),
                NumHotPixels = rand.Next(0, 10000),
                SumHotPixels = rand.Next(0, 100000),
                NumMaxHeatPixels = rand.Next(0, 10000)
            };
            // Save settings to list
            var settings = obj.GetSettings().Select(dp => dp.Value.ToString()).ToList();
            // Create a new object and load settings
            var obj2 = new ProcessFeatureModel(settings);
            // Compare all relevant properties
            Assert(obj.FeatureId == obj2.FeatureId, "FeatureId mismatch");
            Assert(obj.IsTracked == obj2.IsTracked, "IsTracked mismatch");
            Assert(obj.Significant == obj2.Significant, "Significant mismatch");
            Assert(obj.ObjectId == obj2.ObjectId, "ObjectId mismatch");
            Assert(obj.BlockId == obj2.BlockId, "BlockId mismatch");
            Assert(obj.Type == obj2.Type, "Type mismatch");
            Assert(obj.PixelBox == obj2.PixelBox, "PixelBox mismatch");
            Assert(Math.Abs(obj.LocationM.NorthingM - obj2.LocationM.NorthingM) < 0.005f, "LocationM.NorthingM mismatch");
            Assert(Math.Abs(obj.LocationM.EastingM - obj2.LocationM.EastingM) < 0.005f, "LocationM.EastingM mismatch");
            Assert(Math.Abs(obj.HeightM - obj2.HeightM) < 0.005f, "HeightM mismatch");
            Assert(obj.HeightAlgorithm == obj2.HeightAlgorithm, "HeightAlgorithm mismatch");
            Assert(obj.MinHeat == obj2.MinHeat, "MinHeat mismatch");
            Assert(obj.MaxHeat == obj2.MaxHeat, "MaxHeat mismatch");
            Assert(obj.NumHotPixels == obj2.NumHotPixels, "NumHotPixels mismatch");
            Assert(obj.SumHotPixels == obj2.SumHotPixels, "SumHotPixels mismatch");
            Assert(obj.NumMaxHeatPixels == obj2.NumMaxHeatPixels, "NumMaxHeatPixels mismatch");
        }
    }
}
