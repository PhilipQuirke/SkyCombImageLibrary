using SkyCombGround.CommonSpace;
using System.Drawing;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    /// Is the feature:
    /// - Real (containing hot pixels), 
    /// - Unreal (Persistance search feature, no hot pixels), or
    /// - Consumed (was Real once but now eaten by another Real feature in the same block).
    public enum CombFeatureTypeEnum { Real, Unreal, Consumed };


    // A Comb feature, is a dense cluster of hot pixels, associated 1-1 with a Block
    public class CombFeatureModel : ConfigBase
    {
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


        // Draft height of the parent CombObject as calculated when processing this feature. 
        public float HeightM { get; set; }


        public CombFeatureModel(int blockId, CombFeatureTypeEnum type)
        {
            BlockId = blockId;
            Type = type;
            ResetMemberData();
        }


        // Constructor used when loaded objects from the datastore
        public CombFeatureModel(List<string> settings)
        {
            ResetMemberData();
            LoadSettings(settings);
        }


        // Reset member data to mirror a newly created feature.
        // Used in experimentation to allow repeated calculation run against this feature.
        public void ResetMemberData()
        {
            LocationM = null;
            HeightM = UnknownValue;
        }


        private int UnknownHeight = -2;


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        public void GetSettings(DataPairList settings)
        {
            settings.Add("Block", BlockId);
            settings.Add("Type", Type.ToString());
            settings.Add("Northing M", (LocationM != null ? LocationM.NorthingM : 0), LocationNdp);
            settings.Add("Easting M", (LocationM != null ? LocationM.EastingM : 0), LocationNdp);
            settings.Add("Height M", (HeightM == UnknownValue ? UnknownHeight : HeightM), HeightNdp);
            settings.AddRectange("Box", PixelBox);
            settings.Add("Min Heat", MinHeat);
            settings.Add("Max Heat", MaxHeat);
        }


        // Load this feature's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadSettings(List<string> settings)
        {
            BlockId = StringToNonNegInt(settings[ProcessFeatureModel.BlockIdSetting - 1]);
            Type = (CombFeatureTypeEnum)Enum.Parse(typeof(CombFeatureTypeEnum), settings[ProcessFeatureModel.TypeSetting - 1]);
            LocationM = new DroneLocation(settings[ProcessFeatureModel.NorthingMSetting - 1], settings[ProcessFeatureModel.EastingMSetting - 1]);

            HeightM = StringToFloat(settings[ProcessFeatureModel.HeightMSetting - 1]);
            if (HeightM == UnknownHeight)
                HeightM = UnknownValue;

            PixelBox = new Rectangle(
                StringToInt(settings[ProcessFeatureModel.PixelBoxXSetting - 1]),
                StringToInt(settings[ProcessFeatureModel.PixelBoxYSetting - 1]),
                StringToInt(settings[ProcessFeatureModel.PixelBoxWidthSetting - 1]),
                StringToInt(settings[ProcessFeatureModel.PixelBoxHeightSetting - 1]));
            MinHeat = StringToNonNegInt(settings[ProcessFeatureModel.MinHeatSetting - 1]);
            MaxHeat = StringToNonNegInt(settings[ProcessFeatureModel.MaxHeatSetting - 1]);
            // NumHotPixelsSetting
            // DensityPercSetting 
            // PixelDensityGoodSetting 
            // Leg
        }
    }
}
