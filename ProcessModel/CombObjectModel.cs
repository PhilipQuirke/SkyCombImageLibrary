using SkyCombGround.CommonSpace;
using System.Collections.Generic;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // A significant Comb object - a logical object derived from overlapping features over successive frames. 
    public class CombObjectModel : ConfigBase
    {
        // Last Real feature claimed by this object (excluding Consumed features).
        public int LastRealFeatureIndex { get; set; }

        // Is this object still being actively tracked?
        public bool BeingTracked { get; set; }

        // Average velocity of object. Used to calculate expected position in next block.
        public VelocityF AverageVelocityinPixelsPerBlock { get; set; }

        // Maximum NumHotPixels associated with real features claimed by this object.
        public int MaxRealHotPixels { get; set; }
        // Maximum Width of the object pixel box over real Features
        public int MaxRealPixelWidth { get; set; }
        // Maximum Height of the object pixel box over real Features
        public int MaxRealPixelHeight { get; set; }

        // First angle down from drone to object in direct of drone's flight path in degrees
        public float FirstFwdDownDeg { get; set; } = UnknownValue;
        // Last angle down from drone to object in direct of drone's flight path in degrees
        public float LastFwdDownDeg { get; set; } = UnknownValue;


        // Constructor used when loaded objects from the datastore
        public CombObjectModel(List<string> settings = null)
        {
            ResetMemberData();

            if (settings != null)
                LoadSettings(settings);
        }


        // Reset member data to mirror a newly created object.
        // Used in experimentation to allow repeated calculation run against this object.
        public void ResetMemberData()
        {
            LastRealFeatureIndex = UnknownValue;
            BeingTracked = true;
            AverageVelocityinPixelsPerBlock = null;
            MaxRealHotPixels = 0;
            MaxRealPixelWidth = 0;
            MaxRealPixelHeight = 0;
            FirstFwdDownDeg = UnknownValue;
            LastFwdDownDeg = UnknownValue;
        }



        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        public void GetSettings(DataPairList settings)
        {
            settings.Add("Speed Px", (AverageVelocityinPixelsPerBlock != null ? AverageVelocityinPixelsPerBlock.Speed() : 0), PixelVelNdp);
            settings.Add("X Vel Px", (AverageVelocityinPixelsPerBlock != null ? AverageVelocityinPixelsPerBlock.Value.X : 0), PixelVelNdp);
            settings.Add("Y VelPx", (AverageVelocityinPixelsPerBlock != null ? AverageVelocityinPixelsPerBlock.Value.Y : 0), PixelVelNdp);
            settings.Add("Max Real Hot Pxs", MaxRealHotPixels);
            settings.Add("Max Real Px Width", MaxRealPixelWidth);
            settings.Add("Max Real Px Height", MaxRealPixelHeight);
            settings.Add("First Fwd Down Deg", FirstFwdDownDeg, DegreesNdp);
            settings.Add("Last Fwd Down Deg", LastFwdDownDeg, DegreesNdp);
            settings.Add("Range Fwd Down Deg", FirstFwdDownDeg - LastFwdDownDeg, DegreesNdp);
        }


        public void LoadSettings(List<string> settings)
        {
            // SpeedPx = setting[ProcessObjectModel.SpeedInPxPerBlockSetting-1]
            AverageVelocityinPixelsPerBlock = new(
                StringToFloat(settings[ProcessObjectModel.XVelInPxPerBlockSetting - 1]),
                StringToFloat(settings[ProcessObjectModel.YVelInPxPerBlockSetting - 1]));
            MaxRealHotPixels = StringToInt(settings[ProcessObjectModel.MaxRealHotPixelsSetting - 1]);
            MaxRealPixelWidth = StringToInt(settings[ProcessObjectModel.MaxRealPixelWidthSetting - 1]);
            MaxRealPixelHeight = StringToInt(settings[ProcessObjectModel.MaxRealPixelHeightSetting - 1]);
            FirstFwdDownDeg = StringToFloat(settings[ProcessObjectModel.FirstFwdDownDegSetting - 1]);
            LastFwdDownDeg = StringToFloat(settings[ProcessObjectModel.LastFwdDownDegSetting - 1]);
            // RangeFwdDownDeg = setting[ProcessObjectModel.RangeFwdDownDegSetting-1]
        }
    }
}
