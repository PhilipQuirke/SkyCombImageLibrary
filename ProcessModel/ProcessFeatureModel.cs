using SkyCombGround.CommonSpace;
using System.Collections.Generic;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // A class to hold a significant feature 
    public class ProcessFeatureModel : ConfigBase
    {
        // Unique identifier
        public int FeatureId { get; }
        // Is this feature actively being tracked now
        public bool IsTracked { get; set; }
        // Is this feature significant?
        public bool Significant { get; set; }
        // Attributes about this feature
        public string Attributes { get; set; }
        // A feature can be associated with an object
        public int ObjectId { get; set; }


        public ProcessFeatureModel(int featureId)
        {
            FeatureId = featureId;
        }


        // Reset member data to mirror a newly created feature.
        // Used in experimentation to allow repeated calculation run against this feature.
        public virtual void ResetMemberData()
        {
            IsTracked = true;
            Significant = false;
            Attributes = "";
            ObjectId = 0;
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int FeatureIdSetting = 1;
        public const int IsTrackedSetting = 2;
        public const int SignificantSetting = 3;
        public const int NotesSetting = 4;
        public const int ObjectIdSetting = 5;
        // CombFeature additional settings
        public const int BlockIdSetting = 6;
        public const int TypeSetting = 7;
        public const int NorthingMSetting = 8;
        public const int EastingMSetting = 9;
        public const int HeightMSetting = 10;
        public const int PixelBoxXSetting = 11;
        public const int PixelBoxYSetting = 12;
        public const int PixelBoxWidthSetting = 13;
        public const int PixelBoxHeightSetting = 14;
        public const int MinHeatSetting = 15;
        public const int MaxHeatSetting = 16;
        public const int NumHotPixelsSetting = 17;
        public const int DensityPercSetting = 18;
        public const int PixelDensityGoodSetting = 19;
        public const int LegIdSetting = 20;


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
            };
        }


        virtual public void LoadSettings(List<string> settings)
        {
            // FeatureId (already done) = settings[0]
            IsTracked = settings[1] == "true";
            Significant = settings[2] == "true";
            Attributes = settings[3];
            ObjectId = StringToNonNegInt(settings[4]);
        }
    }
}
