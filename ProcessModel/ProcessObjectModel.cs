// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // A logical object derived from overlapping features over successive frames. 
    public class ProcessObjectModel : ConfigBase
    {

        // Unique identifier
        public int ObjectId { get; set; }
        // Defaults to ObjectId (e.g. #15), but for signficant objects
        // is set to C5 meaning 5th significant object in Leg C.
        public string Name { get; set; } = "";

        // Time this object was first seen
        public float RunFromVideoS { get; set; } = 0.0F;
        // Time this object was last seen
        public float RunToVideoS { get; set; } = 0.0F;


        // Location (centroid) of this object relative to the drone's starting point.
        public DroneLocation? LocationM { get; set; } = null;
        // Estimated error in object location, based on variance in child Feature locations.
        public float LocationErrM { get; set; }
        // The drone SumLinealM distance corresponding to the centroid of the object
        public float AvgSumLinealM { get; set; }


        // Best estimate of Height of the object above ground level in metres (not Altitude above sea level).
        public float HeightM { get; set; }
        // Minimum estimate of Height of the object above ground level in metres 
        public float MinHeightM { get; set; }
        // Maximum estimate of Height of the object above ground level in metres 
        public float MaxHeightM { get; set; }
        // The variation in feature estimates of HeightM in metres 
        public float HeightErrM { get; set; }


        // Best estimate of the size of the object in centimeters squared.
        public float SizeCM2 { get; set; }
        // Maximum object "spine" pixel length over all real features
        public float MaxSpinePixels { get; set; }
        // Maximum object "girth" pixel (at right angles to spine) length over all real features
        public float MaxGirthPixels { get; set; }
        // Object spine length in CM
        public int SpineCM { get; set; }
        // Object girth length in CM
        public int GirthCM { get; set; }


        // The average distance from the object to the drone in meters
        public int AvgRangeM { get; set; }


        // Maximum heat value of any pixel in this object in any frame
        public int MaxHeat { get; set; }


        // Average height of the ground below the object (in meters above sea level)
        public float DemM { get; set; }
        // Significance attributes about this object
        public string Attributes { get; set; } = "";
        // Is this a significant object?
        public bool Significant { get; set; }
        // Number of blocks this object was a significant object. Objects can become insignificant, after being significant.
        public int NumSigBlocks { get; set; }
        // LegId of the FlightLeg (if any) this object was found during
        public int FlightLegId { get; set; } = 0;
        public string FlightLegName { get { return IdToLetter(FlightLegId); } }

        // Last Real feature claimed by this object (excluding Consumed features).
        public int LastRealFeatureId { get; set; }

        // Is this object still being actively tracked?
        public bool BeingTracked { get; set; }

        // Maximum NumHotPixels associated with real features claimed by this object.
        public int MaxRealHotPixels { get; set; }
        // Maximum Width of the object pixel box over real Features
        public int MaxRealPixelWidth { get; set; }
        // Maximum Height of the object pixel box over real Features
        public int MaxRealPixelHeight { get; set; }


        public ProcessObjectModel()
        {
            FlightLegId = 0;
            Name = "";
            RunFromVideoS = 0.0F;
            RunToVideoS = 0.0F;
            ResetCalcedMemberData();
        }


        // Reset "calculated" member data to mirror a newly created object.
        // Used in experimentation to allow repeated calculation run against this object.
        public virtual void ResetCalcedMemberData()
        {
            // ObjectId
            // Name
            // RunFromVideoS 
            // RunToVideoS 
            LocationM = new DroneLocation();
            LocationErrM = UnknownValue;
            AvgSumLinealM = UnknownValue;
            HeightM = ProcessObjectModel.UnknownHeight;
            MinHeightM = ProcessObjectModel.UnknownHeight;
            MaxHeightM = ProcessObjectModel.UnknownHeight;
            HeightErrM = UnknownValue;
            SizeCM2 = 0;
            MaxSpinePixels = 0;
            MaxGirthPixels = 0;
            SpineCM = 0;
            GirthCM = 0;
            AvgRangeM = UnknownValue;
            MaxHeat = 0;
            DemM = 0;
            Attributes = "";
            Significant = false;
            NumSigBlocks = 0;
            // LegId

            LastRealFeatureId = UnknownValue;
            BeingTracked = true;
            MaxRealHotPixels = 0;
            MaxRealPixelWidth = 0;
            MaxRealPixelHeight = 0;
        }


        public void SetName(int positionInLeg = UnknownValue)
        {
            if (Name != null && Name.Length > 0 && Name.ToCharArray()[0] != '#')
            {
                Assert(positionInLeg != UnknownValue, "SetName: Bad logic");
                return; // Name already set (e.g. C5)
            }

            if (positionInLeg != UnknownValue)
                Name = FlightLegName + positionInLeg.ToString(); // e.g. C5
            else if (ObjectId == 0)
                Name = "A";
            else
            {
                const int baseValue = 26;
                int number = ObjectId;

                Name = "";
                while (number > 0)
                {
                    int remainder = (number - 1) % baseValue;
                    char letter = (char)('A' + remainder);
                    Name = letter + Name;
                    number = (number - 1) / baseValue;
                }
            }

            Assert(Name.Length < 10, "ProcessObjectModel.SetName: Bad length");
        }


        // One-based settings index values. Must align with GetSettings procedure below    
        public const int ObjectIdSetting = 1;
        public const int NameSetting = 2;
        public const int FromSecSetting = 3;
        public const int ToSecSetting = 4;
        public const int NorthingMSetting = 5;
        public const int EastingMSetting = 6;
        public const int LocationErrMSetting = 7;
        public const int AvgSumLinealMSetting = 8;
        public const int HeightMSetting = 9;
        public const int MinHeightMSetting = 10;
        public const int MaxHeightMSetting = 11;
        public const int HeightErrMSetting = 12;
        public const int HeightRndMSetting = 13;
        public const int SizeCM2Setting = 14;
        public const int SizeRndCM2Setting = 15;
        public const int AvgRangeMSetting = 16;
        public const int MaxHeatSetting = 17;
        public const int DemMSetting = 18;
        public const int LegIdSetting = 19;
        public const int AttributesSetting = 20;
        public const int SignificantSetting = 21;
        public const int NumSigBlocksSetting = 22;
        public const int FirstBlockSetting = 23;
        public const int CenterBlockSetting = 24;
        public const int LastRealBlockSetting = 25;
        public const int LastBlockSetting = 26;
        public const int MaxRealHotPixelsSetting = 27;
        public const int MaxRealPixelWidthSetting = 28;
        public const int MaxRealPixelHeightSetting = 29;
        public const int MaxSpinePixelsSetting = 30;
        public const int MaxGirthPixelsSetting = 31;
        public const int SpineCMSetting = 32;
        public const int GirthCMSetting = 33;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public virtual DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Object", ObjectId },
                { "Name", Name },
                { "From S", RunFromVideoS, SecondsNdp },
                { "To S", RunToVideoS, SecondsNdp },
                { "Northing M", (LocationM != null ? LocationM.NorthingM : 0 ), LocationNdp },
                { "Easting M", (LocationM != null ? LocationM.EastingM : 0 ), LocationNdp },
                { "Locn Err M", LocationErrM, LocationNdp },
                { "Lineal M", AvgSumLinealM, LocationNdp },
                { "Height M", (HeightM == UnknownValue ? UnknownHeight : HeightM), HeightNdp },
                { "Min Hght M", (MinHeightM == UnknownValue ? UnknownHeight : MinHeightM), HeightNdp }, 
                { "Max Hght M", (MaxHeightM == UnknownValue ? UnknownHeight : MaxHeightM), HeightNdp }, 
                { "Hght Err M", (HeightErrM == UnknownValue? UnknownHeight : HeightErrM), HeightNdp }, 
                { "Hght Rnd M", (HeightM == UnknownValue ? UnknownHeight : ((int)(HeightM * 2f)) / 2f), HeightNdp },
                { "Size CM2", SizeCM2, AreaCM2Ndp },
                { "Size Rnd CM2", (SizeCM2 == UnknownValue ? UnknownHeight : ((int)(SizeCM2 / 100f)) * 100), AreaCM2Ndp },
                { "Avg Range M", AvgRangeM },
                { "Max Heat", MaxHeat},
                { "Dem M", DemM, ElevationNdp },
                { "Leg", (FlightLegId == UnknownValue ? 0 : FlightLegId) },
                { "Attributes", Attributes },
                { "Significant", Significant },
                { "# Sig Blocks", NumSigBlocks },
                { "Max Real Hot Pxs", MaxRealHotPixels },
                { "Max Real Px Width", MaxRealPixelWidth },
                { "Max Real Px Height", MaxRealPixelHeight },
                { "Max Spine Pxs", MaxSpinePixels, 2 },
                { "Max Girth Pxs", MaxGirthPixels, 2 },
                { "Spine CM", SpineCM },
                { "Girth CM", GirthCM },
            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        protected virtual void LoadSettings(List<string> settings)
        {
            int i = 0;
            ObjectId = StringToNonNegInt(settings[i++]);
            Name = settings[i++];
            RunFromVideoS = StringToNonNegFloat(settings[i++]);
            RunToVideoS = StringToNonNegFloat(settings[i++]);
            LocationM = new DroneLocation(settings[i++], settings[i++]);
            LocationErrM = StringToNonNegFloat(settings[i++]);
            AvgSumLinealM = StringToNonNegFloat(settings[i++]);
            HeightM = StringToFloat(settings[i++]); 
            MinHeightM = StringToFloat(settings[i++]);
            MaxHeightM = StringToFloat(settings[i++]);
            HeightErrM = StringToFloat(settings[i++]);
            i++; // HghtRndM
            SizeCM2 = StringToNonNegInt(settings[i++]);
            i++; // SizeRndCM2
            AvgRangeM = StringToInt(settings[i++]);
            MaxHeat = StringToNonNegInt(settings[i++]);
            DemM = StringToFloat(settings[i++]);
            FlightLegId = StringToNonNegInt(settings[i++]);
            Attributes = settings[i++];
            Significant = (settings[i++] == "true");
            NumSigBlocks = StringToNonNegInt(settings[i++]);
            MaxRealHotPixels = StringToInt(settings[ProcessObjectModel.MaxRealHotPixelsSetting - 1]);
            MaxRealPixelWidth = StringToInt(settings[ProcessObjectModel.MaxRealPixelWidthSetting - 1]);
            MaxRealPixelHeight = StringToInt(settings[ProcessObjectModel.MaxRealPixelHeightSetting - 1]);
            MaxSpinePixels = StringToInt(settings[ProcessObjectModel.MaxSpinePixelsSetting - 1]);
            MaxGirthPixels = StringToInt(settings[ProcessObjectModel.MaxGirthPixelsSetting - 1]);
            SpineCM = StringToInt(settings[ProcessObjectModel.SpineCMSetting - 1]);
            GirthCM = StringToInt(settings[ProcessObjectModel.GirthCMSetting - 1]);

            if (HeightM == UnknownHeight) HeightM = UnknownValue;
            if (MinHeightM == UnknownHeight) MinHeightM = UnknownValue;
            if (MaxHeightM == UnknownHeight) MaxHeightM = UnknownValue;
            if (HeightErrM == UnknownHeight) HeightErrM = UnknownValue;
        }


        // Zero-based grid-data settings index values.   
        public const int GridObjectNameSetting = 0;
        public const int GridCategorySetting = 1;
        public const int GridIncludeSetting = 2;
        public const int GridHeightClassSetting = 3;
        public const int GridSizeClassSetting = 4;
        public const int GridFromSecSetting = 5;
        public const int GridForSecSetting = 6;
        public const int GridLocationMSetting = 7;
        public const int GridAttributesSetting = 8;
        public const int GridLocationErrMSetting = 9;
        public const int GridHeightErrMSetting = 10;
        public const int GridLocationGoodSetting = 11; // Used for coloring cell
        public const int GridHeightGoodSetting = 12; // Used for coloring cell
        public const int GridAnnotationSetting = 13;


        // Get object's settings shown in the object grid & object form
        public DataPairList GetSettings_Grid(ObjectCategoryModel annotation)
        {
            var category = (annotation == null ? "" : annotation.Category);
            var include = (annotation == null ? "Yes" : (annotation.Include ? "Yes" : "No"));
            (var areaClass, var _) = MasterSizeModelList.CM2ToClass(this);
            (var heightClass, var _) = MasterHeightModelList.HeightMToClass(this);

            return new DataPairList
            {
                { "Object", Name },
                { "Category", category },
                { "Include", include },
                { "Height", heightClass },
                { "Size", areaClass },
                { "From (m:ss)", VideoModel.DurationSecToString(RunFromVideoS, 1) },
                { "For (s)", RunToVideoS - RunFromVideoS, 1 },
                { "Location (m,m)", (LocationM != null ? LocationM.NorthingM.ToString("0") + "," + LocationM.EastingM.ToString("0") : "" ) },
                { "Attributes", Attributes },
                { "Location Err (cm)", (LocationErrM<0 ? UnknownValue : LocationErrM * 100), 0 },
                { "Height Err (cm)", (HeightErrM<0 ? UnknownValue : HeightErrM * 100), 0 },
            };
        }


        // Return the details of a significant object to display in the ObjectGrid
        public object[] GetObjectGridData(ObjectCategoryModel annotation)
        {
            // Use GetSettings to get the same formatting as used in the DataStore.
            var settings_grid = GetSettings_Grid(annotation);


            var HeightErrColor = "none";
            if (HeightM != UnknownValue)
                // We color the Height cell is "good", "bad" or leave white for neutral.
                HeightErrColor = (HeightErrM < ProcessConfigModel.GoodHeightErrM ? "good" : "bad" );

            return new object[] {
                settings_grid[GridObjectNameSetting].Value,
                settings_grid[GridCategorySetting].Value,
                settings_grid[GridIncludeSetting].Value,
                settings_grid[GridHeightClassSetting].Value,
                settings_grid[GridSizeClassSetting].Value,
                settings_grid[GridFromSecSetting].Value,
                settings_grid[GridForSecSetting].Value,
                settings_grid[GridLocationMSetting].Value,
                settings_grid[GridAttributesSetting].Value,
                settings_grid[GridLocationErrMSetting].Value,
                settings_grid[GridHeightErrMSetting].Value,

                // We color the Location cell is "good", "bad" or leave white for neutral.
                ( LocationErrM < ProcessConfigModel.GoodLocationErrM ? "good" : "bad" ),
                // We color the Height cell is "good", "bad" or leave white for neutral.
                HeightErrColor,
                (annotation == null ? "" : annotation.Notes),
            };
        }
    }
}
