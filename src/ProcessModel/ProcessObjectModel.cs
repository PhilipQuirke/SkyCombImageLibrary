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


        // Maximum heat value of any pixel in this object in any frame
        public int MaxHeat { get; set; }
        // Maximum NumHotPixels associated with real features claimed by this object.
        public int MaxNumRealHotPixels { get; set; }
        // Maximum SumHotPixels associated with real features claimed by this object.
        public int MaxSumRealHotPixels { get; set; }
        // Maximum number of pixels with heat 255 (max possible) associated with real features claimed by this object.
        public int MaxNumMaxHeatPixels { get; set; }
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
            DemM = 0;
            Attributes = "";
            Significant = false;
            NumSigBlocks = 0;
            // LegId

            LastRealFeatureId = UnknownValue;
            BeingTracked = true;
            MaxHeat = 0;
            MaxNumRealHotPixels = 0;
            MaxSumRealHotPixels = 0;
            MaxNumMaxHeatPixels = 0;
            MaxRealPixelWidth = 0;
            MaxRealPixelHeight = 0;
        }


        public void SetName(string flightLegName, int positionInLeg)
        {
            if (Name != null && Name.Length > 0 && Name.ToCharArray()[0] != '#')
            {
                Assert(positionInLeg != UnknownValue, "SetName: Bad logic");
                return; // Name already set (e.g. C5)
            }

            Name = flightLegName + positionInLeg.ToString(); // e.g. C5

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
        public const int DemMSetting = 17;
        public const int LegIdSetting = 18;
        public const int AttributesSetting = 19;
        public const int SignificantSetting = 20;
        public const int NumSigBlocksSetting = 21;
        public const int MaxHeatSetting = 22;
        public const int MaxNumRealHotPixelsSetting = 23;
        public const int MaxSumRealHotPixelsSetting = 24;
        public const int MaxNumMaxHeatPixelsSetting = 25;
        public const int MaxRealPixelWidthSetting = 26;
        public const int MaxRealPixelHeightSetting = 27;
        public const int MaxSpinePixelsSetting = 28;
        public const int MaxGirthPixelsSetting = 29;
        public const int SpineCMSetting = 30;
        public const int GirthCMSetting = 31;


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
                { "Dem M", DemM, ElevationNdp },
                { "Leg", (FlightLegId == UnknownValue ? 0 : FlightLegId) },
                { "Attributes", Attributes },
                { "Significant", Significant },
                { "# Sig Blocks", NumSigBlocks },
                { "Max Heat", MaxHeat},
                { "Max Num Real Hot Pxs", MaxNumRealHotPixels },
                { "Max Sum Real Hot Pxs", MaxSumRealHotPixels },
                { "Max Num MaxHeat Pxs", MaxNumMaxHeatPixels },
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
            // Convert int settings in batch for speed  
            var intSettings = new int[15];
            var intIndices = new[] {
                ObjectIdSetting - 1,
                SizeCM2Setting - 1,
                AvgRangeMSetting - 1,
                LegIdSetting - 1,
                NumSigBlocksSetting - 1,
                MaxHeatSetting - 1,
                MaxNumRealHotPixelsSetting - 1,
                MaxSumRealHotPixelsSetting - 1,
                MaxNumMaxHeatPixelsSetting - 1,
                MaxRealPixelWidthSetting - 1,
                MaxRealPixelHeightSetting - 1,
                SpineCMSetting - 1,
                GirthCMSetting - 1 };
            var intInputs = intIndices.Select(i => settings[i]).ToArray();
            ConfigBase.ConvertStringBatch(intInputs, intSettings);

            // Convert float settings in batch for speed  
            var floatSettings = new float[13];
            var floatIndices = new[] {
                FromSecSetting - 1,
                ToSecSetting - 1,
                NorthingMSetting - 1,
                EastingMSetting - 1,
                LocationErrMSetting - 1,
                AvgSumLinealMSetting - 1,
                HeightMSetting - 1,
                MinHeightMSetting - 1,
                MaxHeightMSetting - 1,
                HeightErrMSetting - 1,
                DemMSetting - 1,
                MaxSpinePixelsSetting - 1,
                MaxGirthPixelsSetting - 1 };
            var floatInputs = floatIndices.Select(i => settings[i]).ToArray();
            ConfigBase.ConvertStringBatch(floatInputs, floatSettings);


            ObjectId = intSettings[0];
            Name = settings[NameSetting - 1];
            RunFromVideoS = floatSettings[0];
            RunToVideoS = floatSettings[1];
            LocationM = new DroneLocation(floatSettings[2], floatSettings[3]);
            LocationErrM = floatSettings[4];
            AvgSumLinealM = floatSettings[5];
            HeightM = floatSettings[6];
            MinHeightM = floatSettings[7];
            MaxHeightM = floatSettings[8];
            HeightErrM = floatSettings[9];
            SizeCM2 = intSettings[1];
            AvgRangeM = intSettings[2];
            DemM = floatSettings[10];
            FlightLegId = intSettings[3];
            Attributes = settings[AttributesSetting - 1];
            Significant = (settings[SignificantSetting - 1] == "true");
            NumSigBlocks = intSettings[4];
            MaxHeat = intSettings[5];
            MaxNumRealHotPixels = intSettings[6];
            MaxSumRealHotPixels = intSettings[7];
            MaxNumMaxHeatPixels = intSettings[8];
            MaxRealPixelWidth = intSettings[9];
            MaxRealPixelHeight = intSettings[10];
            MaxSpinePixels = floatSettings[11];
            MaxGirthPixels = floatSettings[12];
            SpineCM = intSettings[11];
            GirthCM = intSettings[12];

            if (HeightM == UnknownHeight) HeightM = UnknownValue;
            if (MinHeightM == UnknownHeight) MinHeightM = UnknownValue;
            if (MaxHeightM == UnknownHeight) MaxHeightM = UnknownValue;
            if (HeightErrM == UnknownHeight) HeightErrM = UnknownValue;
        }


        // Unit test to ensure that GetSettings and LoadSettings form a consistent pair.
        public static void TestSettingsPair()
        {
            var rand = new Random();
            var obj = new ProcessObjectModel
            {
                ObjectId = rand.Next(1, 10000),
                Name = "TestObj" + rand.Next(1, 10000),
                RunFromVideoS = (float)rand.NextDouble() * 1000,
                RunToVideoS = (float)rand.NextDouble() * 1000,
                LocationM = new DroneLocation(rand.Next(-1000, 1000), rand.Next(-1000, 1000)),
                LocationErrM = (float)rand.NextDouble() * 100,
                AvgSumLinealM = (float)rand.NextDouble() * 100,
                HeightM = (float)rand.NextDouble() * 100,
                MinHeightM = (float)rand.NextDouble() * 100,
                MaxHeightM = (float)rand.NextDouble() * 100,
                HeightErrM = (float)rand.NextDouble() * 10,
                SizeCM2 = rand.Next(1, 10000),
                MaxSpinePixels = (float)rand.NextDouble() * 100,
                MaxGirthPixels = (float)rand.NextDouble() * 100,
                SpineCM = rand.Next(1, 1000),
                GirthCM = rand.Next(1, 1000),
                AvgRangeM = rand.Next(1, 10000),
                DemM = (float)rand.NextDouble() * 1000,
                Attributes = "Attr" + rand.Next(1, 10000),
                Significant = rand.Next(0, 2) == 1,
                NumSigBlocks = rand.Next(0, 100),
                FlightLegId = rand.Next(0, 52),
                LastRealFeatureId = rand.Next(0, 10000),
                BeingTracked = rand.Next(0, 2) == 1,
                MaxHeat = rand.Next(0, 255),
                MaxNumRealHotPixels = rand.Next(0, 10000),
                MaxSumRealHotPixels = rand.Next(0, 100000),
                MaxNumMaxHeatPixels = rand.Next(0, 10000),
                MaxRealPixelWidth = rand.Next(0, 1000),
                MaxRealPixelHeight = rand.Next(0, 1000)
            };
            // Save settings to list
            var settings = obj.GetSettings().Select(dp => dp.Value.ToString()).ToList();
            // Create a new object and load settings
            var obj2 = new ProcessObjectModel();
            obj2.LoadSettings(settings);
            // Compare all relevant properties
            Assert(obj.ObjectId == obj2.ObjectId, "ObjectId mismatch");
            Assert(obj.Name == obj2.Name, "Name mismatch");
            Assert(Math.Abs(obj.RunFromVideoS - obj2.RunFromVideoS) < 0.05, "RunFromVideoS mismatch");
            Assert(Math.Abs(obj.RunToVideoS - obj2.RunToVideoS) < 0.05, "RunToVideoS mismatch");
            Assert(Math.Abs(obj.LocationM.NorthingM - obj2.LocationM.NorthingM) < 0.05, "LocationM.NorthingM mismatch");
            Assert(Math.Abs(obj.LocationM.EastingM - obj2.LocationM.EastingM) < 0.05, "LocationM.EastingM mismatch");
            Assert(Math.Abs(obj.LocationErrM - obj2.LocationErrM) < 0.05, "LocationErrM mismatch");
            Assert(Math.Abs(obj.AvgSumLinealM - obj2.AvgSumLinealM) < 0.05, "AvgSumLinealM mismatch");
            Assert(Math.Abs(obj.HeightM - obj2.HeightM) < 0.05, "HeightM mismatch");
            Assert(Math.Abs(obj.MinHeightM - obj2.MinHeightM) < 0.05, "MinHeightM mismatch");
            Assert(Math.Abs(obj.MaxHeightM - obj2.MaxHeightM) < 0.05, "MaxHeightM mismatch");
            Assert(Math.Abs(obj.HeightErrM - obj2.HeightErrM) < 0.05, "HeightErrM mismatch");
            Assert(Math.Abs(obj.SizeCM2 - obj2.SizeCM2) < 0.05, "SizeCM2 mismatch");
            Assert(Math.Abs(obj.AvgRangeM - obj2.AvgRangeM) < 0.05, "AvgRangeM mismatch");
            Assert(Math.Abs(obj.DemM - obj2.DemM) < 0.05, "DemM mismatch");
            Assert(obj.FlightLegId == obj2.FlightLegId, "FlightLegId mismatch");
            Assert(obj.Attributes == obj2.Attributes, "Attributes mismatch");
            Assert(obj.Significant == obj2.Significant, "Significant mismatch");
            Assert(obj.NumSigBlocks == obj2.NumSigBlocks, "NumSigBlocks mismatch");
            Assert(obj.MaxHeat == obj2.MaxHeat, "MaxHeat mismatch");
            Assert(obj.MaxNumRealHotPixels == obj2.MaxNumRealHotPixels, "MaxNumRealHotPixels mismatch");
            Assert(obj.MaxSumRealHotPixels == obj2.MaxSumRealHotPixels, "MaxSumRealHotPixels mismatch");
            Assert(obj.MaxNumMaxHeatPixels == obj2.MaxNumMaxHeatPixels, "MaxNumMaxHeatPixels mismatch");
            Assert(obj.MaxRealPixelWidth == obj2.MaxRealPixelWidth, "MaxRealPixelWidth mismatch");
            Assert(obj.MaxRealPixelHeight == obj2.MaxRealPixelHeight, "MaxRealPixelHeight mismatch");
            Assert(Math.Abs(obj.MaxSpinePixels - obj2.MaxSpinePixels) < 0.05, "MaxSpinePixels mismatch");
            Assert(Math.Abs(obj.MaxGirthPixels - obj2.MaxGirthPixels) < 0.05, "MaxGirthPixels mismatch");
            Assert(obj.SpineCM == obj2.SpineCM, "SpineCM mismatch");
            Assert(obj.GirthCM == obj2.GirthCM, "GirthCM mismatch");
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
                HeightErrColor = (HeightErrM < ProcessConfigModel.GoodHeightErrM ? "good" : "bad");

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
