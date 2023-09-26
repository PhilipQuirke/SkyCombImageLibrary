// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombImage.CategorySpace;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;


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
        public string Name { get; set; }

        // Time this object was first seen
        public float RunFromVideoS { get; set; }
        // Time this object was last seen
        public float RunToVideoS { get; set; }


        // Location (centroid) of this object relative to the drone's starting point.
        public DroneLocation LocationM { get; set; }
        // Estimated error in object location, based on variance in child Feature locations.
        public float LocationErrM { get; set; }
        // The drone SumLinealM distance corresponding to the centroid of the object
        public float AvgSumLinealM { get; set; }


        // Best estimate of Height of the object above ground level in metres (not Altitude above sea level).
        // Calculated using "trig look down" method based on first and last real features. 
        public float HeightM { get; set; }
        // Minimum estimate of Height of the object above ground level in metres 
        public float MinHeightM { get; set; }
        // Maximum estimate of Height of the object above ground level in metres 
        public float MaxHeightM { get; set; }
        // The longer the "baseline" the drone travels, while viewing the object,
        // the more accurate the HeightM calculation. So we measure height range 
        // and error based on the last 8 frame calculations. The "8" is arbitrary.
        // It is > 1 in case there is some drone turbulence or "end of leg" yawing 
        // in the data. For DJI Mavic 8 is equivalent to 1 second.
        public const int NumRealFeaturesForHeightErr = 8;
        // The variation in feature estimates of HeightM in metres over last 8 calcs
        public float HeightErrM { get; set; }


        // Best estimate of the size of the object in meters squared.
        // Based on MAXIMUM number of hot pixels in any real feature.
        // If object is in a tree and partially obscured may be an underestimate. 
        // Don't keep a estimate of error in SizeCM2, as MINIMUM hot pixels is NOT a sensible error estimate.
        public float SizeCM2 { get; set; }


        // The average distance from the object to the drone in meters
        public int AvgRangeM { get; set; }


        // Maximum heat value of any pixel in this object in any frame
        public int MaxHeat { get; set; }


        // Average height of the ground below the object (in meters above sea level)
        public float DemM { get; set; }
        // Significance attributes about this object
        public string Attributes { get; set; }
        // Is this a significant object?
        public bool Significant { get; set; }
        // Number of blocks this object was a significant object. Objects can become insignificant, after being significant.
        public int NumSigBlocks { get; set; }
        // LegId of the FlightLeg (if any) this object was found during
        public int FlightLegId { get; set; }
        public string FlightLegName { get { return IdToLetter(FlightLegId); } }


        public ProcessObjectModel()
        {
            FlightLegId = 0;
            Name = "";
            RunFromVideoS = 0.0F;
            RunToVideoS = 0.0F;
            ResetMemberData();
        }


        // Reset member data to mirror a newly created object.
        // Used in experimentation to allow repeated calculation run against this object.
        public virtual void ResetMemberData()
        {
            // ObjectId
            // Name
            // RunFromVideoS 
            // RunToVideoS 
            LocationM = new DroneLocation();
            LocationErrM = UnknownValue;
            AvgSumLinealM = UnknownValue;
            HeightM = UnknownValue;
            MinHeightM = UnknownValue;
            MaxHeightM = UnknownValue;
            HeightErrM = UnknownValue;
            SizeCM2 = 0;
            AvgRangeM = UnknownValue;
            MaxHeat = 0;
            DemM = 0;
            Attributes = "";
            Significant = false;
            NumSigBlocks = 0;
            // LegId
        }


        public void SetName(int positionInLeg = UnknownValue)
        {
            if (Name != null && Name.Length > 0 && Name.ToCharArray()[0] != '#')
            {
                Assert(positionInLeg != UnknownValue, "SetName: Bad logic");
                return; // Name already set (e.g. C5)
            }

            if (positionInLeg == UnknownValue)
            {
                string mapping = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
                char[] resultChars = new char[5];
                for (int i = 4; i >= 0; i--)
                    resultChars[i] = (char)0;

                var number = ObjectId;
                for (int i = 4; i >= 0; i--)
                {
                    int index = number % 26;
                    resultChars[i] = mapping[index];
                    number /= 26;
                    if(number == 0)
                        break;
                }

                Name = "#" + resultChars;
            }
            else
                Name = FlightLegName + positionInLeg.ToString(); // e.g. C5
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
        // CombObject additional settings
        public const int FirstBlockSetting = 23;
        public const int CenterBlockSetting = 24;
        public const int LastRealBlockSetting = 25;
        public const int LastBlockSetting = 26;
        public const int MaxRealHotPixelsSetting = 27;
        public const int MaxRealPixelWidthSetting = 28;
        public const int MaxRealPixelHeightSetting = 29;
        public const int FirstFwdDownDegSetting = 30;
        public const int LastFwdDownDegSetting = 31;
        public const int RangeFwdDownDegSetting = 32;
        public const int NumRealFeaturesSetting = 33;
        public const int RealDensityPxSetting = 34;
        public const int LocnErrPerFeatCMSetting = 35;


        public int UnknownHeight = -2;


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
                { "Min Hght M", MinHeightM, HeightNdp },
                { "Max Hght M", MaxHeightM, HeightNdp },
                { "Hght Err M", HeightErrM, HeightNdp },
                { "Hght Rnd M", (HeightM == UnknownValue ? UnknownHeight : ((int)(HeightM * 2)) / 2), HeightNdp },
                { "Size CM2", SizeCM2, AreaCM2Ndp },
                { "Size Rnd CM2", (SizeCM2 == UnknownValue ? UnknownHeight : ((int)(SizeCM2 / 100)) * 100), AreaCM2Ndp },
                { "Avg Range M", AvgRangeM },
                { "Max Heat", MaxHeat}, 
                { "Dem M", DemM, HeightNdp },
                { "Leg", (FlightLegId == UnknownValue ? 0 : FlightLegId) },
                { "Attributes", Attributes },
                { "Significant", Significant },
                { "# Sig Blocks", NumSigBlocks },
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
            if (HeightM == UnknownHeight)
                HeightM = UnknownValue;

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
        }


        // Zero-based grid-data settings index values.   
        public const int GridObjectNameSetting = 0;
        public const int GridCategorySetting = 1;
        public const int GridIncludeSetting = 2;
        public const int GridHeightMSetting = 3;
        public const int GridSizeCM2Setting = 4;
        public const int GridFromSecSetting = 5;
        public const int GridForSecSetting = 6;
        public const int GridLocationMSetting = 7;
        public const int GridAttributesSetting = 8;
        public const int GridLocationErrMSetting = 9;
        public const int GridHeightErrMSetting = 10;
        public const int GridLocationGoodSetting = 11; // Used for coloring cell
        public const int GridHeightGoodSetting = 12; // Used for coloring cell


        // Get object's settings shown in the object grid & object form
        public DataPairList GetSettings_Grid(ObjectCategoryModel annotation)
        {
            var category = (annotation == null ? "" : annotation.Category);
            var include = (annotation == null ? "Yes" : (annotation.Include ? "Yes" : "No"));

            return new DataPairList
            {
                { "Object", Name },
                { "Category", category },
                { "Include", include },
                { "Height (cm)", (HeightM==UnknownValue ? UnknownValue : HeightM * 100), 0 },
                { "Size (cm2)", SizeCM2, 0 },
                { "From (m:ss)", VideoModel.DurationSecToString(RunFromVideoS, 1) },
                { "For (s)", RunToVideoS - RunFromVideoS, 1 },
                { "Location (m,m)", (LocationM != null ? LocationM.ToString() : "" ) },
                { "Attributes", Attributes },
                { "Location Err (cm)", (LocationErrM==UnknownValue ? UnknownValue : LocationErrM * 100), 0 },
                { "Height Err (cm)", (HeightErrM==UnknownValue ? UnknownValue : HeightErrM * 100), 0 },
            };
        }


        // Return the details of a significant object to display in the ObjectGrid in MainForm or ObjectForm
        public object[] GetObjectGridData(ProcessConfigModel config, bool mainForm, ObjectCategoryModel annotation)
        {
            // Use GetSettings to get the same formatting as used in the DataStore.
            var settings_grid = GetSettings_Grid(annotation);

            if (mainForm)
                return new object[] {
                    settings_grid[GridObjectNameSetting].Value,
                    settings_grid[GridCategorySetting].Value,
                    settings_grid[GridIncludeSetting].Value,
                    settings_grid[GridHeightMSetting].Value,
                    settings_grid[GridSizeCM2Setting].Value,
                    settings_grid[GridFromSecSetting].Value,
                    settings_grid[GridForSecSetting].Value,
                    settings_grid[GridLocationMSetting].Value,
                    settings_grid[GridAttributesSetting].Value,
                    settings_grid[GridLocationErrMSetting].Value,
                    settings_grid[GridHeightErrMSetting].Value,

                    // We color the Location cell is "good", "bad" or leave white for neutral.
                    ( LocationErrM < config.GoodLocationErrM ? "good" : ( LocationErrM > config.BadLocationErrM ? "bad" : "" ) ),
                    // We color the Height cell is "good", "bad" or leave white for neutral.
                    ( HeightErrM < config.GoodHeightErrM ? "good" : ( HeightErrM > config.BadHeightErrM ? "bad" : "" ) ),
                };
            else
                // Return data for the ObjectList in the ObjectForm
                return new object[] {
                    settings_grid[GridObjectNameSetting].Value,
                    settings_grid[GridCategorySetting].Value,
                    settings_grid[GridIncludeSetting].Value,
                    (annotation == null ? "" : annotation.Notes)
            };
        }
    }
}
