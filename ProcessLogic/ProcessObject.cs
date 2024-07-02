// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A significant object - a logical object derived from overlapping features over successive frames. 
    public class ProcessObject : ProcessObjectModel
    {
        // Static config data shared by all objects
        public static ProcessConfigModel ProcessConfig;

        // Static NextObjectID shared by all objects
        public static int NextObjectId = 0;
        // Static random number generator
        protected static readonly RNG Rng = new();


        public ProcessObject(ProcessConfigModel processConfig, ProcessScope scope) : base()
        {
            ProcessConfig = processConfig;

            ObjectId = ++NextObjectId;
            if (scope != null)
            {
                FlightLegId = scope.PSM.CurrRunLegId;
                RunFromVideoS = (float)(scope.PSM.CurrInputFrameMs / 1000.0);
                RunToVideoS = RunFromVideoS;
            }
        }


        // Is this object in the RunFrom/To scope?
        public bool InRunScope(ProcessScope scope)
        {
            var maxFromMs = Math.Max(RunFromVideoS * 1000, scope.PSM.FirstVideoFrameMs);
            var minToMs = Math.Min(RunToVideoS * 1000, scope.PSM.LastVideoFrameMs);

            var overlapMs = minToMs - maxFromMs;
            if (overlapMs <= 0)
                return false;

            // To cope with edge cases, returns true if > 50% of the objects duration is within the RunFrom/To scope
            var durationMs = RunToVideoS * 1000 - RunFromVideoS * 1000;
            return overlapMs / durationMs >= 0.5;
        }
    }


    public class ProcessObjList : SortedList<int, ProcessObject>
    {
        // The minimum object LocationErrM value in meters
        public float MinLocationErrM { get; set; }
        // The maximum object LocationErrM value in meters
        public float MaxLocationErrM { get; set; }
        // The sum of object LocationErrM value in meters
        public float SumLocationErrM { get; set; }


        // Minimum estimate of Height of any object above ground level in meters 
        public float MinHeightM { get; set; }
        // Maximum estimate of Height of any object above ground level in meters 
        public float MaxHeightM { get; set; }
        // The minimum object HeightErrM value in meters
        public float MinHeightErrM { get; set; }
        // The maximum object HeightErrM value in meters
        public float MaxHeightErrM { get; set; }
        // The sum of object HeightM value in meters
        public float SumHeightM { get; set; }
        // The sum of object HeightErrM value in meters
        public float SumHeightErrM { get; set; }
        // Average height in meters 
        public float AvgHeightM { get { return (Count == 0 ? 0 : SumHeightM / Count); } }


        // Minimum estimate of size of any object in cm2
        public float MinSizeCM2 { get; set; }
        // Maximum estimate of size of any object in cm2
        public float MaxSizeCM2 { get; set; }


        // Minimum range of any object from the drone in meters
        public int MinRangeM { get; set; }
        // Maximum range of any object from the drone in meters
        public int MaxRangeM { get; set; }


        // Minimum heat of any object
        public int MinHeat { get; set; }
        // Maximum heat of any object
        public int MaxHeat { get; set; }


        public ProcessObjList()
        {
            ResetSettings();
        }


        public void ResetSettings()
        {
            MinLocationErrM = BaseConstants.UnknownValue;
            MaxLocationErrM = BaseConstants.UnknownValue;
            SumLocationErrM = 0;
            MinHeightM = ProcessObjectModel.UnknownHeight;
            MaxHeightM = ProcessObjectModel.UnknownHeight;
            MinHeightErrM = BaseConstants.UnknownValue;
            MaxHeightErrM = BaseConstants.UnknownValue;
            SumHeightM = 0;
            SumHeightErrM = 0;
            MinSizeCM2 = BaseConstants.UnknownValue;
            MaxSizeCM2 = BaseConstants.UnknownValue;
            MinRangeM = BaseConstants.UnknownValue;
            MaxRangeM = BaseConstants.UnknownValue;
            MinHeat = BaseConstants.UnknownValue;
            MaxHeat = BaseConstants.UnknownValue;
        }


        public void AddObject(ProcessObject theObject)
        {
            BaseConstants.Assert(theObject.ObjectId > 0, "ProcessObjList.AddObject: No Id");
            Add(theObject.ObjectId, theObject);
        }


        public int NumSignificantObjects()
        {
            int answer = 0;

            foreach (var theObj in this)
                if (theObj.Value.Significant)
                    answer++;

            return answer;
        }


        public int GetObjectIdByName(string objectName)
        {
            foreach (var theObj in this)
                if (theObj.Value.Name == objectName)
                    return theObj.Value.ObjectId;

            return BaseConstants.UnknownValue;
        }


        // Find and return the closest object to the drone location (within maxDeltaM)
        public ProcessObject? GetObjectByLocationM(DroneLocation droneLocation, int minDeltaM = 1, int maxDeltaM = 10)
        {
            ProcessObject? answer = null;
            double minRange = maxDeltaM;

            foreach (var theObj in this)
            {
                var range = RelativeLocation.DistanceM(theObj.Value.LocationM, droneLocation);
                if (range <= minDeltaM)
                    return theObj.Value;
                else if (range <= minRange)
                {
                    answer = theObj.Value;
                    minRange = range;
                }
            }

            return answer;
        }


        public ProcessObjList FilterByProcessScope(ProcessScope scope, int focusObjectID = BaseConstants.UnknownValue)
        {
            ProcessObjList answer = new();

            foreach (var theObject in this)
                if ((theObject.Value.Significant || (theObject.Value.ObjectId == focusObjectID)) &&
                    // Only return objects in the RunFrom/To scope.
                    theObject.Value.InRunScope(scope))
                    answer.AddObject(theObject.Value);

            if (answer.Count > 0)
                answer.CalculateSettings(answer);

            return answer;
        }


        public ProcessObjList FilterByObjectScope(ObjectDrawScope objectScope)
        {
            ProcessObjList answer = new();

            foreach (var theObject in this)
            {
                var theObj = theObject.Value;

                if ((objectScope.MinHeightM != BaseConstants.UnknownValue))
                {
                    if (theObj.HeightM < objectScope.MinHeightM)
                        continue;
                    if (theObj.HeightM > objectScope.MaxHeightM)
                        continue;
                }
                if ((objectScope.MinRangeM != BaseConstants.UnknownValue))
                {
                    if (theObj.AvgRangeM < objectScope.MinRangeM)
                        continue;
                    if (theObj.AvgRangeM > objectScope.MaxRangeM)
                        continue;
                }
                if ((theObj.SizeCM2 != BaseConstants.UnknownValue) && (objectScope.MinSizeCM2 != BaseConstants.UnknownValue))
                {
                    if (theObj.SizeCM2 < objectScope.MinSizeCM2)
                        continue;
                    if (theObj.SizeCM2 > objectScope.MaxSizeCM2)
                        continue;
                }
                if ((theObj.MaxHeat != BaseConstants.UnknownValue) && (objectScope.MinHeat > 0))
                {
                    if (theObj.MaxHeat < objectScope.MinHeat)
                        continue;
                    if (theObj.MaxHeat > objectScope.MaxHeat)
                        continue;
                }

                answer.AddObject(theObject.Value);
            }

            return answer;
        }


        public ProcessObjList FilterByLeg(int legId)
        {
            ProcessObjList answer = new();

            foreach (var theObject in this)
                if ((theObject.Value.FlightLegId == legId) && theObject.Value.Significant)
                    answer.AddObject(theObject.Value);

            return answer;
        }


        // Calculate settings based on all provided objects 
        public void CalculateSettings(ProcessObjList objects)
        {
            ResetSettings();

            foreach (var theObject in objects)
            {
                var thisLocnErrM = theObject.Value.LocationErrM;
                (MinLocationErrM, MaxLocationErrM) = TardisSummaryModel.SummariseFloat(MinLocationErrM, MaxLocationErrM, thisLocnErrM);
                if (thisLocnErrM != BaseConstants.UnknownValue)
                    SumLocationErrM += thisLocnErrM;

                var thisHtM = theObject.Value.HeightM;
                var thisHtErrM = theObject.Value.HeightErrM;
                (MinHeightM, MaxHeightM) = TardisSummaryModel.SummariseFloat(MinHeightM, MaxHeightM, thisHtM);
                (MinHeightErrM, MaxHeightErrM) = TardisSummaryModel.SummariseFloat(MinHeightErrM, MaxHeightErrM, thisHtErrM);
                if (thisHtM != BaseConstants.UnknownValue)
                    SumHeightM += thisHtM;
                if (thisHtErrM != BaseConstants.UnknownValue)
                    SumHeightErrM += thisHtErrM;

                (MinSizeCM2, MaxSizeCM2) = TardisSummaryModel.SummariseFloat(MinSizeCM2, MaxSizeCM2, theObject.Value.SizeCM2);

                (MinRangeM, MaxRangeM) = TardisSummaryModel.SummariseInt(MinRangeM, MaxRangeM, theObject.Value.AvgRangeM);

                (MinHeat, MaxHeat) = TardisSummaryModel.SummariseInt(MinHeat, MaxHeat, theObject.Value.MaxHeat);
            }
        }


        // Calculate settings based on significant child objects 
        public void CalculateSettings(ProcessScope scope, int focusObjectID)
        {
            CalculateSettings(FilterByProcessScope(scope, focusObjectID));
        }


        // Return a histogram of the heights of the objects in this list
        public List<int> HistogramHeightM()
        {
            var answer = new List<int>();

            foreach (var theObject in this)
            {
                int metric = (int)Math.Round(theObject.Value.HeightM);
                if (metric >= 0)
                {
                    var index = metric;
                    while (answer.Count <= index)
                        answer.Add(0);
                    answer[index]++;
                }
            }

            return answer;
        }


        // Return a histogram of the size of the objects in this list
        public List<int> HistogramSizeCm2(int scale)
        {
            var answer = new List<int>();

            foreach (var theObject in this)
            {
                var metric = (int)(theObject.Value.SizeCM2 / scale) + 1;
                if (metric >= 0)
                {
                    var index = metric;
                    while (answer.Count <= index)
                        answer.Add(0);
                    answer[index]++;
                }
            }

            return answer;
        }


        // Return a histogram of the heat of the objects in this list
        public List<int> HistogramHeat()
        {
            var answer = new List<int>();

            foreach (var theObject in this)
            {
                var metric = theObject.Value.MaxHeat - this.MinHeat;
                if (metric >= 0)
                {
                    var index = metric;
                    while (answer.Count <= index)
                        answer.Add(0);
                    answer[index]++;
                }
            }

            return answer;
        }


        // Return a histogram of the range of the objects in this list
        public List<int> HistogramRangeM()
        {
            var answer = new List<int>();

            foreach (var theObject in this)
            {
                var metric = theObject.Value.AvgRangeM;
                if (metric >= 0)
                {
                    var index = metric;
                    while (answer.Count <= index)
                        answer.Add(0);
                    answer[index]++;
                }
            }

            return answer;
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public virtual DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "# Objects", Count },
                { "Min Locn Err M", MinLocationErrM, BaseConstants.HeightNdp },
                { "Max Locn Err M", MaxLocationErrM, BaseConstants.HeightNdp },
                { "Sum Locn Err M", SumLocationErrM, BaseConstants.HeightNdp },
                { "Min Hght M", MinHeightM, BaseConstants.HeightNdp },
                { "Max Hght M", MaxHeightM, BaseConstants.HeightNdp },
                { "Sum Hght M", SumHeightM, BaseConstants.HeightNdp },
                { "Min Hght Err M", MinHeightErrM, BaseConstants.HeightNdp },
                { "Max Hght Err M", MaxHeightErrM, BaseConstants.HeightNdp },
                { "Sum Hght Err M", SumHeightErrM, BaseConstants.HeightNdp },
                { "Min Size CM2", MinSizeCM2, BaseConstants.AreaCM2Ndp },
                { "Max Size CM2", MaxSizeCM2, BaseConstants.AreaCM2Ndp },
                { "Min Range M", MinRangeM },
                { "Max Range M", MaxRangeM },
                { "Min Heat", MinHeat },
                { "Max Heat", MaxHeat },
            };
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public void LoadSettings(List<string> settings)
        {
            int index = 0;
            index++; // # Objects = settings[0]
            MinLocationErrM = ConfigBase.StringToFloat(settings[index++]);
            MaxLocationErrM = ConfigBase.StringToFloat(settings[index++]);
            SumLocationErrM = ConfigBase.StringToFloat(settings[index++]);
            MinHeightM = ConfigBase.StringToFloat(settings[index++]);
            MaxHeightM = ConfigBase.StringToFloat(settings[index++]);
            SumHeightM = ConfigBase.StringToFloat(settings[index++]);
            MinHeightErrM = ConfigBase.StringToFloat(settings[index++]);
            MaxHeightErrM = ConfigBase.StringToFloat(settings[index++]);
            SumHeightErrM = ConfigBase.StringToFloat(settings[index++]);
            MinSizeCM2 = ConfigBase.StringToFloat(settings[index++]);
            MaxSizeCM2 = ConfigBase.StringToFloat(settings[index++]);
            MinRangeM = ConfigBase.StringToInt(settings[index++]);
            MaxRangeM = ConfigBase.StringToInt(settings[index++]);
            MinHeat = ConfigBase.StringToInt(settings[index++]);
            MaxHeat = ConfigBase.StringToInt(settings[index++]);
        }
    }


}
