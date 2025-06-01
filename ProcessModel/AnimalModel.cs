using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessLogic;
using System.Diagnostics;


namespace SkyCombImage.ProcessModel
{
    public class AnimalModel
    {
        public int FlightNum { get; }
        public string Name { get; }
        public GlobalLocation GlobalLocation { get; }
        public float LocationErrM { get; }
        public int SizeCM2 { get; }
        public string SizeClass { get; }
        public float HeightM { get; }
        public float HeightErrM { get; }
        public string HeightClass { get; }
        public float SpineM { get; }
        public float GirthM { get; }

        // Debugging information
        public int CameraDownDegs { get; }
        public float AvgRangeM { get; }
        public float MaxHeat { get; }


        public AnimalModel(int flightNum, Drone drone, ProcessSpanList? processSpans, ProcessObject theObj)
        {
            try
            {
                FlightNum = flightNum;
                Name = theObj.Name;

                // Convert the relative location theObj.LocationM to a global location in longitude and latitude
                GlobalLocation = drone.FlightSections.DroneToGlobalLocation(theObj.LocationM);
                LocationErrM = theObj.LocationErrM;

                SizeCM2 = (int)theObj.SizeCM2;
                (SizeClass, _) = MasterSizeModelList.CM2ToClass(theObj);

                (HeightClass, _) = MasterHeightModelList.HeightMToClass(theObj);
                HeightM = theObj.HeightM;
                HeightErrM = theObj.HeightErrM;

                SpineM = 1.0f * theObj.SpineCM / 100.0f;
                GirthM = 1.0f * theObj.GirthCM / 100.0f;

                CameraDownDegs = 0;
                var flightStep = theObj?.LastRealFeature?.Block?.FlightStep;
                if (flightStep != null)
                    CameraDownDegs = (int)Math.Round(90 - flightStep.CameraToVerticalForwardDeg);

                AvgRangeM = theObj.AvgRangeM; // Small if CameraDownAngle is 80. Larger if CameraDownAngle is 30 degrees

                MaxHeat = theObj.MaxHeat; // This is the maximum heat value for this object, used for debugging

                // Convert -999 to -2
                if (LocationErrM == BaseConstants.UnknownValue) LocationErrM = -2;
                if (HeightM == BaseConstants.UnknownValue) HeightM = -2;
                if (HeightErrM == BaseConstants.UnknownValue) HeightErrM = -2;
            }
            catch (Exception ex)
            {
                Debug.Print("AnimalModel: " + ex.Message);
                throw new Exception("AnimalModel: " + ex.Message);
            }
        }


        // Zero-based column indexes for animal data
        public const int ColFlightNum = 0;
        public const int ColName = 1;
        public const int ColLocation = 2;
        public const int ColLocationErrM = 3;
        public const int ColSizeClass = 4;
        public const int ColSizeCM2 = 5;
        public const int ColHeightClass = 6;
        public const int ColHeightM = 7;
        public const int ColHeightErrM = 8;
        public const int ColSpineM = 9;
        public const int ColGirthM = 10;
        public const int ColCameraDownDegs = 11;
        public const int ColAvgRangeM = 12;
        public const int ColMaxHeat = 13;


        public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Flight Num", FlightNum, 0 },
                { "Name", Name },
                { "Location", GlobalLocation.ToString() },
                { "Location Err M", LocationErrM, 1 },
                { "SizeClass", SizeClass },
                { "Size cm2", SizeCM2},
                { "Height Class", HeightClass },
                { "Height M", HeightM, 1 },
                { "Height Err M", HeightErrM, 1 },
                { "Spine M", SpineM, 2 },
                { "Girth M", GirthM, 2 },
                { "Camera Down Degs", CameraDownDegs },
                { "Avg Range M", AvgRangeM, 0 },
                { "Max Heat", MaxHeat, 1 },
            };
        }
    }


    public class AnimalModelList : List<AnimalModel>
    {
        public void AddProcessObjects(int flightNum, Drone drone, ProcessObjList objects, ProcessSpanList? processSpans = null, bool significantObjectsOnly = true)
        {
            if (objects != null)
                foreach (var obj in objects)
                    if (obj.Value.Significant || !significantObjectsOnly)
                        Add(new AnimalModel(flightNum, drone, processSpans, obj.Value));
        }
    }
}
