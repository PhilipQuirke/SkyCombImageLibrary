using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessLogic;


namespace SkyCombImage.ProcessModel
{
    public class AnimalModel
    {
        public int FlightNum { get; set; }
        public string Name { get; set; }
        public GlobalLocation GlobalLocation { get; set; }
        public float LocationErrM { get; set; }
        public int SizeCM2 { get; set; }
        public string SizeClass { get; set; }
        public float HeightM { get; set; }
        public float HeightErrM { get; set; }
        public string HeightClass { get; set; }
        public float SpineM { get; set; }
        public float GirthM { get; set; }
        public float AvgRangeM { get; set; }


        public AnimalModel(int flightNum, Drone drone, ProcessObject theObj)
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

            SpineM = theObj.MaxSpinePixels / 10; // PQR TODO Convert to M
            GirthM = theObj.MaxGirthPixels / 10; // PQR TODO Convert to M

            AvgRangeM = theObj.AvgRangeM; // Small if CameraDownAngle is 80. Larger if CameraDownAngle is 30 degrees
        }


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
                { "Spine M", SpineM, 1 },
                { "Girth M", GirthM, 1 },
                { "Avg Range M", AvgRangeM, 0 },
            };
        }
    }


    public class AnimalModelList : List<AnimalModel>
    {
        public void AddProcessObjects(int flightNum, Drone drone, ProcessObjList objects, bool significantObjectsOnly = true)
        {
            if (objects != null)
                foreach (var obj in objects)
                    if (obj.Value.Significant || !significantObjectsOnly)
                        Add(new AnimalModel(flightNum, drone, obj.Value));
        }
    }
}
