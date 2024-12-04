using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessLogic;


namespace SkyCombImageLibrary.ProcessModel
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
                { "SpineM", SpineM, 1 },
                { "GirthM", GirthM, 1 },
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
