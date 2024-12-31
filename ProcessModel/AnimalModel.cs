using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessLogic;


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
        public float AvgRangeM { get; }
        public int CameraDownDegs { get; }
        public int BestFixAltM { get; }
        public int BestFixYawDeg { get; }
        public int BestFixPitchDeg { get; }



        public AnimalModel(int flightNum, Drone drone, ProcessSpanList? processSpans, ProcessObject theObj)
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

            CameraDownDegs = 0;
            var flightStep = theObj?.LastRealFeature?.Block?.FlightStep;
            if (flightStep != null)
                CameraDownDegs = (int) Math.Round(90 - flightStep.CameraToVerticalForwardDeg);

            BestFixAltM = 0;
            BestFixYawDeg = 0;
            BestFixPitchDeg = 0;
            if (processSpans != null)
            {
                var block = theObj?.LastRealFeature?.Block;
                if (block != null && block.FlightLegId != BaseConstants.UnknownValue)
                {
                    var span = processSpans[block.FlightLegId];
                    if (span != null)
                    {
                        BestFixAltM = (int) span.BestFixAltM;
                        BestFixYawDeg = (int) span.BestFixYawDeg;
                        BestFixPitchDeg = (int) span.BestFixPitchDeg;
                    }
                }
            }

            // Convert -999 to -2
            if (LocationErrM == BaseConstants.UnknownValue) LocationErrM = -2;
            if (HeightM == BaseConstants.UnknownValue) HeightM = -2;
            if (HeightErrM == BaseConstants.UnknownValue) HeightErrM = -2;
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
                { "Camera Down Degs", CameraDownDegs },
                { "Fix Alt M", BestFixAltM },
                { "Fix Yaw Deg", BestFixYawDeg },
                { "Fix Pitch Deg", BestFixPitchDeg },
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
