// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A significant object - a logical object derived from overlapping features over successive frames. 
    public class ProcessObject : ProcessObjectModel
    {
        // Parent process model
        protected ProcessAll ProcessAll { get; }

        public ProcessConfigModel? ProcessConfig { get { return ProcessAll == null ? null : ProcessAll.ProcessConfig; } }

        // Static NextObjectID shared by all objects
        public static int NextObjectId = 0;
        // Static random number generator
        protected static readonly RNG Rng = new();


        public ProcessFeatureList ProcessFeatures;
        // First (Real) feature claimed by this object. 
        public ProcessFeature? FirstFeature { get { return ProcessFeatures.FirstFeature; } }
        public ProcessFeature? LastRealFeature { get { return (LastRealFeatureId == UnknownValue ? null : ProcessFeatures[LastRealFeatureId]); } }
        // Last (Real or UnReal) feature claimed by this object. May be null.
        public ProcessFeature? LastFeature { get { return ProcessFeatures.LastFeature; } }


        public ProcessObject(ProcessAll processAll, ProcessScope scope) : base()
        {
            ProcessAll = processAll;
            ProcessFeatures = new(ProcessAll.ProcessConfig);

            ObjectId = ++NextObjectId;
            if (scope != null)
            {
                FlightLegId = scope.PSM.CurrRunLegId;
                RunFromVideoS = (float)(scope.PSM.CurrInputFrameMs / 1000.0);
                RunToVideoS = RunFromVideoS;
            }
        }


        public virtual bool ClaimFeature(ProcessFeature theFeature) { return false; }


        public override void ResetCalcedMemberData()
        {
            base.ResetCalcedMemberData();
            ProcessFeatures = new(ProcessConfig);
        }


        // Number of real features owned by this object.
        public int NumRealFeatures()
        {
            int answer = 0;

            if (LastRealFeatureId > 0)
                // Rarely, the object may have a sequence of real, then unreal, then real features.
                foreach (var feature in ProcessFeatures)
                    if (feature.Value.Type == FeatureTypeEnum.Real)
                        answer++;

            return answer;
        }


        // Is this object still worth tracking in specified block?
        public bool KeepTracking(int BlockId)
        {
            // Once inactive, an object stops being tracked permanently.
            if (BeingTracked)
            {
                if (LastRealFeature.Block.BlockId >= BlockId)
                {
                    // Yes, this is worth tracking.
                }
                else if (ProcessConfig.ObjectMaxUnrealBlocks > 0)
                {
                    // Are we still on the persistance window?
                    BeingTracked = (LastFeature.Block.BlockId - LastRealFeature.Block.BlockId < ProcessConfig.ObjectMaxUnrealBlocks);
                }
                else
                    BeingTracked = false;
            }

            return BeingTracked;
        }


        // How long has this object been seen for in Config.ObjectMinDurationMs units?
        public double SeenForMinDurations()
        {
            var minDuration = ProcessConfig.ObjectMinDurationMs; // Say 500ms
            var timeSeenMs = (1000.0F * NumRealFeatures()) / ProcessAll.VideoData.Fps;
            return (timeSeenMs / minDuration);
        }


        // More features is correlated with more location error.
        // A good measure of location scatter is location error per real feature.
        public float LocationErrPerFeatureM()
        {
            int realFeats = NumRealFeatures();

            return (realFeats == 0 ? UnknownValue : LocationErrM / realFeats);
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


        // Get the object's DEM at the OBJECT'S location.
        protected void Calculate_DemM()
        {
            if ((LocationM == null) || (ProcessAll.GroundData == null) || (ProcessAll.GroundData.DemModel == null))
                return;

            // Most accurate method. Nearly always works.
            var newDemM = ProcessAll.GroundData.DemModel.GetElevationByDroneLocn(LocationM);
            if (newDemM != UnknownValue)
            {
                DemM = newDemM;
                return;
            }

            // In rare cases, we have an object just outside ground datum grid.
            // Object may be say 10m to left and 40m ahead of the drone's location.
            // Forced to use less progressively less accurate methods.
            var firstFeat = ProcessFeatures.FirstFeature;
            var firstStep = firstFeat.Block.FlightStep;
            if (firstStep == null)
                return;

            if (firstStep.InputImageDemM != UnknownValue)
                DemM = firstStep.InputImageDemM;
            else
                DemM = firstStep.DemM;
        }


        // Given this object's last known position, and the object's
        // average velocity, where do we expect the object to be this block?
        public Rectangle ExpectedLocationThisBlock()
        {
            Rectangle answer;

            var firstFeat = ProcessFeatures.FirstFeature;
            var lastFeat = ProcessFeatures.LastFeature;

            var firstBox = firstFeat.PixelBox;
            var lastBox = lastFeat.PixelBox;

            int lastWidth = lastBox.Width;
            int lastHeight = lastBox.Height;

            var numBlockSteps = lastFeat.BlockId - firstFeat.BlockId + 1;
            if (numBlockSteps >= 2)
            {
                // In DJI_0118 leg 3, Object 1 starts large but fades
                // so that LastFeature().PixelBox is very small.
                // The expected location should use the maximum object size.
                int addWidth = Math.Max(0, MaxRealPixelWidth - lastWidth);
                int addHeight = Math.Max(0, MaxRealPixelHeight - lastHeight);

                // We have multiple features. Use their difference in location.
                var distanceX = 1.0F * lastBox.X + lastBox.Width / 2.0F - firstBox.X - lastBox.Width / 2.0F;
                var distanceY = 1.0F * lastBox.Y + lastBox.Height / 2.0F - firstBox.Y - lastBox.Height / 2.0F;
                var numMoves = numBlockSteps - 1;

                // Advance one average stride from the previous location.
                answer = new Rectangle(
                    (int)(
                    lastBox.X + distanceX / numMoves - addWidth / 2.0f),
                    (int)(lastBox.Y + distanceY / numMoves - addHeight / 2.0f),
                    lastWidth + addWidth,
                    lastHeight + addHeight);
            }
            else
                // With one feature we dont know the object's velocity across the image.
                // Rely on image overlap
                answer = new Rectangle(
                    lastBox.X,
                    lastBox.Y,
                    lastWidth,
                    lastHeight);

            if (lastFeat.Type == FeatureTypeEnum.Real)
                // We don't want a drone wobble to break the object feature sequence
                // So we inflate the expected location by 5 pixels in each direction.
                answer.Inflate(5, 5);

            return answer;
        }


        // After loading data from a previous run from the data store
        // we have blocks, features and objects, but the links between them are not set.
        public void SetLinksAfterLoad(ProcessFeature feature)
        {
            Assert(feature != null, "SetLinksAfterLoad: Feature not provided.");
            Assert(feature.ObjectId == ObjectId, "SetLinksAfterLoad: Feature not for this object.");

            ProcessFeatures.AddFeature(feature);
            if (feature.Type == FeatureTypeEnum.Real)
                LastRealFeatureId = feature.FeatureId;
        }


        public virtual void Calculate_Significant()
        {
            Significant = ProcessFeatures.Count > 0;
        }


        // Calculate the drone SumLinealM distance corrsponding to the centroid of the object
        protected void Calculate_AvgSumLinealM()
        {
            var firstFeat = FirstFeature;
            var lastFeat = LastRealFeature;

            if ((firstFeat != null) &&
               (firstFeat.Block != null) &&
               (firstFeat.Block.FlightStep != null) &&
               (lastFeat != null) &&
               (lastFeat.Block != null) &&
               (lastFeat.Block.FlightStep != null))
            {
                var firstSumLinealM = firstFeat.Block.FlightStep.SumLinealM;
                var lastSumLinealM = lastFeat.Block.FlightStep.SumLinealM;
                if (firstSumLinealM == UnknownValue)
                    AvgSumLinealM = lastSumLinealM;
                else if (lastSumLinealM != UnknownValue)
                    AvgSumLinealM = (firstSumLinealM + lastSumLinealM) / 2;
            }
        }


        // Calculate object's location (centroid) as average of real feature's locations.
        // The feature locations are based on where in the drone's field of image the object was detected. 
        // Calculate location error, a measure of real feature dispersion,
        // as average distance from object location (centroid).
        // Refer https://stats.stackexchange.com/questions/13272/2d-analog-of-standard-deviation for rationale.
        protected void Calculate_LocationM_and_LocationErrM()
        {
            (LocationM, LocationErrM) = ProcessFeatures.Calculate_Avg_LocationM_and_LocationErrM();
        }


        protected bool HasMoved()
        {
            // Drone moved from point A to point B (base-line distance L) in metres.
            double baselineM = RelativeLocation.DistanceM(
                FirstFeature.Block.DroneLocnM,
                LastRealFeature.Block.DroneLocnM);
            // If drone has not moved enough this method will be very inaccurate.
            return (baselineM >= 2);
        }


        // Calculate object height and object height error by averaging the feature data.
        protected void Calculate_HeightM_and_HeightErrM()
        {
            (HeightM, HeightErrM, MinHeightM, MaxHeightM) = ProcessFeatures.Calculate_Avg_HeightM_and_HeightErrM();
        }


        // Calculate the average horizontal range of the object from the drone in meters.
        // Relies on object.LocationM already being calculated.
        protected void Calculate_AvgRangeM()
        {
            var firstFeature = FirstFeature;
            var lastRealFeature = LastRealFeature;
            if ((firstFeature == null) ||
                (firstFeature.Block == null) ||
                (firstFeature.Block.FlightStep == null) ||
                (lastRealFeature == null) ||
                (lastRealFeature.Block == null) ||
                (lastRealFeature.Block.FlightStep == null))
                return;

            AvgRangeM = (int)(
                (RelativeLocation.DistanceM(LocationM, firstFeature.Block.FlightStep.DroneLocnM) +
                 RelativeLocation.DistanceM(LocationM, lastRealFeature.Block.FlightStep.DroneLocnM)) / 2.0);
        }


        // Returns the average sideways-down-angles (right angles to the direction of flight)
        // of the object in the first image detected and the last image detected.
        // Only uses camera physics and hot-object position in image data.
        protected double Calculate_Image_AvgSidewaysRads()
        {
            (var xFracFirst, var _) = FirstFeature.CentroidImageFractions();
            (var xFracLast, var _) = LastFeature.CentroidImageFractions();

            // Calculation is based on physical parameters of the thermal camera.
            double fullHorizFoVRadians = ProcessAll.VideoData.HFOVRad;
            double halfHorizFoVRadians = fullHorizFoVRadians / 2;

            // Calculate the average angle to object, at right angles to the direction of flight (sideways), to the vertical, in radians
            //      If PixelBox.X = 0 then object is at the very left of the image => SideRads is -horizFoVRadians/2
            //      If PixelBox.X = ImageWidth/2 then object is in the middle of the image => SideRads is 0
            //      If PixelBox.X = ImageWidth then object is at the very right of the image => SideRads is +horizFoVRadians/2
            // Assumes drone is moving forward (not sidewards or backwards) or is stationary.
            double firstSideFraction = xFracFirst * 2 - 1;
            Assert(firstSideFraction >= -1 && firstSideFraction <= 1, "Calculate_Image_AvgSidewaysRads: firstSideFraction out of range");

            double firstSideRads = halfHorizFoVRadians * firstSideFraction;
            Assert(Math.Abs(firstSideRads) <= halfHorizFoVRadians, "Calculate_Image_AvgSidewaysRads: firstSideRads out of range");

            double lastSideFraction = xFracLast * 2 - 1;
            Assert(lastSideFraction >= -1 && firstSideFraction <= 1, "Calculate_Image_AvgSidewaysRads: lastSideFraction out of range");

            double lastSideRads = halfHorizFoVRadians * lastSideFraction;
            Assert(Math.Abs(lastSideRads) <= halfHorizFoVRadians + 0.005, "Calculate_Image_AvgSidewaysRads: lastSideRads out of range");

            Assert(Math.Abs(lastSideRads - firstSideRads) < fullHorizFoVRadians, "Calculate_Image_AvgSidewaysRads: Bad From/to side range");

            return (firstSideRads + lastSideRads) / 2;
        }


        // Calculate the size of the object in square centimeters.
        // Based on MAXIMUM number of hot pixels in any real feature.
        protected void Calculate_SizeCM2()
        {
            if ((ProcessAll == null) || (ProcessAll.VideoData == null))
                return;

            var lastFeature = LastFeature;
            if ((lastFeature == null) ||
                (lastFeature.Type != FeatureTypeEnum.Real) ||
                (lastFeature.Block == null) ||
                (lastFeature.Block.FlightStep == null) ||
                (lastFeature.Block.FlightStep.InputImageSizeM == null))
                return;

            // The number of hot pixels in the last (real) feature.
            float hotPixels = lastFeature.NumHotPixels;

            // Grab the drone input image area
            float imageAreaM2 = lastFeature.Block.FlightStep.InputImageSizeM.AreaM2();

            // Calculate the number of pixels in the video image
            float framePixels = ProcessAll.VideoData.ImageWidth * ProcessAll.VideoData.ImageHeight;

            // Calculate the size of the object in this frame in square centimeters
            float thisSizeM2 = imageAreaM2 * hotPixels / framePixels;

            var thisSizeCM2 = (int)(thisSizeM2 * 100 * 100);

            SizeCM2 = Math.Max(SizeCM2, thisSizeCM2);
        }


        // Calculate the maximum heat value of any pixel in this object in any frame 
        protected void Calculate_MaxHeat()
        {
            (int _, int maxHeat, int _) = ProcessFeatures.HeatSummary();
            MaxHeat = maxHeat;
        }

        // Maximum density of the object across real features.
        // Assumes that the maximum hot pixels occurred when the object was at its maximum size.
        // Measured in hot pixels in rectangular area so <= 1. Example value 0.5
        public double RealDensityPx()
        {
            if ((MaxRealPixelWidth <= 0) || (MaxRealPixelHeight <= 0))
                return 0;

            var pixelArea = 1.0f * MaxRealPixelWidth * MaxRealPixelHeight;

            // Density is measured as # hot pixels within an area.
            // pixelArea, as calculated above, is rectangular.
            // Assuming hotspot is a circular object contained in rectangle,
            // the circular object covers 78.5 % of the rectangle, so decrease pixelArea. 
            pixelArea *= 0.785f;

            return (1.0f * MaxRealHotPixels) / pixelArea;
        }


        // Calculate the simple (int, float, VelocityF, etc) member-data of this real object.
        // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
        public void Calculate_RealObject_SimpleMemberData()
        {
            try
            {
                // Calculate the drone SumLinealM distance corresponding to the centroid of the object
                Calculate_AvgSumLinealM();

                FirstFwdDownDeg = (float)FirstFeature.Calculate_Image_FwdDeg();
                LastFwdDownDeg = (float)LastRealFeature.Calculate_Image_FwdDeg();

                // First estimate of OBJECT location (centroid) as average over all real features.
                // The feature locations are based on where in the drone's field of image the object was detected,
                // and takes into account ground undulations. The object's location is further refined later.
                Calculate_LocationM_and_LocationErrM();

                // Estimate the OBJECT's DEM at the object's location
                // (which could be say 20m left of the drone's flight path).
                Calculate_DemM();

                // In RunVideoCombDrone.AddBlockAndProcessInputVideoFrame, a new Feature is created
                // and these "one-feature" function calls are made (before the feature is assigned to an object):
                //    FEATURE.CalculateSettings_LocationM_FlatGround();           // Assumes flat ground & feature is on ground
                //    FEATURE.CalculateSettings_LocationM_HeightM_LineofSight();  // Works if CameraFwdDeg between 10 and 80.
                // Later a CombObject consumes/claims the new feature.
                // The new feature may be the 10th feature of the object we call:
                //    FEATURE.Calculate_HeightM_BaseLineMovement();           // Works if drone has moved horizontally
                // Which estimates object height above ground based on distance down from drone
                // calculated using trigonometry and first/last real feature camera-view-angles.
                // Only works if the drone has moved horizontally some distance. Works at 1m. Better at 5m
                // May override CalculateSettings_LocationM_HeightM_LineofSight
                var lastFeature = LastFeature;
                if ((SeenForMinDurations() >= 1) && HasMoved())
                {
                    // Estimate last FEATURE height above ground based on distance down from drone
                    // calculated using trigonometry and first/last real feature camera-view-angles.
                    // This is a "look down" trig method. Accuracy limited by the accuracy of the drone altitude.
                    // Object at the left/right edge of the image are slightly further from the drone
                    // than objects directly under the drone.
                    // If drone is not moving now, calculated HeightM will be the same as last feature (within Gimbal wobble). 
                    if (lastFeature.Type == FeatureTypeEnum.Real) // PQR    && is moving now.
                        lastFeature.Calculate_HeightM_BaseLineMovement(
                                FirstFeature,
                                DemM,
                                ProcessFeatures.AverageFlightStepFixedAltitudeM());
                }
                else
                    lastFeature.SetHeightAlgorithmError("BL_TooShort"); // Either in time or distance.

                // Calculate OBJECT height and object height error (as average over real features).
                Calculate_HeightM_and_HeightErrM();

                // Calculate OBJECT size in square centimeters (based on maximum # hot pixels over real features).
                Calculate_SizeCM2();

                // Calculate the range of the OBJECT from the drone in meters (as average over real features).
                Calculate_AvgRangeM();

                // Calculate the OBJECT maximum heat value (of any pixel over real features).
                Calculate_MaxHeat();

                // Is this OBJECT significant?
                Calculate_Significant();
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessObject.Calculate_RealObject_SimpleMemberData", ex);
            }
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        public override DataPairList GetSettings()
        {
            var answer = base.GetSettings();

            var firstBlock = (FirstFeature != null ? FirstFeature.Block.BlockId : 0);
            var lastRealBlock = (LastRealFeature != null ? LastRealFeature.Block.BlockId : 0);
            var lastBlock = (LastFeature != null ? LastFeature.Block.BlockId : 0);
            var centerBlock = (firstBlock + lastRealBlock + 1) / 2; // The +1 rounds upward.

            answer.Add("FirstBlock", firstBlock);
            answer.Add("CenterBlock", centerBlock);
            answer.Add("LastRealBlock", lastRealBlock);
            answer.Add("LastBlock", lastBlock);

            answer.Add("#RealFeats", NumRealFeatures());
            answer.Add("RealDensityPx", RealDensityPx(), AreaM2Ndp);
            answer.Add("LocnErrPerFeatCM", LocationErrPerFeatureM() * 100, LocationNdp);

            // Average horizontal distance from object to drone in M.
            answer.Add("RangeM", AvgRangeM);

            return answer;
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


        // Ensure each object has at least an "insignificant" name e.g. #16
        public void EnsureObjectsNamed()
        {
            foreach (var theObj in this)
                if (theObj.Value.Name == "")
                    theObj.Value.SetName();
        }



        public int NumSignificantObjects()
        {
            int answer = 0;

            foreach (var theObj in this)
                if (theObj.Value.Significant)
                    answer++;

            return answer;
        }


        // Number of objects that have ever been significant. 
        // Not same as num objs significant in the current Block, as objects can become insignificant. 
        public int NumEverSignificantObjects
        {
            get
            {
                int numSig = 0;
                foreach (var theObject in this)
                    if (theObject.Value.NumSigBlocks > 0)
                        numSig++;
                return numSig;
            }
        }


        public virtual void StopTracking()
        {
            foreach (var theObject in this)
                theObject.Value.BeingTracked = false;
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


        public ProcessObjList FilterByProcessScope(ProcessScope scope)
        {
            ProcessObjList answer = new();

            foreach (var theObject in this)
                if (theObject.Value.Significant &&
                    // Only return objects in the RunFrom/To scope.
                    theObject.Value.InRunScope(scope))
                    answer.AddObject(theObject.Value);

            if (answer.Count > 0)
                answer.CalculateSettings();

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


        // Returns key attributes of objects and associated user annotations (if any)
        // to show in the ObjectGrid in the Main Form or the ObjectList in the Object Form
        public List<object[]> GetObjectGridData(ProcessScope scope, ProcessConfigModel processConfig, bool mainForm, ObjectCategoryList annotations)
        {
            var answer = new List<object[]>();

            var sigObjects = FilterByProcessScope(scope);
            foreach (var theObject in sigObjects)
            {
                ObjectCategoryModel? annotation = null;
                if (annotations != null)
                    annotation = annotations.GetData(theObject.Value.Name);

                answer.Add(theObject.Value.GetObjectGridData(processConfig, mainForm, annotation));
            }

            return answer;
        }


        // After loading data from a previous run from the data store
        // we have blocks, features and objects, but the links between them are not set.
        // Apply the feature to the corresponding object.
        public void SetLinksAfterLoad(ProcessFeature feature)
        {
            if ((feature == null) || (feature.ObjectId <= 0))
                return;

            this.TryGetValue(feature.ObjectId, out var processObject);

            // Null case occurs when saving All features but only Significant objects and load from datastore
            if (processObject != null)
                processObject.SetLinksAfterLoad(feature);
        }


        // Calculate settings based on provided ProcessObjects 
        private void CalculateSettings(ProcessObjList objects)
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
        // Calculate settings based on owned ProcessObjects 
        public void CalculateSettings()
        {
           CalculateSettings(this);
        }
        // Calculate settings based on in-scope ProcessObjects 
        public void CalculateSettings(ProcessScope scope)
        {
            CalculateSettings(FilterByProcessScope(scope));
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


        // Get the minimum StepId of all objects
        public int GetMinStepId()
        {
            if (Count == 0)
                return BaseConstants.UnknownValue;

            int answer = 9999999;
            foreach (var theObject in this)
            {
                var firstFeat = theObject.Value.FirstFeature;
                if (firstFeat != null)
                    answer = Math.Min(answer, firstFeat.Block.FlightStepId);
            }
            return answer;
        }


        public string DescribeSignificantObjects()
        {
            var num = NumEverSignificantObjects;
            return (num == 0 ? "" : string.Format("{0} Objects", num));
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
