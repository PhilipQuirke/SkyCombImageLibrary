// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV.Structure;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A process feature combines the stored data and some logic
    public class ProcessFeature : ProcessFeatureModel
    {
        // Parent process model
        protected ProcessAll ProcessAll { get; }
        public ProcessConfigModel? ProcessConfig { get { return ProcessAll == null ? null : ProcessAll.ProcessConfig; } }


        // A feature is associated 1-1 with a Block
        public ProcessBlock Block { get; set; }

        // Location of hot pixels in this feature.
        public PixelHeatList? Pixels { get; set; } = null;


        public ProcessFeature(ProcessAll processAll, int blockId, FeatureTypeEnum type) : base(blockId, type)
        {
            ResetCalcedMemberData();

            ProcessAll = processAll;
            Block = processAll.Blocks[blockId];
            if (type != FeatureTypeEnum.Unreal)
                Pixels = new();
        }


        // Constructor used when loaded objects from the datastore
        public ProcessFeature(ProcessAll processAll, List<string> settings) : base(settings)
        {
            ProcessAll = processAll;
            Block = ProcessAll.Blocks[BlockId];
        }


        // Is this feature larger than the largest allowed?
        public bool FeatureOverSized
        {
            get
            {
                return
                    (PixelBox.Width > ProcessConfigModel.FeatureMaxSize) ||
                    (PixelBox.Height > ProcessConfigModel.FeatureMaxSize);
            }
        }


        protected void CalcNumHotPixels()
        {
            NumHotPixels = (Pixels == null ? 0 : Pixels.Count());
        }


        public void ClearHotPixels()
        {
            Pixels = null;
            CalcNumHotPixels();
        }


        protected void AddHotPixel(int currY, int currX, Bgr currColor)
        {
            int currHeat = (int)((currColor.Blue + currColor.Green + currColor.Red) / 3);

            MinHeat = Math.Min(MinHeat, currHeat);
            MaxHeat = Math.Max(MaxHeat, currHeat);

            Pixels.Add(new PixelHeat(BlockId, FeatureId, currY, currX, currHeat));
            CalcNumHotPixels();
        }


        // Does this Feature's PixelBox and the specified object's rectangle overlap significantly?
        public bool SignificantPixelBoxIntersection(Rectangle objectExpectedLocation)
        {
            int featureMinOverlapPerc = ProcessConfigModel.FeatureMinOverlapPerc;
            var intersection = Rectangle.Intersect(PixelBox, objectExpectedLocation);

            // Refer https://stackoverflow.com/questions/9324339/how-much-do-two-rectangles-overlap
            // SI = Max(0, Min(XA2, XB2) - Max(XA1, XB1)) * Max(0, Min(YA2, YB2) - Max(YA1, YB1))
            // SU = SA + SB - SI
            // OverlapFraction = SI / SU
            var sizeIntersection = intersection.Width * intersection.Height;

            var sizeA = PixelBox.Width * PixelBox.Height;
            var sizeB = objectExpectedLocation.Width * objectExpectedLocation.Height;

            var resultA = 1.0F * sizeIntersection / sizeA;
            var resultB = 1.0F * sizeIntersection / sizeB;

            var minOverlap = featureMinOverlapPerc / 100.0F;

            return
                resultA >= minOverlap ||   // 25% overlap of rectA
                resultB >= minOverlap;     // 25% overlap of rectB
        }




        // This feature consumes/absorbs/takes-hot-pixels-from the otherFeature, leaving otherFeature empty.
        public virtual void Consume(ProcessFeature otherFeature)
        {
            // Expand PixelBox
            var thisRect = this.PixelBox;
            var otherRect = otherFeature.PixelBox;
            var answerLeft = Math.Min(thisRect.X, otherRect.X);
            var answerTop = Math.Min(thisRect.Y, otherRect.Y);
            var answerRight = Math.Max(thisRect.X + thisRect.Width, otherRect.X + otherRect.Width);
            var answerBottom = Math.Max(thisRect.Y + thisRect.Height, otherRect.Y + otherRect.Height);
            this.PixelBox = new Rectangle(answerLeft, answerTop, answerRight - answerLeft + 1, answerBottom - answerTop + 1);

            // Keep otherFeature pointing at ObjectID so we know which object decided to consume it.
            // otherFeature.ObjectID = 0;

            otherFeature.Significant = false;
            otherFeature.IsTracked = false;
            otherFeature.Type = FeatureTypeEnum.Consumed;
        }


        // Calculate center (centroid) of feature in image as fraction 0 to 1
        // If yImageFrac = 0 then object is at the very bottom of the image (furtherest from drone)
        // If yImageFrac = 1/2 then object is in the middle of the image 
        // If yImageFrac = 1 then object is at the top of the image (closest to drone)
        // Y = 1 is the top of the image, closest to the drone. 
        public (double xFraction, double yFraction) CentroidImageFractions()
        {
            double xCenterPixels = Math.Min(PixelBox.X + PixelBox.Width / 2.0, ProcessAll.VideoData.ImageWidth);
            double yCenterPixels = Math.Min(PixelBox.Y + PixelBox.Height / 2.0, ProcessAll.VideoData.ImageHeight);

            // Calculate position of center of feature as fraction of drone image area.
            double xFraction = xCenterPixels / ProcessAll.VideoData.ImageWidth;
            // With image pixels, y = 0 is the top of the image. 
            // Here we change the "sign" of Y, so that y = 0 is the bottom of the image.
            double yFraction = (ProcessAll.VideoData.ImageHeight - yCenterPixels) / ProcessAll.VideoData.ImageHeight;

            return (xFraction, yFraction);
        }


        // Returns the forward-down-angle (to vertical) of the object in the feature.
        // Only uses camera physics and object position in image data.
        public double Calculate_Image_FwdDeg()
        {
            var flightStep = Block.FlightStep;
            if (flightStep == null)
                return 0;

            // Unitless numbers
            // If yImageFrac = 0 then object is at the very bottom of the image (furtherest from drone) => FwdDegs is +16
            // If yImageFrac = 1/2 then object is in the middle of the image => FwdDegs is 0
            // If yImageFrac = 1 then object is at the top of the image (closest to drone) => FwdDegs is -16
            (var _, var yImageFrac) = CentroidImageFractions();

            // Calculation is based on physical parameters of the camera.
            double fullVertFoVDeg = ProcessAll.VideoData.VFOVDeg; // Say 32 degrees
            double halfVertFoVDeg = fullVertFoVDeg / 2; // Say 16 degrees

            // Calculate the angle to object, in direction of flight (forward), to the vertical, in degrees
            // Assumes drone is moving forward (not sidewards or backwards) or stationary.
            double fwdFraction = yImageFrac * 2 - 1;
            Assert(fwdFraction >= -1.01 && fwdFraction <= 1.01, "Calculate_Image_FwdDeg: fwdFraction out of range: " + fwdFraction.ToString());

            double fwdDeg = halfVertFoVDeg * fwdFraction; // Often close to +16
            Assert(Math.Abs(fwdDeg) <= halfVertFoVDeg + 0.1, "Calculate_Image_FwdDeg: fwdDeg out of range: " + fwdDeg.ToString());

            // Have seen real world case where CameraToVerticalForwardDeg was (for a short period) 90.6 degrees.
            // What is slightly above the horizontal.
            var cameraToVertDeg = flightStep.CameraToVerticalForwardDeg;

            fwdDeg += cameraToVertDeg;

            // Angles above horizontal may break our code so limit max to 90 (horizontal).
            // We are looking for close-by animals, not animals on the horizon,
            // so our algorithms tend to ignore data when fwdDeg > 80 (eighty) degrees. 
            fwdDeg = Math.Min(90, fwdDeg); // 90 degrees is horizontal

            return fwdDeg;
        }


        // Calculate the location (centroid) of this feature inside the drone imaging box
        // This is the key translation from IMAGE to PHYSICAL coordinate system.
        // If DSM or DEM data is available then considers ground level undulations between the drone and the feature.
        // Assumes ground is flat and object is on the ground.
        public void CalculateSettings_LocationM_FlatGround(ProcessFeature? lastRealFeature)
        {
            try
            {
                if ((LocationM != null) || (Block.FlightStep == null) || (Block.FlightStep.InputImageCenter == null))
                    return;
                var flightStep = Block.FlightStep;

                if ((Type == FeatureTypeEnum.Unreal) && (lastRealFeature != null))
                {
                    // For unreal features, just copy the last real feature's location
                    LocationM = lastRealFeature.LocationM?.Clone();
                    HeightM = lastRealFeature.HeightM;
                    HeightAlgorithm = UnrealCopyHeightAlgorithm;
                    return;
                }

                // Calculate center (centroid) of feature in image pixels.
                (double xFraction, double yFraction) = CentroidImageFractions();

                // There are multiple Blocks per FlightLocation.
                // This is the smoothed block-level change in location
                // since the previous FlightStep, in the flight direction.
                var droneBlockLocnM = Block.DroneLocnM;
                DroneLocation deltaBlockLocnM = new(
                    droneBlockLocnM.NorthingM - flightStep.DroneLocnM.NorthingM,
                    droneBlockLocnM.EastingM - flightStep.DroneLocnM.EastingM);

                // Calculate physical location of this feature based on:
                // 1) the POSITION in the image of the feature (given by xFraction, yFraction, say 0.4, 0.1)
                // 2) the CENTER of the drone physical field of vision (given by FlightStep.InputImageCenter, say 240m Northing, 78m Easting )
                // 3) the SIZE of the drone physical field of vision (given by InputImageSizeM, say 18m by 9m)
                // 4) the DIRECTION of flight of the drone (given by YawDeg, say -73 degrees)
                // This is the key translation from IMAGE to PHYSICAL coordinate system. 
                // Does NOT consider land contour undulations. Assumes land is flat.
                var flatLandLocationM =
                    flightStep.CalcImageFeatureLocationM(deltaBlockLocnM, xFraction, yFraction)
                        ?.Clone();

                // First approximation of location is based on flat land assumption.
                LocationM = flatLandLocationM;
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessFeature.CalculateSettings_LocationM_FlatGround", ex);
            }
        }


        // Calculate land contour undulations impact on object location and height.
        // The DSM level at flatLandLocationM may be higher or lower than the ground level below the drone.
        // Start half-way between the drone location and move stepwise to the flatLandLocationM and some distance beyond.
        // Compare the sight-line height at each step to the DSM level at that location.
        // Stop when the drone sight-line intersects the DSM level.
        // Algorithm works even if drone is stationary. CameraToVerticalForwardDeg must be between 10 and 80 degrees
        public void CalculateSettings_LocationM_HeightM_LineofSight(GroundData groundData)
        {
            int phase = 0;
            try
            {
                phase = 1;
                if ((ProcessAll == null) || (Block.FlightStep == null) || (Block.FlightStep.InputImageCenter == null))
                    return;
                var flightStep = Block.FlightStep;

                // Algorithm does not work if camera is pointing straight down.
                // Algorithm works inaccurately if the camera is pointing at the horizon.
                phase = 2;
                var fwdToVertDeg = flightStep.CameraToVerticalForwardDeg;
                if ((fwdToVertDeg < 10) || (fwdToVertDeg > 80))
                    return;

                phase = 3;
                if ((LocationM == null) || (groundData == null))
                    return;
                var flatLandLocationM = LocationM;
                var groundModel = groundData.HasDsmModel ? groundData.DsmModel : groundData.DemModel;
                if (groundModel == null)
                    return;

                // We use the drone camera's forward-down-angle (to vertical) to calculate the step-down distance (per 1 m horizontal).
                // (Not the object as the object may be at the edge of the image with a FwdDeg of ~0.
                phase = 4;
                var tan = Math.Tan(fwdToVertDeg * DegreesToRadians);
                if (tan == 0)
                    return;
                float vertStepDownPerHorizM = (float)(1.0 / tan);
                if (vertStepDownPerHorizM <= 0.1)
                    // This is a 10cm drop in altitude for each 1m step towards the 
                    // Camera is almost horizontal and this method wont work well.
                    return;

                // Calculate the distance from the drone to the flatLandLocationM
                phase = 5;
                var droneBlockLocnM = Block.DroneLocnM;
                DroneLocation deltaLocnM = flatLandLocationM.Subtract(droneBlockLocnM);
                var deltaM = deltaLocnM.DiagonalM;
                var horizUnitVector = deltaLocnM.UnitVector();

                // Calculate horizontal step distance
                var vertEpsilonM = 0.20f; // 20cm
                var horizStepM = vertEpsilonM / vertStepDownPerHorizM;

                // Step from the drone towards the flatLandLocationM and beyond
                DroneLocation? prevLocnM = null;
                float prevHeightM = 0;
                for (float testM = deltaM * 0.2f; testM < deltaM * 1.4f; testM += horizStepM)
                {
                    phase = 6;
                    var testLocnM = droneBlockLocnM.Add(horizUnitVector.Multiply(testM));
                    float testDsmM = groundModel.GetElevationByDroneLocn(testLocnM);
                    if (testDsmM == UnknownValue)
                    {
                        SetHeightAlgorithmError("LOS NoDsm");
                        continue;
                    }

                    phase = 7;
                    float testAltM = Block.AltitudeM - testM * vertStepDownPerHorizM;
                    float testHeightM = UnknownValue;
                    if (testAltM < testDsmM + vertEpsilonM)
                    {
                        // Drone line of sight has intersected the surface layer
                        phase = 8;
                        LocationM = testLocnM;
                        if (groundData.HasDemModel)
                        {
                            phase = 9;
                            var testDemM = groundData.DemModel.GetElevationByDroneLocn(testLocnM);
                            if (testDemM != UnknownValue)
                            {
                                phase = 10;
                                testHeightM = testAltM - testDemM;

                                // Taking small steps, we should not go very negative between steps.
                                // But have seen real world cases with testHeightM == -7,
                                // perhaps because of a step change in testDemM
                                if ((testHeightM < 0) &&
                                    (prevLocnM != null) &&
                                    (prevHeightM > 0) &&
                                    (Math.Abs(prevHeightM) < Math.Abs(testHeightM)))
                                {
                                    LocationM = prevLocnM;
                                    HeightM = prevHeightM;
                                }
                                else
                                    HeightM = Math.Max(0, testHeightM);
                                HeightAlgorithm = LineOfSightHeightAlgorithm;
                            }
                            else
                                SetHeightAlgorithmError("LOS NoDem");
                        }
                        else
                            SetHeightAlgorithmError("LOS NoDem");

                        break;
                    }

                    prevLocnM = testLocnM;
                    prevHeightM = testHeightM;
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessFeature.CalculateSettings_LocationM_HeightM_LineofSight " + phase.ToString(), ex);
            }
        }


        // Estimate last FEATURE height above ground (which is the best estimate of OBJECT height above ground).
        // Requires drone to have moved horizontally a distance.
        // Requires hot spot to have moved in the image.
        // Uses a "look down" trigonometry method based on difference in camera-view-angles from first to last feature.
        // Hotspot may be 10m to left and 40m in front of the drone location.
        // More accurate if CameraDownAngle is close to 90 degrees.
        // More accurate if object is near to the vertical center of the image.
        // Accuracy limited by the accuracy of the drone altitude.
        // Object at the left/right edge of the image are slightly further from the drone than objects directly under the drone.
        public void Calculate_HeightM_BaseLineMovement(
                    ProcessFeature firstRealFeature,
                    float objectDemM,
                    float avgDroneFlightStepFixedAltitudeM) // Measured under the drone
        {
            try
            {
                if ((firstRealFeature == null) ||
                    (firstRealFeature.FeatureId == this.FeatureId) || // Need multiple real distinct features
                    (this.Block == null) ||
                    (this.Block.FlightStep == null))
                {
                    SetHeightAlgorithmError("TooFew");
                    return;
                }

                if (this.Block.FlightStep.FixedDistanceDown < 10)
                {
                    // Drone is too low. This method will not work.
                    SetHeightAlgorithmError("TooLow: " + this.Block.FlightStep.FixedDistanceDown.ToString("0.0"));
                    return;
                }

                double firstPixelY = firstRealFeature.PixelBox.Y + firstRealFeature.PixelBox.Height / 2.0;
                double lastPixelY = this.PixelBox.Y + PixelBox.Height / 2.0;
                if (firstPixelY == lastPixelY)
                {
                    // Object has no changed image position. This method will not work.
                    SetHeightAlgorithmError("SameY");
                    return;
                }

                // Drone moved from point A to point B (base-line distance L) in metres.
                double baselineM = RelativeLocation.DistanceM(firstRealFeature.Block.DroneLocnM, this.Block.DroneLocnM);

                // The object may be 10m to left and 40m in front of the drone location
                // Calculation is based on where object appears in the thermal camera field of view (FOV).

                // Get forward-down-angles of the object in the first / last frame detected.
                // A stationary object may be detected from +20 to +15 degrees, or +15 to -4 degrees, or -6 to -8 degrees.
                double firstFwdDegs = firstRealFeature.Calculate_Image_FwdDeg();
                double lastFwdDegs = this.Calculate_Image_FwdDeg();

                // This is the change in angle from drone to object over the first/lastRealFeature frames.
                var fwdDegDiff = firstFwdDegs - lastFwdDegs;
                if (Math.Abs(fwdDegDiff) < 2 )
                {
                    // A small difference in vertical angle moved in direct of flight makes this method too inaccurate to be useful.
                    // Can occur when 1) camera is near horizontal and the target is far away or 2) drone is not moving.
                    SetHeightAlgorithmError("FwdDiffLow: " + fwdDegDiff.ToString("0.0"));
                    return;
                }

                double firstFwdTan = Math.Tan(firstFwdDegs * BaseConstants.DegreesToRadians); // Often postive
                double lastFwdTan = Math.Tan(lastFwdDegs * BaseConstants.DegreesToRadians); // Often negative

                double fwdTanDiff = firstFwdTan - lastFwdTan;
                var trigDownM = baselineM / fwdTanDiff;

                // Returns the average tan of the sideways-down-angles
                // of the object in the first frame detected and the last frame detected.
                // double avgSideRads = Math.Abs(Calculate_Image_AvgSidewaysRads());
                // double avgSideTan = Math.Tan(avgSideRads);
                // PQR TODO Integrate avgSideTan into calcs?? Or too small to matter?

                // Last feature altitude and height
                var featureAltitudeM = avgDroneFlightStepFixedAltitudeM - trigDownM;

                // Calculate the height of the drone above the OBJECT's location DemM.
                var featureHeightM = featureAltitudeM - objectDemM;

                // We show unknown heights as -2. If height is >= -1.9 show it.
                if (featureHeightM >= UnknownHeight + 0.1f)
                {
                    HeightM = (float)featureHeightM;
                    HeightAlgorithm = BaseLineHeightAlgorithm;
                }
                else
                    SetHeightAlgorithmError("BL Neg:" + featureHeightM.ToString("0.000"));
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessFeature.Calculate_HeightM_BaseLineMovement", ex);
            }
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        public override DataPairList GetSettings()
        {
            var settings = base.GetSettings();

            // Derived data that is saved but not reloaded.
            settings.Add("Leg", (Block != null ? Block.FlightLegId : 0));
            // Horizontal distance from feature to drone.
            settings.Add("RangeM", (Block != null ? RelativeLocation.DistanceM(LocationM, Block.DroneLocnM) : 0), LocationNdp);

            return settings;
        }


        // Get the class's settings as datapairs for use in ObjectForm
        public DataPairList GetSettings_ObjectForm(RelativeLocation objectLocation)
        {
            return new DataPairList
            {
                { "Feature", FeatureId },
                { "NorthingDiffCM", ( LocationM != null ? (LocationM.NorthingM - objectLocation.NorthingM ) * 100 : 0), 0},
                { "EastingDiffCM", ( LocationM != null ? (LocationM.EastingM - objectLocation.EastingM) * 100 : 0), 0},
                { "HeightCM", HeightM * 100, 0 },
                { "WidthPixels", PixelBox.Width },
                { "DepthPixels", PixelBox.Height },
                { "#HotPixels", NumHotPixels },
            };
        }
    }


    // A list of Comb or Yolo features 
    public class ProcessFeatureList : SortedList<int, ProcessFeature>
    {
        private static ProcessConfigModel ProcessConfig;

        public ProcessFeature? FirstFeature { get { return (Count == 0 ? null : Values[0]); } }
        // Last (Real or UnReal) feature claimed by this object. May be null.
        public ProcessFeature? LastFeature { get { return (Count == 0 ? null : Values[^1]); } }


        public ProcessFeatureList(ProcessConfigModel config)
        {
            ProcessFeatureList.ProcessConfig = config;
        }


        public void AddFeature(ProcessFeature feature)
        {
            BaseConstants.Assert(feature != null, "AddFeature: Feature not specified.");
            BaseConstants.Assert(feature.FeatureId > 0, "AddFeature: No Id");

            this.Add(feature.FeatureId, feature);
        }


        // Add the feature list for a new block into this ProcessFeatureList
        public void AddFeatureList(ProcessFeatureList featuresToAdd)
        {
            if (featuresToAdd != null)
                foreach (var feature in featuresToAdd)
                    AddFeature(feature.Value);
        }



        // Clone the feature list (not the feature themselves).
        public ProcessFeatureList Clone()
        {
            var answer = new ProcessFeatureList(ProcessConfig);

            foreach (var feature in this)
                answer.AddFeature(feature.Value);

            return answer;
        }


        // Return the number of significant features in this list
        public int NumSig
        {
            get
            {
                int numSig = 0;
                foreach (var theFeature in this)
                    if (theFeature.Value.Significant)
                        numSig++;
                return numSig;
            }
        }



        // Calculate object's location (centroid) as average of real feature's locations, using real features.
        // Also calculate the average error in location relative to the centroid.
        public (DroneLocation, float) Calculate_Avg_LocationM_and_LocationErrM()
        {
            if (this.Count >= 2)
            {
                int sumCount = 0;
                DroneLocation sumLocation = new();

                foreach (var feature in this)
                    if ((feature.Value.LocationM != null) &&
                        (feature.Value.Type == FeatureTypeEnum.Real))
                    {
                        sumCount++;
                        sumLocation.NorthingM += feature.Value.LocationM.NorthingM;
                        sumLocation.EastingM += feature.Value.LocationM.EastingM;
                    }

                if (sumCount > 0)
                {
                    var theLocationM = sumLocation.Multiply(1.0f / sumCount);

                    double sumDist = 0;
                    foreach (var feature in this)
                        if ((feature.Value.LocationM != null) &&
                            (feature.Value.Type == FeatureTypeEnum.Real))
                            sumDist += RelativeLocation.DistanceM(feature.Value.LocationM, theLocationM);
                    var theLocationErrM = (float)(sumDist / sumCount);

                    return (theLocationM.Clone(), theLocationErrM);
                }
            }
            else if (this.Count == 1)
            {
                var firstFeat = this.Values[0];
                if (firstFeat.LocationM != null)
                    return (firstFeat.LocationM.Clone(), 0);
            }

            return (new DroneLocation(), 0);
        }


        // Calculate object height and object height error, using real features.
        // With the BaseLine calculation algorithm the last value is most accurate.
        // With the LineOfSight calculation algorithm every value is equally accurate.
        public (float heightM, float heightErrM, float minHeight, float maxHeight) Calculate_Avg_HeightM_and_HeightErrM()
        {
            int countLOS = 0;
            float sumLOSHeight = 0;
            float minHeight = 9999;
            float maxHeight = BaseConstants.UnknownValue;
            float lastBLHeight = BaseConstants.UnknownValue;
            foreach (var feature in this)
                if ((feature.Value.LocationM != null) &&
                    (feature.Value.Type == FeatureTypeEnum.Real))
                {
                    var featureHeight = feature.Value.HeightM;
                    if (featureHeight > BaseConstants.UnknownHeight)
                    {
                        minHeight = Math.Min(featureHeight, minHeight);
                        maxHeight = Math.Max(featureHeight, maxHeight);
                        if (feature.Value.HeightAlgorithm == ProcessFeature.BaseLineHeightAlgorithm)
                            lastBLHeight = featureHeight;
                        else if (feature.Value.HeightAlgorithm == ProcessFeature.LineOfSightHeightAlgorithm)
                        {
                            countLOS++;
                            sumLOSHeight += featureHeight;
                        }
                    }
                }

            // Use BaseLine value if available, else the LOS value if available.
            var heightM = (lastBLHeight > BaseConstants.UnknownHeight ? lastBLHeight : (countLOS > BaseConstants.UnknownHeight ? (float)(sumLOSHeight / countLOS) : BaseConstants.UnknownValue));
            if (heightM > BaseConstants.UnknownHeight)
                return (
                    heightM,
                    Math.Max(
                        Math.Abs(maxHeight - heightM),
                        Math.Abs(minHeight - heightM)),
                    minHeight, maxHeight);

            return (BaseConstants.UnknownValue, BaseConstants.UnknownValue, BaseConstants.UnknownValue, BaseConstants.UnknownValue);
        }


        // Return the average altitude of the drone over the object features.
        public float AverageFlightStepFixedAltitudeM()
        {
            float answer = 0;
            int count = 0;

            foreach (var feature in this)
            {
                if (feature.Value.Type == FeatureTypeEnum.Real)
                {
                    var step = feature.Value.Block.FlightStep;
                    if (step != null)
                    {
                        var atlM = feature.Value.Block.FlightStep.FixedAltitudeM;
                        if (atlM != BaseConstants.UnknownValue)
                        {
                            answer += atlM;
                            count++;
                        }
                    }
                }
            }
            if (count > 0)
                answer /= count;
            else
                answer = BaseConstants.UnknownValue;

            return answer;
        }


        public (int minHeat, int maxHeat, int maxPixels) HeatSummary()
        {
            int maxHeat = 0;
            int minHeat = 255;
            int maxPixels = 0;
            foreach (var feature in this)
            {
                var combFeature = feature.Value;
                maxHeat = Math.Max(maxHeat, combFeature.MaxHeat);
                if (feature.Value.MinHeat > 0)
                    minHeat = Math.Min(minHeat, combFeature.MinHeat);
                maxPixels = Math.Max(maxPixels, combFeature.NumHotPixels);
            }

            return (minHeat, maxHeat, maxPixels);
        }
    };
}
