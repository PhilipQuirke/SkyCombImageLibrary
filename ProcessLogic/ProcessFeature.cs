// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombDrone.DroneLogic;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A process feature combines the stored data and some logic
    public class ProcessFeature : ProcessFeatureModel
    {
        // Parent process model
        protected ProcessAll ProcessAll { get; }

        // A feature is associated 1-1 with a Block
        public ProcessBlock Block { get; set; }


        public ProcessFeature(ProcessAll processAll, int blockId, FeatureTypeEnum type) : base(blockId, type)
        {
            ResetMemberData();

            ProcessAll = processAll;
            Block = processAll.Blocks[blockId];
        }


        // Constructor used when loaded objects from the datastore
        public ProcessFeature(ProcessAll processAll, List<string> settings) : base(settings)
        {
            ProcessAll = processAll;
            Block = ProcessAll.Blocks[BlockId];
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

            otherFeature.Attributes = "";
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
            double xCenterPixels = PixelBox.X + PixelBox.Width / 2.0;
            double yCenterPixels = PixelBox.Y + PixelBox.Height / 2.0;

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
            Assert(fwdFraction >= -1 && fwdFraction <= 1, "Calculate_Image_FwdDeg: fwdFraction out of range: " + fwdFraction.ToString());

            double fwdDeg = halfVertFoVDeg * fwdFraction; // Often close to +16
            Assert(Math.Abs(fwdDeg) <= halfVertFoVDeg, "Calculate_Image_FwdDeg: fwdDeg out of range: " + fwdDeg.ToString());

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
                        SetHeightAlgorithmError("LOS_NoDsm");
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
                                SetHeightAlgorithmError("LOS_NoDem");
                        }
                        else
                            SetHeightAlgorithmError("LOS_NoDem");

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


        // Estimate last FEATURE height above ground based on distance down from drone
        // calculated using trigonometry and first/last real feature camera-view-angles.
        // This is a "look down" trig method. Accuracy limited by the accuracy of the drone altitude.
        // Object at the left/right edge of the image are slightly further from the drone
        // than objects directly under the drone.
        // Only works if the drone has moved horizontally a distance. Works at 1m. Better at 5m
        // If drone is not moving now, calculated HeightM will be the same as last feature (within Gimbal wobble). 
        public void Calculate_HeightM_BaseLineMovement(
                    ProcessFeature firstRealFeature,
                    float demM,
                    float averageFlightStepFixedAltitudeM)
        {
            try
            {
                if ((firstRealFeature == null) ||
                    (firstRealFeature.FeatureId == this.FeatureId) || // Need multiple real distinct features
                    (this.Block == null) || // Last real feature
                    (this.Block.FlightStep == null))
                {
                    SetHeightAlgorithmError("BL_TooFew");
                    return;
                }

                // If drone is too low this method will not work.
                var lastStep = this.Block.FlightStep;
                float droneDistanceDownM = lastStep.FixedDistanceDown;
                if (droneDistanceDownM < 5)
                {
                    SetHeightAlgorithmError("BL_TooLow");
                    return;
                }


                // Drone moved from point A to point B (base-line distance L) in metres.
                double baselineM = RelativeLocation.DistanceM(firstRealFeature.Block.DroneLocnM, this.Block.DroneLocnM);

                // The object may be 10m to left and 40m in front of the drone location
                // Calculate the height of the drone above the OBJECT's location DemM.
                var groundDownM = averageFlightStepFixedAltitudeM - demM;

                // Calculation is based on where object appears in the thermal camera field of view (FOV).
                // If object does not appear to have changed image position then this method will not work.
                double firstPixelY = firstRealFeature.PixelBox.Y + ((double)firstRealFeature.PixelBox.Height) / 2.0;
                double lastPixelY = this.PixelBox.Y + ((double)this.PixelBox.Height) / 2.0;
                if (firstPixelY == lastPixelY)
                {
                    SetHeightAlgorithmError("BL_SameY");
                    return;
                }


                // Get forward-down-angles of the object in the first / last frame detected.
                // In DJI_0116, leg 4, objects are detected from +15 to -4 degrees.
                double firstFwdDegs = firstRealFeature.Calculate_Image_FwdDeg();
                double lastFwdDegs = this.Calculate_Image_FwdDeg();

                double firstFwdTan = Math.Tan(firstFwdDegs * BaseConstants.DegreesToRadians); // Often postive
                double lastFwdTan = Math.Tan(lastFwdDegs * BaseConstants.DegreesToRadians); // Often negative

                // This is the change in angle from drone to object over the first/lastRealFeature frames.
                double fwdTanDiff = firstFwdTan - lastFwdTan;

                // If the difference in vertical angle moved in direct of flight is too small,
                // this method will be too inaccurate to be useful.
                // Can occur when 1) camera is near horizontal and the target is far away or 2) drone is not moving.
                if (Math.Abs(fwdTanDiff) < 0.1) // 0.1 rads = ~6 degrees
                {
                    SetHeightAlgorithmError("BL_TanDiff");
                    return;
                }


                // Returns the average tan of the sideways-down-angles
                // of the object in the first frame detected and the last frame detected.
                // Drone may be stationary but rotating.
                // double avgSideRads = Math.Abs(Calculate_Image_AvgSidewaysRads());
                // double avgSideTan = Math.Tan(avgSideRads);
                // PQR TODO Integrate avgSideTan into calcs?? Or too small to matter?

                var trigDownM = baselineM / fwdTanDiff;

                var featureHeightM = (float)(groundDownM - trigDownM);

                if (featureHeightM >= 0)
                {
                    HeightM = featureHeightM;
                    HeightAlgorithm = BaseLineHeightAlgorithm;
                }
                else
                    SetHeightAlgorithmError("BL_Neg");
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessFeature.Calculate_HeightM_BaseLineMovement", ex);
            }
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


        // Add the feature list for a new block into this CombFeatureListList 
        public void AddFeatureList(ProcessFeatureList featuresToAdd)
        {
            if (featuresToAdd != null)
                foreach (var feature in featuresToAdd)
                    AddFeature(feature.Value);
        }


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

    };
}
