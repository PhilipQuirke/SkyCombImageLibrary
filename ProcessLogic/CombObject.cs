// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A significant Comb object - a logical object derived from overlapping features over successive frames. 
    public class CombObject : ProcessObject
    {
        // Parent process model
        private CombProcess CombProcess { get; }

        // First (Real) feature claimed by this object. 
        public CombFeature? FirstFeature { get { return ProcessFeatures.FirstFeature as CombFeature; } }
        public CombFeature? LastRealFeature { get { return (LastRealFeatureIndex == UnknownValue ? null : ProcessFeatures.Values[LastRealFeatureIndex] as CombFeature); } }
        // Last (Real or UnReal) feature claimed by this object. May be null.
        public CombFeature? LastFeature { get { return ProcessFeatures.LastFeature as CombFeature; } }


        // Constructor used processing video 
        public CombObject(ProcessScope scope, CombProcess model, CombFeature firstFeature) : base(model, scope)
        {
            CombProcess = model;
            ResetMemberData();

            if (firstFeature != null)
            {
                Assert(firstFeature.Type == FeatureTypeEnum.Real, "Initial feature must be Real");
                ClaimFeature(firstFeature);
            }
        }


        // Constructor used when loaded objects from the datastore
        public CombObject(CombProcess model, List<string> settings) : base(model, null)
        {
            CombProcess = model;
            ResetMemberData();

            LoadSettings(settings);
        }


        // Number of real features owned by this object.
        public override int NumRealFeatures()
        {
            if (LastRealFeatureIndex >= 0)
                return base.NumRealFeatures();

            return 0;
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


        // Is this object still worth tracking in specified block?
        public bool KeepTracking(int BlockId)
        {
            // Once inactive, a Comb object stops being tracked permanently.
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


        // Key logic of the Comb model to distinguish significant objects from insignificant, based on data collected.
        //
        // Label this object as "Significant" based on these characteristics:
        // 1) Count: the object has >= 5 hot pixels per block
        //      (avoids very small objects. Includes unreal blocks between real blocks, so unreal blocks work against success)
        // 2) Density: The object's rectangle is at least 20% filled with hot pixels
        //      (deselects large sparce rectangles)
        // 3) Time: Duration we have seen the object for. 
        // 4) Elevation: The object and ground elevations differ significantly - parallex implies object is above ground.
        //
        // Key use cases:
        // 1) Animal in tree, partially visible through foliage, moving faster than ground.
        //      Example in test video at 10s to 12s
        // 2) Animal on open ground, fully visible, very dense, stationary.
        //      Example in test video at 2.0s to 2.8s
        // 3) Animal on open ground, fully visible, very dense, moving very slowly.
        //      Example in test video at 37.5s to 39.0s
        public void Calculate_Significant()
        {
            try
            {
                // Debugging - Set breakpoint on assignment. Assignment value is overridden later in this proc.
                if (ObjectId == ProcessConfig.FocusObjectId)
                    if (CombProcess.Blocks.Count >= 17)
                        Significant = false;

                // COUNT
                // Maximum pixel count per real feature
                var maxCount = MaxRealHotPixels;
                var countOk = (maxCount > ProcessConfig.ObjectMinPixelsPerBlock); // Say 5 pixels / Block
                var countGood = (maxCount > 2 * ProcessConfig.ObjectMinPixelsPerBlock); // Say 10 pixels / Block
                var countGreat = (maxCount > 4 * ProcessConfig.ObjectMinPixelsPerBlock); // Say 20 pixels / Block

                // DENSITY
                // Average pixel density based on encompassing pixel block of each real feature.
                var minDensity = ProcessConfig.ObjectMinDensityPerc / 100.0F;
                var density = RealDensityPx();
                var densityOk = (density > minDensity); // Say 33%
                var densityGood = (density > 1.5 * minDensity); // Say 50%
                var densityGreat = (density > 2 * minDensity); // Say 66%

                // TIME
                // Aka duration. Proxy for numRealFeatures.
                var seenForMinDurations = SeenForMinDurations();
                var timeOk = (seenForMinDurations >= 1); // Say 500ms
                var timeGood = (seenForMinDurations >= 2); // Say 1000ms
                var timeGreat = (seenForMinDurations >= 4); // Say 2000ms

                // ELEVATION
                // Object with a significant height above ground are more interesting 
                var elevationOK = (HeightM >= 0);
                var elevationGood = (HeightM > 2);
                var elevationGreat = (HeightM > 4);

                // Key calculation of Comb algorithm for identifying significant objects
                Significant =
                    countOk &&
                    densityOk &&
                    timeOk &&
                    (
                        elevationGreat ||
                        countGreat ||
                        densityGreat ||
                        (densityGood && countGood)
                    );

                if (Significant)
                    NumSigBlocks++;

                // Summarise why object is significant or not. Gets displayed in UI and saved to xls.
                Attributes = String.Format("{0}: {1} {2} {3} {4}",
                    Significant ? "Yes" : "No",
                    countGreat ? "C3" : (countGood ? "C2" : (countOk ? "C1" : "c")),
                    densityGreat ? "D3" : (densityGood ? "D2" : (densityOk ? "D1" : "d")),
                    timeGreat ? "T3" : (timeGood ? "T2" : (timeOk ? "T1" : "t")),
                    elevationGreat ? "E3" : (elevationGood ? "E2" : (elevationOK ? "E1" : "e")));
            }
            catch (Exception ex)
            {
                throw ThrowException("CombObject.Calculate_Significant", ex);
            }
        }


        // A CombObject can't be significant until Config.ObjectMinDurationMs has passed.
        // A CombFeature can be significant immediately.
        // This "VaguelySignificant" object code mirrors the feature "Significant" code
        public bool VaguelySignificant()
        {
            // COUNT
            // Maximum pixel count per real feature
            var maxCount = MaxRealHotPixels;
            var maxCountOk = (maxCount > ProcessConfig.ObjectMinPixelsPerBlock); // Say 5 pixels / Block

            // Density
            var minDensity = ProcessConfig.ObjectMinDensityPerc / 100.0F;
            var density = RealDensityPx();
            var densityOk = (density > minDensity); // Say 20%

            return maxCountOk && densityOk;
        }


        // Calculate the simple (int, float, VelocityF, etc) member-data of this real object.
        // Calculates LocationM, LocationErrM, HeightM, HeightErrM, etc.
        public void Calculate_RealObject_SimpleMemberData_Core()
        {
            try
            {
                FirstFwdDownDeg = (float)FirstFeature.Calculate_Image_FwdDeg();
                LastFwdDownDeg = (float)LastRealFeature.Calculate_Image_FwdDeg();

                // First estimate of OBJECT location (centroid) as average over all real features.
                // The feature locations are based on where in the drone's field of image the object was detected,
                // and takes into account ground undulations. The object's location is further refined later.
                Calculate_LocationM_and_LocationErrM();

                // Estimate the OBJECT's DEM at the object's location
                // (which could be say 20m left of the drone's flight path).
                Calculate_DemM();

                // In RunVideoCombDrone.AddBlockAndProcessInputVideoFrame, a new CombFeature is created
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
                throw ThrowException("CombObject.Calculate_RealObject_SimpleMemberData_Core", ex);
            }
        }


        // Calculate the simple (int, float, VelocityF, etc) member-data of this real object.
        // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
        public void Calculate_RealObject_SimpleMemberData()
        {
            // Calculate the drone SumLinealM distance corresponding to the centroid of the object
            Calculate_AvgSumLinealM();

            Calculate_RealObject_SimpleMemberData_Core();
        }


        // After loading data from a previous run from the data store
        // we have blocks, features and objects, but the links between them are not set.
        public void SetLinksAfterLoad(CombFeature feature)
        {
            Assert(feature != null, "SetLinksAfterLoad: Feature not provided.");
            Assert(feature.ObjectId == ObjectId, "SetLinksAfterLoad: Feature not for this object.");

            ProcessFeatures.AddFeature(feature);
            if (feature.Type == FeatureTypeEnum.Real)
                LastRealFeatureIndex = ProcessFeatures.Count - 1;
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        // But only if the object reamins viable after claiming feature (e.g. doesn't get too big or density too low).
        public bool ClaimFeature(CombFeature theFeature)
        {
            try
            {
                var lastFeature = LastFeature;
                if ((theFeature.Type == FeatureTypeEnum.Real) && (lastFeature != null))
                {
                    // To get here, theFeature overlaps this object significantly.
                    // But claiming theFeature can make this object exceed FeatureMaxSize
                    // or reduce the density below FeatureMinDensityPerc, potentially making the object insignificant.

                    if (theFeature.FeatureOverSized)
                        return false;
                    if (!theFeature.PixelDensityGood)
                        return false;

                    // ToDo: This approach allows one bad block before it stops growth. Bad. May make object insignificant.
                    if (lastFeature.FeatureOverSized)
                        return false;
                    if ((lastFeature.Type == FeatureTypeEnum.Real) && // Unreal features have no density
                        !lastFeature.PixelDensityGood)
                        return false;
                }


                // Associate the feature with this object.
                Assert(theFeature.ObjectId <= 0, "CombObject.ClaimFeature: Feature is already owned.");
                theFeature.ObjectId = this.ObjectId;

                // Debugging - Set breakpoint on assignment. (Value is unchanged by assignment)
                if (ObjectId == ProcessConfig.FocusObjectId)
                    theFeature.ObjectId = this.ObjectId;

                bool wasSignificant = Significant;

                // Is object a real feature?
                if (theFeature.Type == FeatureTypeEnum.Real)
                {
                    theFeature.IsTracked = true;
                    MaxRealHotPixels = Math.Max(MaxRealHotPixels, theFeature.NumHotPixels);

                    var theBlock = theFeature.Block;

                    if ((LastRealFeature == null) || (LastRealFeature.Block.BlockId < theBlock.BlockId))
                    {
                        // First real feature claimed by this object for THIS block
                        ProcessFeatures.AddFeature(theFeature);
                        LastRealFeatureIndex = ProcessFeatures.Count - 1;
                        RunToVideoS = (float)(LastRealFeature.Block.InputFrameMs / 1000.0);

                        MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, theFeature.PixelBox.Width);
                        MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, theFeature.PixelBox.Height);
                    }
                    else
                    {
                        // This object is claiming a second or third feature for this block.
                        // Use case is a large rectangle in previous block, getting replaced by 2 or 3 smaller rectangles in this block.
                        // For better visualisation we want to combine all features in this block into one.

                        // The first real feature for the last block consumes theFeature, leaving theFeature empty.
                        LastRealFeature.Consume(theFeature);
                        theFeature.ObjectId = UnknownValue;

                        MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, LastRealFeature.PixelBox.Width);
                        MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, LastRealFeature.PixelBox.Height);
                    }

                    // Calculate the simple member data (int, float, VelocityF, etc) of this real object.
                    // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
                    Calculate_RealObject_SimpleMemberData();
                }
                else if (theFeature.Type == FeatureTypeEnum.Unreal)
                {
                    // theFeature is unreal - it is a persistance object
                    ProcessFeatures.AddFeature(theFeature);

                    LastFeature.HeightM = HeightM;
                    LastFeature.HeightAlgorithm = CombFeature.UnrealCopyHeightAlgorithm;
                }


                // Copy these details to the feature to be saved in the DataStore.
                // Useful for understanding the feature by feature progression of values that are refined over time.
                LastFeature.Significant = Significant & (LastFeature.Type == FeatureTypeEnum.Real);
                LastFeature.Attributes = Attributes;
                if (Significant && !wasSignificant)
                    // This object has just become significant. Two use cases:
                    // - After 5 real features, enough time has based for object to become significant on 6th real feature.
                    // - Object had 20 real features, then 5 unreal features, then another 1 real features.
                    // Mark all (real and unreal) features associated with this object as significant.
                    foreach (var feature in ProcessFeatures)
                        feature.Value.Significant = true;

                // Debugging - Set breakpoint on assignment. 
                if (ObjectId == ProcessConfig.FocusObjectId)
                    if ((theFeature.FeatureId == 41) || (theFeature.FeatureId == 59))
                        LastFeature.Attributes = this.Attributes + ".";

                return true;
            }
            catch (Exception ex)
            {
                throw ThrowException("CombObject.ClaimFeature", ex);
            }
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        // But only if the object reamins viable after claiming feature (e.g. doesn't get too big or density too low).
        public bool MaybeClaimFeature(CombFeature feature, Rectangle objectExpectedPixelBox)
        {
            if (feature.ObjectId == 0) // Not claimed yet
                if (feature.Significant || this.Significant)
                    if (feature.SignificantPixelBoxIntersection(objectExpectedPixelBox))
                        // Object will claim feature if the object remains viable after claiming feature
                        return ClaimFeature(feature);

            return false;
        }


        // Calculate the drone SumLinealM distance corrsponding to the centroid of the object
        private void Calculate_AvgSumLinealM()
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
        private void Calculate_LocationM_and_LocationErrM()
        {
            (LocationM, LocationErrM) = ProcessFeatures.Calculate_Avg_LocationM_and_LocationErrM();
        }


        // Returns the average sideways-down-angles (right angles to the direction of flight)
        // of the object in the first image detected and the last image detected.
        // Only uses camera physics and hot-object position in image data.
        private double Calculate_Image_AvgSidewaysRads()
        {
            (var xFracFirst, var _) = FirstFeature.CentroidImageFractions();
            (var xFracLast, var _) = LastFeature.CentroidImageFractions();

            // Calculation is based on physical parameters of the thermal camera.
            double fullHorizFoVRadians = CombProcess.VideoData.HFOVRad;
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


        private bool HasMoved()
        {
            // Drone moved from point A to point B (base-line distance L) in metres.
            double baselineM = RelativeLocation.DistanceM(
                FirstFeature.Block.DroneLocnM,
                LastRealFeature.Block.DroneLocnM);
            // If drone has not moved enough this method will be very inaccurate.
            return (baselineM >= 2);
        }


        // Calculate object height and object height error by averaging the feature data.
        private void Calculate_HeightM_and_HeightErrM()
        {
            (HeightM, HeightErrM, MinHeightM, MaxHeightM) = ProcessFeatures.Calculate_Avg_HeightM_and_HeightErrM();
        }


        // Calculate the size of the object in square centimeters.
        // Based on MAXIMUM number of hot pixels in any real feature.
        private void Calculate_SizeCM2()
        {
            if ((CombProcess == null) || (CombProcess.VideoData == null))
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
            float framePixels = CombProcess.VideoData.ImageWidth * CombProcess.VideoData.ImageHeight;

            // Calculate the size of the object in this frame in square centimeters
            float thisSizeM2 = imageAreaM2 * hotPixels / framePixels;

            var thisSizeCM2 = (int)(thisSizeM2 * 100 * 100);

            SizeCM2 = Math.Max(SizeCM2, thisSizeCM2);
        }


        // Calculate the average horizontal range of the object from the drone in meters.
        // Relies on object.LocationM already being calculated.
        private void Calculate_AvgRangeM()
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


        // Calculate the maximum heat value of any pixel in this object in any frame 
        private void Calculate_MaxHeat()
        {
            (int _, int maxHeat, int _) = CombFeatureLogic.HeatSummary(ProcessFeatures);
            MaxHeat = maxHeat;
        }

/*
        // Get the object's DEM at the OBJECT'S location.
        private void Calculate_DemM()
        {
            if ((LocationM == null) || (CombProcess.GroundData == null) || (CombProcess.GroundData.DemModel == null))
                return;

            // Most accurate method. Nearly always works.
            var newDemM = CombProcess.GroundData.DemModel.GetElevationByDroneLocn(LocationM);
            if (newDemM != UnknownValue)
            {
                DemM = newDemM;
                return;
            }

            // In rare cases, we have an object just outside ground datum grid.
            // Object may be say 10m to left and 40m ahead of the drone's location.
            // Forced to use less progressively less accurate methods.
            var firstFeat = FirstFeature;
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

            var firstFeat = FirstFeature;
            var lastFeat = LastFeature;

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

            if(lastFeat.Type == FeatureTypeEnum.Real)
                // We don't want a drone wobble to break the object feature sequence
                // So we inflate the expected location by 5 pixels in each direction.
                answer.Inflate(5, 5);

            return answer;
        }
*/

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
    };


    public class CombObjList : ProcessObjList  
    {
 
        public CombObjList()
        {
            ResetSettings();
        }


        // Get the minimum StepId of all objects
        public int GetMinStepId()
        {
            if (Count == 0)
                return BaseConstants.UnknownValue;

            int answer = 9999999;
            foreach (var theObject in this)
            {
                var combObject = theObject.Value as CombObject;
                var firstFeat = combObject.FirstFeature;
                if (firstFeat != null)
                    answer = Math.Min(answer, firstFeat.Block.FlightStepId);
            }
            return answer;
        }

    }


    // A list of Comb objects
    public class CombObjs
    {
        private readonly CombProcess Model;

        public CombObjList CombObjList;


        public CombObjs(CombProcess model)
        {
            Model = model;
            CombObjList = new();
        }


        public void Add(ProcessScope scope, CombFeature firstFeature)
        {
            var theObject = ProcessFactory.NewCombObject(scope, Model, firstFeature);
            CombObjList.AddObject(theObject);
        }


        // After loading data from a previous run from the data store
        // we have blocks, features and objects, but the links between them are not set.
        // Apply the feature to the corresponding object.
        public void SetLinksAfterLoad(CombFeature feature)
        {
            if ((feature == null) || (feature.ObjectId <= 0))
                return;

            CombObjList.TryGetValue(feature.ObjectId, out var combObject);

            // Null case occurs when saving All features but only Significant objects and load from datastore
            if (combObject != null)
                (combObject as CombObject).SetLinksAfterLoad(feature);
        }


        // Number of objects that have ever been significant. 
        // Not same as num objs significant in the current Block, as objects can become insignificant. 
        public int NumSig
        {
            get
            {
                int numSig = 0;
                foreach (var theObject in CombObjList)
                    if (theObject.Value.NumSigBlocks > 0)
                        numSig++;
                return numSig;
            }
        }


        public void StopTracking()
        {
            foreach (var theObject in CombObjList)
                theObject.Value.BeingTracked = false;
        }


        public string DescribeSignificantObjects()
        {
            var num = NumSig;
            return (num == 0 ? "" : string.Format("{0} Objects", num));
        }


        // Returns key attributes of objects and associated user annotations (if any)
        // to show in the ObjectGrid in the Main Form or the ObjectList in the Object Form
        public List<object[]> GetObjectGridData(ProcessScope scope, bool mainForm, ObjectCategoryList annotations, int focusObjectID = BaseConstants.UnknownValue)
        {
            var answer = new List<object[]>();

            var sigObjects = CombObjList.FilterByProcessScope(scope, focusObjectID);
            foreach (var theObject in sigObjects)
            {
                ObjectCategoryModel? annotation = null;
                if (annotations != null)
                    annotation = annotations.GetData(theObject.Value.Name);

                answer.Add(theObject.Value.GetObjectGridData(Model.ProcessConfig, mainForm, annotation));
            }

            return answer;
        }
    }
}
