// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;



namespace SkyCombImage.ProcessLogic
{
    // A Comb feature, is a dense cluster of hot pixels, associated 1-1 with a Block
    public class CombFeature : ProcessFeatureModel
    {
        // Static config data shared by all Comb features
        public static ProcessConfigModel? Config = null;
        // Static NextFeatureId shared by all Comb features
        public static int NextFeatureId = 0;


        // Parent process model
        private CombProcessAll? Model { get; } = null;

        // A Comb feature is associated 1-1 with a Block
        public ProcessBlock? Block { get; set; } = null;

        // Location of hot pixels in this feature.
        public PixelHeatList? Pixels { get; set; } = null;

        // All the CombFeature specific attributes that are persisted to the DataStore
        public CombFeatureModel CFM { get; set; }



        public CombFeature(CombProcessAll model, ProcessBlock block, CombFeatureTypeEnum type) : base(++NextFeatureId)
        {
            CFM = new(block.BlockId, type);
            if (CFM.Type != CombFeatureTypeEnum.Unreal)
                Pixels = new();
            ResetMemberData();

            Model = model;
            Block = block;
        }


        // Constructor used when loaded objects from the datastore
        public CombFeature(CombProcessAll model, int featureID, List<string> settings) : base(featureID)
        {
            CFM = new(UnknownValue, CombFeatureTypeEnum.Real); // Values overridden in LoadSettings.
            ResetMemberData();
            LoadSettings(settings);

            Model = model;
            Block = Model.Blocks[CFM.BlockId];
        }


        // Reset member data to mirror a newly created feature.
        // Used in experimentation to allow repeated calculation run against this feature.
        public override void ResetMemberData()
        {
            base.ResetMemberData();

            CFM.ResetMemberData();
        }


         // Number of hot pixels inside the PixelBox
        public int NumHotPixels()
        {
            if (Pixels != null)
                return Pixels.Count;
            return 0;
        }


        // Percentage of PixelBox which is hot pixels
        public int DensityPerc()
        {
            if (Pixels != null)
                return (int)((100.0f * Pixels.Count) / (CFM.PixelBox.Width * CFM.PixelBox.Height));
            return 0;
        }


        // Is this feature's hot pixel density percentage above the minimum?
        public bool PixelDensityGood()
        {
            return DensityPerc() >= Config.FeatureMinDensityPerc;
        }


        // Is this feature larger than the largest allowed?
        public bool FeatureOverSized()
        {
            return
                (CFM.PixelBox.Width > Config.FeatureMaxSize) ||
                (CFM.PixelBox.Height > Config.FeatureMaxSize);
        }


        // Does this Feature's rectangles and the specified object's rectangle overlap significantly?
        public bool SignificantIntersection(Rectangle objectExpectedLocation)
        {
            var intersection = Rectangle.Intersect(CFM.PixelBox, objectExpectedLocation);

            // Refer https://stackoverflow.com/questions/9324339/how-much-do-two-rectangles-overlap
            // SI = Max(0, Min(XA2, XB2) - Max(XA1, XB1)) * Max(0, Min(YA2, YB2) - Max(YA1, YB1))
            // SU = SA + SB - SI
            // OverlapFraction = SI / SU
            var sizeIntersection = intersection.Width * intersection.Height;

            var sizeA = CFM.PixelBox.Width * CFM.PixelBox.Height;
            var sizeB = objectExpectedLocation.Width * objectExpectedLocation.Height;

            var resultA = 1.0F * sizeIntersection / sizeA;
            var resultB = 1.0F * sizeIntersection / sizeB;

            var minOverlap = Config.FeatureMinOverlapPerc / 100.0F;

            return
                resultA >= minOverlap ||   // 25% overlap of rectA
                resultB >= minOverlap;     // 25% overlap of rectB
        }


        // Search for hot neighbours of an initial hot pixel
        //      - Minimum search size is 3 x 3
        //      - Maximum search size if config.FeatureMaxSize x config.FeatureMaxSize
        //      - For each pixel considered
        //          - Set inputSearch[y,x] = true
        //          - If imgThreshold[y,x] is a hot pixel
        //              - Add ([y,x],heat) to hotPixels list
        //      - Search until each edge finds zero hot pixels.
        // Search copes with "+" shaped hot pixels, expanding the search range left and right as needed.
        // By design:
        //      - The search can EXPAND horizontally (left or right) multiple pixels on one row
        //      - The search can SHRINK horizontally (left or right) one pixel on one row
        public void PixelNeighborSearch(
            ref bool[] inputSearched,
            Image<Bgr, byte> imgOriginal,
            Image<Gray, byte> imgThreshold,
            int startY,
            int startX)
        {
            int currY = 0;
            int currX = 0;

            try
            {
                CFM.MinHeat = 255 + 255 + 255;
                CFM.MaxHeat = 0;

                // Start with 3 pixels wide. This width will vary row by row
                int fromX = 0;
                int toX = 3;
                int rectTop = startY;
                int rectLeft = startX + fromX;
                int rectRight = startX + toX - 1;

                // Search down the image
                for (currY = startY; currY < Model.VideoData.ImageHeight; currY++)
                {
                    int hotPixelsInRow = 0;

                    // CASE: EXPAND LEFT
                    // If the FIRST (left edge) pixel being searched on row is hot
                    // EXPAND the left edge, multiple pixels if necessary,
                    // to bring into scope all 'attached' hot pixels on THIS row.
                    while ((imgThreshold.Data[currY, startX + fromX, 0] != 0) && (startX + fromX > 0))
                        fromX--;

                    for (currX = startX + fromX; (currX < startX + toX) && (currX < Model.VideoData.ImageWidth); currX++)
                    {
                        // Set inputSearched[y,x] = true
                        inputSearched[currY * Model.VideoData.ImageWidth + currX] = true;

                        // If imgInputGray[y,x] is a hot pixel
                        var currPixelIsHot = (imgThreshold.Data[currY, currX, 0] != 0);
                        if (currPixelIsHot)
                        {
                            hotPixelsInRow++;

                            // Evaluate the heat from the original image (not the smooth / threshold image)
                            int currHeat = (
                                imgOriginal.Data[currY, currX, 0] +
                                imgOriginal.Data[currY, currX, 1] +
                                imgOriginal.Data[currY, currX, 2]) / 3;

                            CFM.MinHeat = Math.Min(CFM.MinHeat, currHeat);
                            CFM.MaxHeat = Math.Max(CFM.MaxHeat, currHeat);
                            Pixels.Add(new PixelHeat(CFM.BlockId, FeatureId, currY, currX, currHeat));

                            // Expand rectangle to include the hot pixel.
                            if (currX < rectLeft) rectLeft = currX;
                            if (currX > rectRight) rectRight = currX;
                        }

                        // If the FIRST (left edge) pixel being searched on row is hot...
                        if (currX == startX + fromX)
                        {
                            if (currPixelIsHot)
                            // CASE: EXPAND LEFT. Applies to THIS row search. Covered in reloop code above
                            { }
                            else
                                // CASE: SHRINK LEFT. Applies to NEXT row search
                                fromX++;
                        }

                        // If we are looking at LAST (rightmost) pixel to be searched...
                        if (currX == startX + toX - 1)
                        {
                            if (currPixelIsHot)
                                // CASE: EXPAND RIGHT EDGE. Applies to THIS row search
                                toX++;
                            else
                                // CASE: SHRINK RIGHT EDGE. Applies to NEXT row search
                                toX--;
                        }
                    }

                    // If we did not find any hot pixels in this pass then go no further down
                    if (hotPixelsInRow == 0)
                        break;

                    int rectBottom = currY;
                    CFM.PixelBox = new Rectangle(rectLeft, rectTop, rectRight - rectLeft + 1, rectBottom - rectTop + 1);

                    // If the rectangle density is lower than the density threshold then stop expanding. Avoids large, difuse features.
                    if (!PixelDensityGood())
                        break;

                    // If this fearure is larger than allowed then stop expanding
                    if (FeatureOverSized())
                        break;
                }

                // Is this feature significant?
                bool sizeOk = (NumHotPixels() >= Config.FeatureMinPixels);
                bool densityOk = PixelDensityGood();
                Significant = sizeOk && densityOk;
                if (Significant)
                    Attributes = "Yes";
                else
                    Attributes = string.Format("No: {0}{1}",
                        sizeOk ? "P" : "p",
                        densityOk ? "D" : "d");
                IsTracked = Significant;
            }
            catch (Exception ex)
            {
                throw ThrowException("CombFeature.PixelNeighborSearch: " +
                    "startY=" + startY + " startX=" + startX + " currY=" + currY + " currX=" + currX,
                    ex);
            }
        }


        // This feature consumes/absorbs/takes-hot-pixels-from the otherFeature, leaving otherFeature empty.
        public void Consume(CombFeature otherFeature)
        {
            // Transfer the pixels
            this.Pixels.AddRange(otherFeature.Pixels);
            otherFeature.Pixels = null;

            // Expand PixelBox
            var thisRect = this.CFM.PixelBox;
            var otherRect = otherFeature.CFM.PixelBox;
            var answerLeft = Math.Min(thisRect.X, otherRect.X);
            var answerTop = Math.Min(thisRect.Y, otherRect.Y);
            var answerRight = Math.Max(thisRect.X + thisRect.Width, otherRect.X + otherRect.Width);
            var answerBottom = Math.Max(thisRect.Y + thisRect.Height, otherRect.Y + otherRect.Height);
            this.CFM.PixelBox = new Rectangle(answerLeft, answerTop, answerRight - answerLeft + 1, answerBottom - answerTop + 1);

            // Keep otherFeature pointing at ObjectID so we know which object decided to consume it.
            // otherFeature.ObjectID = 0;

            otherFeature.Attributes = "";
            otherFeature.Significant = false;
            otherFeature.IsTracked = false;
            otherFeature.CFM.Type = CombFeatureTypeEnum.Consumed;
        }


        // Calculate center (centroid) of feature in image as fraction 0 to 1
        // If yImageFrac = 0 then object is at the very bottom of the image (furtherest from drone)
        // If yImageFrac = 1/2 then object is in the middle of the image 
        // If yImageFrac = 1 then object is at the top of the image (closest to drone)
        // Y = 1 is the top of the image, closest to the drone. 
        public (double xFraction, double yFraction) CentroidImageFractions(bool initialCalc = true)
        {
            double xCenterPixels = CFM.PixelBox.X + CFM.PixelBox.Width / 2.0;
            double yCenterPixels = CFM.PixelBox.Y + CFM.PixelBox.Height / 2.0;

            // Calculate position of center of feature as fraction of drone image area.
            double xFraction = xCenterPixels / Model.VideoData.ImageWidth;
            // With image pixels, y = 0 is the top of the image. 
            // Here we change the "sign" of Y, so that y = 0 is the bottom of the image.
            double yFraction = (Model.VideoData.ImageHeight - yCenterPixels) / Model.VideoData.ImageHeight;

            if (initialCalc)
            {
                // Should be between 0 and 1, but allow for rounding errors.
                Assert(xFraction >= -0.1 && xFraction <= 1.1, "CentroidImageFractions: xFraction out of range");
                Assert(yFraction >= -0.1 && yFraction <= 1.1, "CentroidImageFractions: yFraction out of range");
            }

            return (xFraction, yFraction);
        }


        // Returns the forward-down-angle (to vertical) of the object in the feature.
        // Only uses camera physics and object position in image data.
        public double Calculate_Image_FwdDeg()
        {
            // Unitless numbers
            // If yImageFrac = 0 then object is at the very bottom of the image (furtherest from drone) => FwdDegs is +16
            // If yImageFrac = 1/2 then object is in the middle of the image => FwdDegs is 0
            // If yImageFrac = 1 then object is at the top of the image (closest to drone) => FwdDegs is -16
            (var _, var yImageFrac) = this.CentroidImageFractions();

            // Calculation is based on physical parameters of the camera.
            double fullVertFoVDeg = Model.VideoData.VFOVDeg; // Say 32 degrees
            double halfVertFoVDeg = fullVertFoVDeg / 2; // Say 16 degrees

            // Calculate the angle to object, in direction of flight (forward), to the vertical, in degrees
            // Assumes drone is moving forward (not sidewards or backwards) or stationary.
            double fwdFraction = yImageFrac * 2 - 1;
            Assert(fwdFraction >= -1 && fwdFraction <= 1, "Calculate_Image_FwdDeg: fwdFraction out of range");

            double fwdDeg = halfVertFoVDeg * fwdFraction; // Often close to +16
            Assert(Math.Abs(fwdDeg) <= halfVertFoVDeg, "Calculate_Image_FwdDeg: fwdDeg out of range");

            var cameraToVertDeg = this.Block.FlightStep.CameraToVerticalForwardDeg;

            Assert(cameraToVertDeg >= 0 && cameraToVertDeg <= 90, "Calculate_Image_FwdDeg: Bad cameraToVertDeg");

            fwdDeg += cameraToVertDeg;

            return fwdDeg;
        }


        // Calculate the location (centroid) of this feature inside the drone imaging box
        // This is the key translation from IMAGE to PHYSICAL coordinate system.
        // If DSM or DEM data is available then considers ground level undulations between the drone and the feature.
        // Assumes ground is flat and object is on the ground.
        public void CalculateSettings_LocationM_FlatGround(CombFeature? lastRealFeature, bool initialCalc = true)
        {
            if ((CFM.LocationM != null) || (Block.FlightStep == null) || (Block.FlightStep.InputImageCenter == null))
                return;
            var flightStep = Block.FlightStep;

            if ((CFM.Type == CombFeatureTypeEnum.Unreal) && (lastRealFeature != null))
            {
                // For unreal features, just copy the last real feature's location
                CFM.LocationM = lastRealFeature.CFM.LocationM?.Clone();
                CFM.HeightM = lastRealFeature.CFM.HeightM;
                return;
            }

            // Calculate center (centroid) of feature in image pixels.
            (double xFraction, double yFraction) = CentroidImageFractions(initialCalc);

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
                flightStep.CalcImageFeatureLocationM(deltaBlockLocnM, xFraction, yFraction, initialCalc)
                    ?.Clone();

            // First approximation of location is based on flat land assumption.
            CFM.LocationM = flatLandLocationM;
        }


        // Calculate land contour undulations impact on object location and height.
        // The DSM level at flatLandLocationM may be higher or lower than the ground level below the drone.
        // Start half-way between the drone location and move stepwise to the flatLandLocationM and some distance beyond.
        // Compare the sight-line height at each step to the DSM level at that location.
        // Stop when the drone sight-line intersects the DSM level.
        // Algorithm works even if drone is stationary. CameraToVerticalForwardDeg must be between 10 and 80 degrees
        public void CalculateSettings_LocationM_HeightM_LineofSight(bool initialCalc = true)
        {
            if ((Block.FlightStep == null) || (Block.FlightStep.InputImageCenter == null))
                return;
            var flightStep = Block.FlightStep;

            // Algorithm does not work if camera is pointing straight down.
            // Algorithm works inaccurately if the camera is pointing at the horizon.
            var fwdToVertDeg = flightStep.CameraToVerticalForwardDeg;
            if ((fwdToVertDeg < 10) || (fwdToVertDeg > 80))
                return;

            if ((CFM.LocationM == null) || (Model.GroundData == null))
                return;
            var flatLandLocationM = CFM.LocationM;
            var groundModel = Model.GroundData.HasDsmModel ? Model.GroundData.DsmModel : Model.GroundData.DemModel;
            if(groundModel == null)
                return;

            // We use the drone camera's forward-down-angle (to vertical) to calculate the step-down distance (per 1 m horizontal).
            // (Not the object as the object may be at the edge of the image with a FwdDeg of ~0.

            var tan = Math.Tan(fwdToVertDeg * DegreesToRadians);
            if (tan == 0)
                return;
            float vertStepDownPerHorizM = (float)(1.0 / tan);
            if (vertStepDownPerHorizM <= 0.1)
                // This is a 10cm drop in altitude for each 1m step towards the 
                // Camera is almost horizontal and this method wont work well.
                return;

            // Calculate the distance from the drone to the flatLandLocationM
            var droneBlockLocnM = Block.DroneLocnM;
            DroneLocation deltaLocnM = flatLandLocationM.Subtract(droneBlockLocnM);
            var deltaM = deltaLocnM.DiagonalM;
            var horizUnitVector = deltaLocnM.UnitVector();

            // Calculate horizontal step distance
            var vertEpsilonM = 0.20f; // 20cm
            var horizStepM = vertEpsilonM / vertStepDownPerHorizM;

            // Step from the drone towards the flatLandLocationM and beyond
            for (float testM = deltaM * 0.2f; testM < deltaM * 1.4f; testM += horizStepM)
            {
                var testLocnM = droneBlockLocnM.Add(horizUnitVector.Multiply(testM));
                var testDsmM = groundModel.GetElevationByDroneLocn(testLocnM);
                if (testDsmM == UnknownValue)
                    continue;

                var testAltM = Block.AltitudeM - testM * vertStepDownPerHorizM;
                if (testAltM < testDsmM + vertEpsilonM)
                {
                    // Drone line of sight has intersected the surface layer
                    CFM.LocationM = testLocnM;
                    if (Model.GroundData.HasDemModel)
                    {
                        var testDemM = Model.GroundData.DemModel.GetElevationByDroneLocn(testLocnM);
                        if (testDemM != UnknownValue)
                        {
                            CFM.LocationM = testLocnM;
                            CFM.HeightM = testAltM - testDemM;

                            Assert(testAltM <= Block.AltitudeM, "CalculateSettings_LocationM_and_HeightM: Bad HeightM 1");
                            Assert(testDemM <= Block.AltitudeM, "CalculateSettings_LocationM_and_HeightM: Bad HeightM 2");
                            Assert(CFM.HeightM < Block.AltitudeM - testDemM, "CalculateSettings_LocationM_and_HeightM: Bad HeightM 3");
                            Assert(CFM.HeightM >= -vertEpsilonM, "CalculateSettings_LocationM_and_HeightM: Bad HeightM 4");
                        }
                    }

                    break;
                }
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
                    CombFeature firstRealFeature,
                    double seenForMinDurations,
                    float demM,
                    float averageFlightStepFixedAltitudeM)
        {
            if ((firstRealFeature == null) ||
                (firstRealFeature.FeatureId == this.FeatureId) || // Need multiple real distinct features
                (this.Block == null) || // Last real feature
                (this.Block.FlightStep == null))
                return;

            // If drone is too low this method will not work.
            var lastStep = this.Block.FlightStep;
            float droneDistanceDownM = lastStep.FixedDistanceDown;
            if (droneDistanceDownM < 5)
                return; // Maintain current object height

            // Drone moved from point A to point B (base-line distance L) in metres.
            double baselineM = RelativeLocation.DistanceM(firstRealFeature.Block.DroneLocnM, this.Block.DroneLocnM);

            // The object may be 10m to left and 40m in front of the drone location
            // Calculate the height of the drone above the OBJECT's location DemM.
            var groundDownM = averageFlightStepFixedAltitudeM - demM;

            // Calculation is based on where object appears in the thermal camera field of view (FOV).
            // If object does not appear to have changed image position then this method will not work.
            double firstPixelY = firstRealFeature.CFM.PixelBox.Y + ((double)firstRealFeature.CFM.PixelBox.Height) / 2.0;
            double lastPixelY = this.CFM.PixelBox.Y + ((double)this.CFM.PixelBox.Height) / 2.0;
            if (firstPixelY == lastPixelY)
                return; // Maintain current object height

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
            if (Math.Abs(fwdTanDiff) < 0.1) // 0.1 rads = ~6 degrees
                return; // Maintain current object height

            // Returns the average tan of the sideways-down-angles
            // of the object in the first frame detected and the last frame detected.
            // Drone may be stationary but rotating.
            // double avgSideRads = Math.Abs(Calculate_Image_AvgSidewaysRads());
            // double avgSideTan = Math.Tan(avgSideRads);
            // PQR TODO Integrate avgSideTan into calcs?? Or too small to matter?

            var trigDownM = baselineM / fwdTanDiff;

            var featureHeightM = (float)(groundDownM - trigDownM);

            if (featureHeightM >= 0)
                this.CFM.HeightM = featureHeightM;
        }


        // Get the class's settings as datapairs for use in ObjectForm
        public DataPairList GetSettings_ObjectForm(RelativeLocation objectLocation)
        {
            return new DataPairList
            {
                { "Feature", FeatureId },
                { "Attributes", Attributes },
                { "NorthingDiffCM", ( CFM.LocationM != null ? (CFM.LocationM.NorthingM - objectLocation.NorthingM ) * 100 : 0), 0},
                { "EastingDiffCM", ( CFM.LocationM != null ? (CFM.LocationM.EastingM - objectLocation.EastingM) * 100 : 0), 0},
                { "HeightCM", CFM.HeightM * 100, 0 },
                { "WidthPixels", CFM.PixelBox.Width },
                { "DepthPixels", CFM.PixelBox.Height },
                { "#HotPixels", NumHotPixels() },
            };
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        override public DataPairList GetSettings()
        {
            var settings = base.GetSettings();

            CFM.GetSettings(settings);

            settings.Add("Num Hot Pixels", NumHotPixels());
            settings.Add("Density Perc", DensityPerc()); // 0 to 100
            settings.Add("Density Good", PixelDensityGood());
            settings.Add("Leg", (Block != null ? Block.FlightLegId : 0));

            return settings;
        }


        // Load this feature's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        override public void LoadSettings(List<string> settings)
        {
            base.LoadSettings(settings);

            CFM.LoadSettings(settings);
        }
    }


    // A list of Comb features bound to a specific Block
    public class CombFeatureList : SortedList<int, CombFeature>
    {
        public void AddFeature(CombFeature feature)
        {
            BaseConstants.Assert(feature != null, "AddFeature: Feature not specified.");
            BaseConstants.Assert(feature.FeatureId > 0, "AddFeature: No Id");

            this.Add(feature.FeatureId, feature);
        }


        // Add the feature list for a new block into this CombFeatureListList 
        public void AddFeatureList(CombFeatureList featuresToAdd)
        {
            if (featuresToAdd != null)
                foreach (var feature in featuresToAdd)
                    AddFeature(feature.Value);
        }


        public CombFeatureList Clone()
        {
            var answer = new CombFeatureList();

            foreach (var feature in this)
                answer.AddFeature(feature.Value);

            return answer;
        }


        // Analyse input image using Comb specific approach:
        // - PreReq: Convert image to gray scale, smooth and threshold => imgThreshold
        // - Create an Array[y,x] of bools set to 0 called inputSearched
        // - For each location where inputSearched[y,x] = false
        //      - Set inputSearched[y,x] = true
        //      - If imgThreshold[y,x] is a hot pixel
        //          - Add ([y,x],heat) to hotPixels list
        //          - PixelNeighborSearch, minimum 3 x 3 search, adding hot pixels to hotPixels, until all edges finds zero hot pixels.
        //          - Add search area rectangle, hot pixel count, max heat, min heat, and hotPixels list into a results list.  
        //  - Repeat for next location where inputSearched[y,x] = 0                   
        public void CreateFeaturesFromImage(
            CombProcessAll model,
            ProcessBlock block,
            Image<Bgr, byte> imgOriginal,
            Image<Gray, byte> imgThreshold)
        {
            int y = 0;
            int x = 0;

            try
            {
                // Create an Array[y, x] of bools. Booleans default to false
                int imageHeight = model.VideoData.ImageHeight;
                int imageWidth = model.VideoData.ImageWidth;
                bool[] inputSearched = new bool[imageHeight * imageWidth];

                // For each location where inputSearched[y,x] = false
                for (y = 0; y < imageHeight; y++)
                    for (x = 0; x < imageWidth; x++)
                    {
                        var index = y * imageWidth + x;
                        if (! inputSearched[index])
                        {
                            // Set inputSearched[y,x] = true
                            inputSearched[index] = true;

                            // If imgThreshold[y,x] is a hot pixel
                            var currPixelIsHot = (imgThreshold.Data[y, x, 0] != 0);
                            if (currPixelIsHot)
                            {
                                var feature = new CombFeature(model, block, CombFeatureTypeEnum.Real);

                                feature.PixelNeighborSearch(
                                    ref inputSearched,
                                    imgOriginal, imgThreshold,
                                    y, x);

                                AddFeature(feature);
                            }
                        }
                    }
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("CombFeatureList.Process: " +
                    "y=" + y + " x=" + x,
                    ex);
            }
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


        public (int minHeat, int maxHeat, int maxPixels) HeatSummary()
        {
            int maxHeat = 0;
            int minHeat = 255;
            int maxPixels = 0;
            foreach (var feature in this)
            {
                maxHeat = Math.Max(maxHeat, feature.Value.CFM.MaxHeat);
                if (feature.Value.CFM.MinHeat > 0)
                    minHeat = Math.Min(minHeat, feature.Value.CFM.MinHeat);
                maxPixels = Math.Max(maxPixels, feature.Value.NumHotPixels());
            }

            return (minHeat, maxHeat, maxPixels);
        }


        // Calculate object's location (centroid) as average of real feature's locations, using real features.
        // Also calculate the average error in location relative to the centroid.
        public (DroneLocation, float) Calculate_Avg_LocationM_and_LocationErrM()
        {
            if (Count >= 2)
            {
                int sumCount = 0;
                DroneLocation sumLocation = new();

                foreach (var feature in this)
                    if ((feature.Value.CFM.LocationM != null) &&
                        (feature.Value.CFM.Type == CombFeatureTypeEnum.Real))
                    {
                        sumCount++;
                        sumLocation.NorthingM += feature.Value.CFM.LocationM.NorthingM;
                        sumLocation.EastingM += feature.Value.CFM.LocationM.EastingM;
                    }

                if (sumCount > 0)
                {
                    var theLocationM = sumLocation.Multiply(1.0f / sumCount);

                    double sumDist = 0;
                    foreach (var feature in this)
                        if ((feature.Value.CFM.LocationM != null) &&
                            (feature.Value.CFM.Type == CombFeatureTypeEnum.Real))
                            sumDist += RelativeLocation.DistanceM(feature.Value.CFM.LocationM, theLocationM);
                    var theLocationErrM = (float)(sumDist / sumCount);

                    return (theLocationM.Clone(), theLocationErrM);
                }
            }
            else if (Count == 1)
            {
                var firstFeat = this.Values[0];
                if (firstFeat.CFM.LocationM != null)
                    return (firstFeat.CFM.LocationM.Clone(), 0);
            }

            return (new DroneLocation(), 0);
        }


        // Calculate object height and object height error, using real features.
        public (float heightM, float heightErrM, float minHeight, float maxHeight) Calculate_Avg_HeightM_and_HeightErrM()
        {
            int theCount = 0;
            float sumHeight = 0;
            float minHeight = 9999;
            float maxHeight = -9999;
            foreach (var feature in this)
                if ((feature.Value.CFM.LocationM != null) &&
                    (feature.Value.CFM.Type == CombFeatureTypeEnum.Real))
                {
                    var featureHeight = feature.Value.CFM.HeightM;
                    if (featureHeight != BaseConstants.UnknownValue)
                    {
                        theCount++;
                        sumHeight += featureHeight;
                        minHeight = Math.Min(featureHeight, minHeight);
                        maxHeight = Math.Max(featureHeight, maxHeight);
                    }
                }

            if (theCount > 0)
            {
                var heightM = (float)(sumHeight / theCount);
                return (
                    heightM,
                    Math.Max(
                        Math.Abs(maxHeight - heightM),
                        Math.Abs(minHeight - heightM)),
                    minHeight, maxHeight);
            }

            return (BaseConstants.UnknownValue, BaseConstants.UnknownValue, BaseConstants.UnknownValue, BaseConstants.UnknownValue);
        }


        // Return the average altitude of the drone over the object features.
        public float AverageFlightStepFixedAltitudeM()
        {
            float answer = 0;
            int count = 0;

            foreach (var feature in this)
            {
                if (feature.Value.CFM.Type == CombFeatureTypeEnum.Real)
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
    };
}
