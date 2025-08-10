// Copyright SkyComb Limited 2025. All rights reserved. 
using Emgu.CV;
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


        // A feature is associated 1-1 with a Block 
        public ProcessBlock Block { get; set; }

        // Location of hot pixels in this feature.
        public PixelHeatList? Pixels { get; set; } = null;


        public ProcessFeature(ProcessAll processAll, int blockId, FeatureTypeEnum type) : base(blockId, type)
        {
            ResetCalcedMemberData();

            ProcessAll = processAll;
            Block = processAll.Blocks[blockId];
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


        public void ClearHotPixelArray()
        {
            Pixels = null;
        }

        public void ClearHotPixelData()
        {
            NumHotPixels = 0;
            SumHotPixels = 0;
            MinHeat = UnknownValue;
            MaxHeat = UnknownValue;
        }

        // Regenerate pixel data from the original and threshold images when needed
        // This is used when pixel data was cleared for memory management but is now needed for display
        public virtual void RegeneratePixelData(in Image<Bgr, byte> imgOriginal, in Image<Gray, byte> imgThreshold)
        {
            if (Pixels != null)
                return; // Already have pixel data

            // For base ProcessFeature, use the common threshold-based approach
            ImageProcessingUtils.RegeneratePixelsInBoundingBox(this, imgOriginal, imgThreshold, ProcessAll.ProcessConfig);
        }

        public void AddHotPixel(int currY, int currX, Bgr currColor)
        {
            int currHeat = (int)((currColor.Blue + currColor.Green + currColor.Red) / 3);

            Pixels.Add(new PixelHeat(BlockId, FeatureId, currY, currX, currHeat));
        }

        public void Calculate_HotPixelData()
        {
            if ((Pixels != null) && (Pixels.Count() > 0))
            {
                ClearHotPixelData();

                NumHotPixels = Pixels.Count();
                SumHotPixels = 0;
                MinHeat = Pixels[0].Heat;
                MaxHeat = MinHeat;

                var threshold = ProcessAll.ProcessConfig.HeatThresholdValue;
                foreach (var pixel in Pixels)
                {
                    SumHotPixels += Math.Max(0, pixel.Heat - threshold);
                    MinHeat = Math.Min(MinHeat, pixel.Heat);
                    MaxHeat = Math.Max(MaxHeat, pixel.Heat);
                }
            }
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
                resultA >= minOverlap ||   // 5% overlap of rectA
                resultB >= minOverlap;     // 5% overlap of rectB
        }


        // This feature consumes/absorbs/takes-hot-pixels-from the otherFeature, leaving otherFeature empty.
        public virtual void Consume(ProcessFeature otherFeature)
        {
            Assert(otherFeature != null, "Consume: otherFeature not specified.");
            Assert(ProcessAll is not YoloProcess, "Consume: YoloProcess not supported.");

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
        public (double xFraction01, double yFraction01) CentroidImageFractions()
        {
            double xCenterPixels = Math.Min(PixelBox.X + PixelBox.Width / 2.0, ProcessAll.VideoData.ImageWidth);
            double yCenterPixels = Math.Min(PixelBox.Y + PixelBox.Height / 2.0, ProcessAll.VideoData.ImageHeight);

            // Calculate position of center of feature as fraction of drone image area.
            double xFraction01 = xCenterPixels / ProcessAll.VideoData.ImageWidth;

            // With image pixels, y = 0 is the top of the image, but we want y = 0 to be the bottom.
            // Transform y-coordinate so that y = 0 is the bottom of the image and y = 1 is the top.
            double yFraction01 = (ProcessAll.VideoData.ImageHeight - yCenterPixels) / ProcessAll.VideoData.ImageHeight;

            return (xFraction01, yFraction01);
        }


        // Calculate object location and height considering land contour undulations.
        // "Walk" sight-line from drone to object in 3D space. Stop when line intersects DEM.
        // This code depends on ProcessAll.VideoData.HFOVDeg, FlightStep.FixAltM, FixYawDeg and FixPitchDeg (via FixedCameraToVerticalForwardDeg).
        // These "fixed" values are used to link into ProcessSpan optimisation algorithm.
        // This code does NOT depend on FlightStep.InputImageCenter/InputImageSize/InputImageUnitVector/InputImageDemM/InputImageDsmM
        public void CalculateSettings_LocationM_HeightM_LOS(GroundData groundData)
        {
            int phase = 0;
            try
            {
                if ((ProcessAll == null) || (Block == null) || (Block.FlightStep == null) || (Block.DroneLocnM == null) || (groundData == null))
                    return;

                phase = 1;
                var flightStep = Block.FlightStep;

                // Use ground (not surface) model for this calculation.
                phase = 2;
                var groundModel = groundData.HasDemModel ? groundData.DemModel : groundData.DsmModel;
                if (groundModel == null)
                    return;

                phase = 3;
                TerrainGrid terrainGrid = new(groundModel);

                DroneState droneState = new();
                droneState.LocationNE = Block.DroneLocnM;
                droneState.Altitude = flightStep.FixedAltitudeM; // Relies on FixAltM
                droneState.Yaw = flightStep.YawDeg;
                droneState.CameraDownAngle = 90 - flightStep.CameraToVerticalForwardDeg;

                // LOS algorithm works very inaccurately if the camera is pointing near the horizon.
                phase = 4;
                if (droneState.CameraDownAngle < 15)
                    return;

                // LOS algorithm works is not useful if the drone is less than 10m above the ground.
                // This also catches the very rare case where the DEM data is bad (very large) and so FixedDistanceDown goes negative.
                if (flightStep.FixedDistanceDown < 10)
                    return;

                CameraParameters cameraParams = new();
                cameraParams.FocalLength = ProcessAll.VideoData.FocalLength;
                cameraParams.ImageWidth = ProcessAll.VideoData.ImageWidth;
                cameraParams.ImageHeight = ProcessAll.VideoData.ImageHeight;
                cameraParams.SensorWidth = ProcessAll.VideoData.SensorWidth;
                cameraParams.SensorHeight = ProcessAll.VideoData.SensorHeight;
                cameraParams.HorizontalFOV = ProcessAll.VideoData.HFOVDeg;
                cameraParams.VerticalFOV = ProcessAll.VideoData.VFOVDeg;

                (double xFraction01, double yFraction01) = CentroidImageFractions(); // Range 0 to 1
                ImagePosition imagePosition = new();
                imagePosition.PixelX = (PixelBox.X + PixelBox.Width / 2.0);
                imagePosition.PixelY = (PixelBox.Y + PixelBox.Height / 2.0);


                phase = 5;
                DroneTargetCalculator droneTargetCalculator = new(droneState, cameraParams, terrainGrid, false);
#if DEBUG
                // Do a sanity test once per run
                if ((flightStep.FixAltM == 0) && (Block.BlockId == 13))
                    droneTargetCalculator.UnitTest_Centroid(Block);
#endif
                LocationResult? result = droneTargetCalculator.CalculateTargetLocation(imagePosition);
                if (result != null)
                {
                    phase = 6;
                    HeightAlgorithm = LineOfSightHeightAlgorithm;
                    var heightM = (groundData.HasDemModel ? result.Elevation - groundData.DemModel.GetElevationByDroneLocn(result.LocationNE) : 0);
                    Set_LocationM_HeightM(result.LocationNE, heightM);
                }
                else
                    HeightAlgorithm = "NoResult";
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessFeature.CalculateSettings_LocationM_HeightM_LOS. Phase=" + phase.ToString(), ex);
            }
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)\
        // Add derived data that is saved but not reloaded.
        public override DataPairList GetSettings()
        {
            var settings = base.GetSettings();

            settings.Add("Leg", (Block != null ? Block.FlightLegId : 0));
            // Horizontal distance from feature to drone.
            settings.Add("RangeM", (Block != null ? RelativeLocation.DistanceM(LocationM, Block.DroneLocnM) : 0), LocationNdp);

            return settings;
        }


        // Get the class's settings as datapairs for use in ObjectForm
        public DataPairList GetSettings_ObjectForm(RelativeLocation objectLocation, double objectHeight)
        {
            var centroidx = PixelBox.X + PixelBox.Width / 2;
            var centroidy = PixelBox.Y + PixelBox.Height / 2;
            return new DataPairList
            {
                { "Feature", FeatureId },
                { "blockid", BlockId },
                { "WidthPixels", PixelBox.Width },
                { "DepthPixels", PixelBox.Height },
                { "#HotPixels", NumHotPixels },
                { "centroid", "(" + centroidx + ", " + centroidy + ")"},
                { "location", ( LocationM != null ? "(" + LocationM.EastingM + ", " + LocationM.NorthingM + ", " + HeightM + ")":"")},
                { "NorthingDiffM", ( LocationM != null ? (LocationM.NorthingM - objectLocation.NorthingM ) : 0.0), 1},
                { "EastingDiffM", ( LocationM != null ? (LocationM.EastingM - objectLocation.EastingM) : 0.0), 1},
                { "HeightDiff", ( LocationM != null ? (HeightM - objectHeight) : 0), 1},
                { "DroneLocation", "(" + Block.DroneLocnM.EastingM + ", " + Block.DroneLocnM.NorthingM + ", " + Block.AltitudeM + ")" },
            };
        }
    }


    // A list of Comb or Yolo features 
    public class ProcessFeatureList : SortedList<int, ProcessFeature>
    {
        private static ProcessConfigModel ProcessConfig;

        public ProcessFeature? FirstFeature { get { return Count == 0 ? null : Values[0]; } }
        // Last (Real or UnReal) feature claimed by this object. May be null.
        public ProcessFeature? LastFeature { get { return Count == 0 ? null : Values[^1]; } }


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
                    if ((feature.Value.LocationM != null) && (feature.Value.HeightM != 0) &&
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
        public (float heightM, float heightErrM, float minHeight, float maxHeight) Calculate_Avg_HeightM_and_HeightErrM()
        {
            int countLOS = 0;
            float sumLOSHeight = 0;
            float minHeight = 9999;
            float maxHeight = BaseConstants.UnknownValue;
            foreach (var feature in this)
                if ((feature.Value.LocationM != null) &&
                    (feature.Value.Type == FeatureTypeEnum.Real))
                {
                    var featureHeight = feature.Value.HeightM;
                    if (featureHeight > BaseConstants.UnknownHeight)
                    {
                        minHeight = Math.Min(featureHeight, minHeight);
                        maxHeight = Math.Max(featureHeight, maxHeight);
                        if (feature.Value.HeightAlgorithm.StartsWith(ProcessFeature.LineOfSightHeightAlgorithm))
                        {
                            countLOS++;
                            sumLOSHeight += featureHeight;
                        }
                    }
                }

            var heightM = (countLOS > BaseConstants.UnknownHeight ? (float)(sumLOSHeight / countLOS) : BaseConstants.UnknownValue);
            if (heightM > BaseConstants.UnknownHeight)
                return (
                    heightM,
                    Math.Max(
                        Math.Abs(maxHeight - heightM),
                        Math.Abs(minHeight - heightM)),
                    minHeight, maxHeight);

            return (BaseConstants.UnknownValue, BaseConstants.UnknownValue, BaseConstants.UnknownValue, BaseConstants.UnknownValue);
        }


        public (int minHeat, int maxHeat, int maxPixels) HeatSummary()
        {
            int maxHeat = 0;
            int minHeat = 255;
            int maxHotPixels = 0;
            foreach (var feature in this)
            {
                var combFeature = feature.Value;
                maxHeat = Math.Max(maxHeat, combFeature.MaxHeat);
                if (feature.Value.MinHeat > 0)
                    minHeat = Math.Min(minHeat, combFeature.MinHeat);
                maxHotPixels = Math.Max(maxHotPixels, combFeature.NumHotPixels);
            }

            return (minHeat, maxHeat, maxHotPixels);
        }
    };
}
