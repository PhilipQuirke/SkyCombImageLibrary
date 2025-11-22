// Copyright SkyComb Limited 2025. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    public class BoundingBoxAnalyzer
    {
        private Point _topLeft;
        private Point _topRight;
        private Point _bottomLeft;
        private Point _bottomRight;

        public BoundingBoxAnalyzer(int minX, int minY, int maxX, int maxY)
        {
            _bottomLeft = new Point(minX, minY);
            _bottomRight = new Point(maxX, minY);
            _topLeft = new Point(minX, maxY);
            _topRight = new Point(maxX, maxY);
        }

        // The diagonal of a pixel box bounding an animal is a good proxy for the animal's spine length (in pixels)
        public double CalcSpineLength()
        {
            int width = _bottomRight.X - _bottomLeft.X;
            int height = _topLeft.Y - _bottomLeft.Y;
            return Math.Sqrt(width * width + height * height);
        }

        // The greatest perpendicular distance from the diagonal of a pixel box bounding an animal is a good proxy for the animal's girth (in pixels)
        public double CalcMaxPerpendicularDistance(Point start, Point end, PixelHeatList points)
        {
            // Calculate line equation coefficients (ax + by + c = 0)
            double a = end.Y - start.Y;
            double b = start.X - end.X;
            double c = end.X * start.Y - start.X * end.Y;
            double denominator = Math.Sqrt(a * a + b * b);

            double maxPointDistance = 0;
            foreach (var point in points)
            {
                double distance = Math.Abs(a * point.X + b * point.Y + c) / denominator;
                maxPointDistance = Math.Max(maxPointDistance, distance);
            }
            return maxPointDistance;
        }

        // Animal could be orientated NW to SE or NE to SW.
        public double CalcGirthLength(PixelHeatList points)
        {
            double girth1 = CalcMaxPerpendicularDistance(_bottomLeft, _topRight, points);
            double girth2 = CalcMaxPerpendicularDistance(_bottomRight, _topLeft, points);
            return Math.Min(girth1, girth2);
        }
    }


    // A logical object derived from overlapping features over successive frames. 
    public class ProcessObject : ProcessObjectModel
    {
        // Parent process model
        protected ProcessAll ProcessAll { get; }

        public ProcessConfigModel? ProcessConfig { get { return ProcessAll?.ProcessConfig; } }
        public ProcessScope? ProcessScope { get; set; } = null;

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

        // During processing, save an image of the object in the last real feature 
        public Image<Bgr, byte>? LastImage { get; set; } = null;


        public ProcessObject(ProcessAll processAll, ProcessScope? processScope) : base()
        {
            ProcessAll = processAll;
            ProcessScope = processScope;
            ProcessFeatures = new(ProcessAll.ProcessConfig);

            ObjectId = ++NextObjectId;
            if (ProcessScope != null)
            {
                FlightLegId = ProcessScope.PSM.CurrRunLegId;
                RunFromVideoS = (float)(ProcessScope.PSM.CurrInputFrameMs / 1000.0);
                RunToVideoS = RunFromVideoS;
            }
        }


        // For Yolo, each object can have at most one feature per block.
        private bool StopYoloSecondBlockClaim(ProcessFeature theFeature)
        {
            return (ProcessAll is YoloProcess) &&
                (theFeature.Type == FeatureTypeEnum.Real) &&
                (LastRealFeature != null) &&
                (LastRealFeature.Block.BlockId == theFeature.Block.BlockId);
        }


        // Set the last image of this object.
        private void SetLastImage(Image<Bgr, byte>? lastImage = null)
        {
            LastImage?.Dispose();
            LastImage = lastImage;
        }


        // Object may claim ownership of this feature extending the object's lifetime and improving its "Significant" score.
        // This code implements a feature claim. Other functions mostly decide whether the feature should be claimed.
        public virtual bool ClaimFeature(ProcessFeature theFeature)
        {
            try
            {
                Assert(theFeature.ObjectId <= 0, "ClaimFeature: Feature is already owned.");
                if (LastRealFeature == null)
                    Assert(theFeature.Type == FeatureTypeEnum.Real, "ClaimFeature: Initial feature must be Real");

                // For Yolo, each object can have at most one feature per block.
                if (StopYoloSecondBlockClaim(theFeature))
                    return false;

                if ((FlightLegId == 0) && (LastRealFeature != null) && (theFeature.Block.FlightLegId > 0))
                    // First object feature(s) were not in a leg, but this feature is in a leg. Update the object leg id.   
                    FlightLegId = theFeature.Block.FlightLegId;

                // Associate the feature with this object.
                theFeature.ObjectId = this.ObjectId;

                bool wasSignificant = Significant;
                bool increasedMaxSumRealHotPixels = false;

                // Is feature real?
                if (theFeature.Type == FeatureTypeEnum.Real)
                {
                    theFeature.IsTracked = true;
                    increasedMaxSumRealHotPixels = theFeature.SumHotPixels > MaxSumRealHotPixels;
                    MaxNumRealHotPixels = Math.Max(MaxNumRealHotPixels, theFeature.NumHotPixels);
                    MaxSumRealHotPixels = Math.Max(MaxSumRealHotPixels, theFeature.SumHotPixels);
                    MaxNumMaxHeatPixels = Math.Max(MaxNumMaxHeatPixels, theFeature.NumMaxHeatPixels);

                    var theBlock = theFeature.Block;

                    if ((LastRealFeature == null) || (LastRealFeature.Block.BlockId < theBlock.BlockId))
                    {
                        // First real feature claimed by this object for THIS block
                        ProcessFeatures.AddFeature(theFeature);
                        LastRealFeatureId = theFeature.FeatureId;
                        RunToVideoS = (float)(theFeature.Block.InputFrameMs / 1000.0);
                        MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, theFeature.PixelBox.Width);
                        MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, theFeature.PixelBox.Height);
                    }
                    else // if( RunConfig.RunProcess == RunProcessEnum.Comb) 
                    {
                        // Comb Process: This object is claiming a second or third feature for this block.
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

                // Save the image of the object with the most hot pixels.
                if ((theFeature.Type == FeatureTypeEnum.Real) &&
                    (ProcessScope?.CurrInputImage != null) &&
                    ((LastImage == null) || increasedMaxSumRealHotPixels))
                {
                    DrawImageConfig drawImageConfig = new DrawImageConfig();
                    drawImageConfig.BoxExtraScale = 2;
                    drawImageConfig.TextExtraScale = 1;

                    var closeupInputImage =
                        DrawFrameImage.Draw(
                            RunProcessEnum.Yolo, // Force drawing of hot pixels.
                            ProcessAll.ProcessConfig,
                            drawImageConfig,
                            ProcessAll.Drone,
                            ProcessScope.CurrInputImage.Convert<Bgr,byte>(), // May need to scale by 2.
                            this,
                            theFeature.Block, ProcessAll,
                            false);

                    var showInputBox = Transform.GetInflatedSquareBox(theFeature.PixelBox, 50);
                    var safeShowInputBox = DrawFrameImage.MoveVisibleBoxInsideImageSize(showInputBox, ProcessScope.CurrInputImage.Size);
                    closeupInputImage = closeupInputImage.Copy(safeShowInputBox);
                    SetLastImage(closeupInputImage);
                }

                return true;
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessObject.ClaimFeature", ex);
            }
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        public bool MaybeClaimFeature(ProcessFeature the_feature, Rectangle objectExpectedPixelBox)
        {
            if (StopYoloSecondBlockClaim(the_feature))
                return false;

            if (the_feature.ObjectId == 0) // Not claimed yet
                if (the_feature.Significant || this.Significant)
                    if (the_feature.SignificantPixelBoxIntersection(objectExpectedPixelBox))
                        // Object will claim feature if the object remains viable after claiming feature
                        return ClaimFeature(the_feature);

            return false;
        }


        public override void ResetCalcedMemberData()
        {
            base.ResetCalcedMemberData();
            ProcessFeatures = new(ProcessConfig);
        }


        // Save memory (if compatible with Config settings) by deleting pixel data
        public void ClearHotPixelArray()
        {
            foreach (var feature in ProcessFeatures)
                feature.Value.ClearHotPixelArray();
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
                if (ProcessAll.Drone.InputIsImages)
                    // For images we do not use unreal persistance features.
                    BeingTracked = false;

                else if (LastRealFeature.Block.BlockId >= BlockId)
                {
                    // Yes, this is worth tracking.
                }
                else
                    // Are we still on the persistance window?
                    BeingTracked = (LastFeature.Block.BlockId - LastRealFeature.Block.BlockId < ProcessConfigModel.ObjectMaxUnrealBlocks);
            }

            return BeingTracked;
        }


        // How many units of Config.ObjectMinDurationMs has this object been seen for?
        // If input is images we always return 1
        public double SeenForMinDurations()
        {
            if (ProcessAll.Drone.InputIsImages)
                return 1;

            var minDuration = ProcessConfigModel.ObjectMinDurationMs; // Say 500ms
            var timeSeenMs = (1000.0F * NumRealFeatures()) / ProcessAll.VideoData.Fps;
            return (timeSeenMs / minDuration);
        }


        // Is this object in the RunFrom/To scope?
        public bool InRunScope(ProcessScope scope)
        {
            var maxFromMs = Math.Max(RunFromVideoS * 1000, scope.PSM.FirstVideoFrameMs);
            var minToMs = Math.Min(RunToVideoS * 1000, scope.PSM.LastVideoFrameMs);

            var overlapMs = minToMs - maxFromMs;

            if (ProcessAll.Drone.InputIsImages)
                // When InputIsImages, object duration is zero.
                if (overlapMs == 0)
                    return true;

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

            DemM = firstStep.DemM;
        }


        // Given this object's last known pixel position, and the object's
        // movement across the image, where do we expect the object to be this block?
        public Rectangle ExpectedLocationThisBlock(int xScale = 1, int yScale = 2)
        {
            Rectangle answer;

            var firstFeat = ProcessFeatures.FirstFeature;
            var lastFeat = ProcessFeatures.LastFeature;

            var firstBox = firstFeat.PixelBox;
            var lastBox = lastFeat.PixelBox;

            if ((this.ObjectId == 31) && (lastFeat.FeatureId == 675))
                answer = answer = new Rectangle(
                    lastBox.X,
                    lastBox.Y,
                    lastBox.Width,
                    lastBox.Height);

            var numBlockSteps = lastFeat.BlockId - firstFeat.BlockId + 1;
            if (numBlockSteps >= 2)
            {
                // We have multiple features. Use their difference in location as velocity in pixels/frame.
                var distanceX = 1.0F * lastBox.X + lastBox.Width / 2.0F - firstBox.X - lastBox.Width / 2.0F;
                var distanceY = 1.0F * lastBox.Y + lastBox.Height / 2.0F - firstBox.Y - lastBox.Height / 2.0F;
                var numMoves = 1.0f * numBlockSteps - 1;
                var xPixelsPerBlock = distanceX / numMoves;
                var yPixelsPerBlock = distanceY / numMoves;

                // The object may, over a few frames, be progressively obscured by a branch, then reappear,
                // so we use the largest pixel box seen for the object over previous features.
                int maxHeight = MaxRealPixelHeight;
                int maxWidth = MaxRealPixelWidth;
                var halfHeightDiff = (maxHeight - lastBox.Height) / 2.0f;
                var halfWidthDiff = (maxWidth - lastBox.Width) / 2.0f;

                // Advance one average stride from the previous location.
                answer = new Rectangle(
                    (int)(lastBox.X + xPixelsPerBlock - halfWidthDiff),
                    (int)(lastBox.Y + yPixelsPerBlock - halfHeightDiff),
                    maxWidth * xScale,
                    maxHeight * yScale);
            }
            else
                // With one feature we dont know the object's velocity across the image.
                // Rely on image overlap
                answer = new Rectangle(
                    lastBox.X,
                    lastBox.Y,
                    lastBox.Width,
                    lastBox.Height);

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
        // 2) Animal on open ground, fully visible, very dense, stationary.
        // 3) Animal on open ground, fully visible, very dense, moving very slowly.
        public virtual void Calculate_Significant()
        {
            try
            {
                // PIXELS
                var maxNumHotPixels = MaxNumRealHotPixels;// Maximum pixel count per real feature
                var pixelBasics = 
                    (MaxNumMaxHeatPixels >= ProcessConfig.ObjectMinMaxHeatPixels) &&
                    ((ProcessConfig.ObjectMaxPixels <= 0) || (maxNumHotPixels <= ProcessConfig.ObjectMaxPixels));

                var pixelsOk = pixelBasics && (maxNumHotPixels > ProcessConfig.ObjectMinPixels); // Say 5 pixels
                var pixelsGood = pixelBasics && (maxNumHotPixels > 2 * ProcessConfig.ObjectMinPixels); // Say 10 pixels 
                var pixelsGreat = pixelBasics && (maxNumHotPixels > 4 * ProcessConfig.ObjectMinPixels); // Say 20 pixels

                // DENSITY
                var densityOK =  (RealDensityPx() >= ProcessConfigModel.ObjectMinHotDensity);

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

                Significant =
                   pixelsOk &&
                   densityOK &&
                   (ProcessAll.Drone.InputIsImages || 
                    // Key video calculation for identifying significant objects
                    (timeOk && ( elevationGood || pixelsGood )));

                if (Significant)
                    NumSigBlocks++;

                // Summarise why object is significant or not. Gets displayed in UI and saved to xls.
                Attributes = String.Format("{0} {1} {2} {3}",
                    Significant ? "Sig" : "",
                    pixelsGreat ? "P3" : (pixelsGood ? "P2" : (pixelsOk ? "P1" : "p")),
                    timeGreat ? "T3" : (timeGood ? "T2" : (timeOk ? "T1" : "t")),
                    elevationGreat ? "E3" : (elevationGood ? "E2" : (elevationOK ? "E1" : "e"))).Trim();
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessObject.Calculate_Significant", ex);
            }
        }


        // An Object can't be significant until Config.ObjectMinDurationMs has passed.
        // A Feature can be significant immediately.
        // This "VaguelySignificant" object code mirrors the feature "Significant" code
        public bool VaguelySignificant()
        {
            if ((ProcessConfig.ObjectMinPixels > 0) && (MaxNumRealHotPixels < ProcessConfig.ObjectMinPixels)) // Say 5 pixels / Block
                return false;

            if ((ProcessConfig.ObjectMaxPixels > 0) && (MaxNumRealHotPixels > ProcessConfig.ObjectMaxPixels)) // Say 1000 pixels / Block
                return false;

            return true;
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


        // Calculate object height and object height error by averaging the feature data.
        protected void Calculate_HeightM_and_HeightErrM()
        {
            (HeightM, HeightErrM, MinHeightM, MaxHeightM) = ProcessFeatures.Calculate_Avg_HeightM_and_HeightErrM();
        }


        // Calculate the average horizontal range of the object from the drone in meters.
        // Relies on object.LocationM already being calculated.
        public void Calculate_AvgRangeM()
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


        // Calculate the size of the object in square centimeters.
        //
        // Objects may be partially obscured by branches in some frames.
        // The object may be moving across the ground or a long a branch during the frames.
        //
        // The object's (maximum over frames) "spine" length, is a proxy for size. 
        // The object's (maximum over frames) "girth" length (at right angles to the spine length), is a proxy for size. 
        // If unobscured, these are very good proxies, else they are lower bounds.
        public void Calculate_SizeCM2()
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

            var area = lastFeature.PixelBox;

            BoundingBoxAnalyzer analyzer = new(area.X, area.Y, area.X + area.Width, area.Y + area.Height);
            double thisSpinePixels = analyzer.CalcSpineLength();
            double thisGirthPixels = lastFeature.Pixels != null ? analyzer.CalcGirthLength(lastFeature.Pixels) : thisSpinePixels / 4.0f;
            Assert(thisSpinePixels >= 0, "Calculate_SizeCM2: spinePixels <= 0");
            Assert(thisGirthPixels >= 0, "Calculate_SizeCM2: girthPixels <= 0");
            Assert(thisSpinePixels + 1 >= thisGirthPixels, "Calculate_SizeCM2: spinePixels < girthPixels");

            MaxSpinePixels = Math.Max(MaxSpinePixels, (float)thisSpinePixels);
            MaxGirthPixels = Math.Max(MaxGirthPixels, (float)thisGirthPixels);

            // The number of hot pixels in the last (real) feature.
            float hotPixels = lastFeature.NumHotPixels; // PQR TODO should use most heated feature

            // Grab the drone input image area
            float imageAreaM2 = lastFeature.Block.FlightStep.InputImageSizeM.AreaM2();
            // Calculate the number of pixels in the video image
            float framePixels = ProcessAll.VideoData.ImageWidth * ProcessAll.VideoData.ImageHeight;

            // Calculate the size of the object in this frame in square centimeters
            float thisSizeM2 = imageAreaM2 * hotPixels / framePixels;
            var thisSizeCM2 = (int)(thisSizeM2 * 100 * 100);

            SizeCM2 = Math.Max(SizeCM2, thisSizeCM2);

            float cmPerPixel = MathF.Sqrt(imageAreaM2 * 10000 / framePixels);
            SpineCM = Math.Max(1, (int)(thisSpinePixels * cmPerPixel));
            GirthCM = Math.Max(1, (int)(thisGirthPixels * cmPerPixel));
        }


        // Calculate the maximum heat value of any pixel in this object in any frame 
        public void Calculate_MaxHeat()
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

            return (1.0f * MaxNumRealHotPixels) / pixelArea;
        }


        // Calculate the simple (int, float, VelocityF, etc) member-data of this real object.
        // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
        public void Calculate_RealObject_SimpleMemberData()
        {
            try
            {
                // Calculate the drone SumLinealM distance corresponding to the centroid of the object
                Calculate_AvgSumLinealM();

                // First estimate of OBJECT location (centroid) as average over all real features.
                // The feature locations are based on where in the drone's field of image the object was detected,
                // and takes into account ground undulations. The object's location is further refined later.
                Calculate_LocationM_and_LocationErrM();

                // Estimate the OBJECT's DEM at the object's location
                // (which could be say 20m left of the drone's flight path).
                Calculate_DemM();

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


        // For each real feature in the object, store last-calculated location and height
        // If feature location and height are not specified, assign them last-calculated location and height
        public void Calculate_RealObjectFeatures_MissingLocationsAndHeights()
        {
            DroneLocation prevFeatureLocationM = null;
            float prevFeatureHeightM = UnknownHeight;

            foreach ((var _, var feature) in ProcessFeatures)
            {
                if (feature.Type == FeatureTypeEnum.Real)
                {
                    bool fix = false;

                    if (feature.LocationM != null)
                        prevFeatureLocationM = feature.LocationM;
                    else
                        fix = true;

                    if (feature.HeightM > UnknownHeight)
                        prevFeatureHeightM = feature.HeightM;
                    else
                        fix = true;

                    if (fix)
                        feature.Set_LocationM_HeightM(prevFeatureLocationM, prevFeatureHeightM);
                }
            }
        }



        public int AvgRealHotPixelHeat
        {
            get
            {
                // For each real feature, calculate the average heat per hot pixel
                // Then calculate the average of these averages.
                if (MaxNumRealHotPixels <= 0) return 0;

                int numFeatures = 0;
                double sumfeatureAvg = 0;

                foreach (var feature in ProcessFeatures)
                {
                    if (feature.Value.Type == FeatureTypeEnum.Real && feature.Value.NumHotPixels > 0)
                    {
                        double featureAvg = 1.0 * feature.Value.SumHotPixels / feature.Value.NumHotPixels;

                        numFeatures++;
                        sumfeatureAvg += featureAvg;
                    }
                }

                return (int)(numFeatures > 0 ? sumfeatureAvg / numFeatures : 0.0);
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
            MinHeightM = BaseConstants.UnknownHeight;
            MaxHeightM = BaseConstants.UnknownHeight;
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
        public void EnsureObjectsNamed(ProcessAll processAll)
        {
            foreach (var theObj in this)
                if (theObj.Value.Name == "")
                {
                    processAll.NumInsignificantObjects++;

                    theObj.Value.SetName("#", processAll.NumInsignificantObjects);
                }
        }


        public int NumSignificantObjects
        {
            get
            {
                int answer = 0;

                foreach (var theObj in this)
                    if (theObj.Value.Significant)
                        answer++;

                return answer;
            }
        }


        // Number of objects that have ever been significant. 
        // Not same as num objs significant in the current Block, as objects can become insignificant. 
        public int NumEverSignificantObjects
        {
            get
            {
                int numSig = 0;
                foreach (var theObject in this)
                    if ((theObject.Value.NumSigBlocks > 0) && (theObject.Value.SeenForMinDurations() >= 1))
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

        public ProcessObjList FilterByObjectScope(RunConfig runConfig, ProcessObjList sigObjects)
        {
            if (runConfig == null)
                return sigObjects;

            ProcessObjList answer = new();

            foreach (var theObject in sigObjects)
            {
                // Only return objects in the object size selection
                if (runConfig.InRange(theObject.Value))
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

        // Returns key attributes of objects and associated user annotations (if any) to show in the ObjectGrid
        public List<object[]> GetObjectGridData(ProcessScope scope, RunConfig runConfig, ObjectCategoryList annotations)
        {
            var answer = new List<object[]>();

            var sigObjects = FilterByProcessScope(scope);
            var selectedOjects = FilterByObjectScope(runConfig, sigObjects);


            foreach (var theObject in selectedOjects)
            {
                ObjectCategoryModel? annotation = null;
                if (annotations != null)
                    annotation = annotations.GetData(theObject.Value.Name);

                answer.Add(theObject.Value.GetObjectGridData(annotation));
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
        public virtual void LoadSettings(List<string> settings)
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
