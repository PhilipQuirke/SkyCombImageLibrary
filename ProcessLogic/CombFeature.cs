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


        // During calculation we store in memory some values to aid debugging.
        // They are NOT persisted to the Datastore, & so are NOT loaded from the Datastore.
        public double Debug1 { get; set; }
        public double Debug2 { get; set; }
        public double Debug3 { get; set; }
        public double Debug4 { get; set; }


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

            Debug1 = 0;
            Debug2 = 0;
            Debug3 = 0;
            Debug4 = 0;
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


        // Calculate the location (centroid) of this feature inside the drone imaging box
        // This is the key translation from IMAGE to PHYSICAL coordinate system. 
        public void CalculateSettings_LocationM(bool initialCalc = true)
        {
            if ((CFM.LocationM != null) || (Block.FlightStep == null) || (Block.FlightStep.InputImageCenter == null))
                return;

            // Debugging - Set breakpoint on assignment. Assignment value is overridden later in this proc.
            if ((Config.FocusObjectId != 0) && (ObjectId == Config.FocusObjectId))
                CFM.LocationM = new();

            // Calculate center (centroid) of feature in image pixels.
            (double xFraction, double yFraction) = CentroidImageFractions(initialCalc);

            // There are multiple Blocks per FlightLocation.
            // This is the smoothed block-level change in location
            // since the previous FlightStep, in the flight direction.
            DroneLocation deltaBlockLocnM = new(
                Block.DroneLocnM.NorthingM - Block.FlightStep.DroneLocnM.NorthingM,
                Block.DroneLocnM.EastingM - Block.FlightStep.DroneLocnM.EastingM);

            // Calculate physical location of this feature based on:
            // 1) the POSITION in the image of the feature (given by xFraction, yFraction, say 0.4, 0.1)
            // 2) the CENTER of the drone physical field of vision (given by FlightStep.InputImageCenter, say 240m Northing, 78m Easting )
            // 3) the SIZE of the drone physical field of vision (given by InputImageSizeM, say 18m by 9m)
            // 4) the DIRECTION of flight of the drone (given by YawDeg, say -73 degrees)
            // This is the key translation from IMAGE to PHYSICAL coordinate system. 
            CFM.LocationM =
                Block.FlightStep.CalcImageFeatureLocationM(deltaBlockLocnM, xFraction, yFraction, initialCalc)
                    ?.Clone();
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
            // Debugging - Set breakpoint on assignment. Assignment value is overridden later in this proc.
            if (otherFeature.ObjectId == Config.FocusObjectId)
                otherFeature.Attributes = "";


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

                // During calculation we store in memory some values to aid debugging. They are NOT persisted to the Datastore.
                { "Debug1", Debug1, 4 },
                { "Debug2", Debug2, 4 },
                { "Debug3", Debug3, 4 },
                { "Debug4", Debug4, 4 },
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
            settings.Add("Leg", (Block != null ? Block.LegId : 0));

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
            BaseConstants.Assert(feature.FeatureId > 0, "AddFeature: FeatureId not specified.");

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
        public void Process(
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
                        if (inputSearched[y * imageWidth + x] == false)
                        {
                            // Set inputSearched[y,x] = true
                            inputSearched[y * imageWidth + x] = true;

                            // If imgThreshold[y,x] is a hot pixel
                            var currPixelIsHot = (imgThreshold.Data[y, x, 0] != 0);
                            if (currPixelIsHot)
                            {
                                var feature = new CombFeature(model, block, CombFeatureTypeEnum.Real);

                                feature.PixelNeighborSearch(
                                    ref inputSearched,
                                    imgOriginal, imgThreshold,
                                    y, x);

                                // This is the key translation from IMAGE to PHYSICAL coordinate system. 
                                feature.CalculateSettings_LocationM();

                                AddFeature(feature);
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
    };
}
