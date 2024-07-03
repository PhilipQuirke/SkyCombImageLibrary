// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;



namespace SkyCombImage.ProcessLogic
{
    // A Comb feature, is a dense cluster of hot pixels, associated 1-1 with a Block
    public class CombFeature : ProcessFeature
    {
        // Parent process model
        protected CombProcess CombProcess { get { return ProcessAll as CombProcess; } }


        public CombFeature( CombProcess combProcess, ProcessBlock block, FeatureTypeEnum type) : base(combProcess, block.BlockId, type)
        {
            if (type != FeatureTypeEnum.Unreal)
                Pixels = new();
            ResetMemberData();
        }


        // Constructor used when loaded objects from the datastore
        public CombFeature(CombProcess combProcess, List<string> settings) : base(combProcess, settings)
        {
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
                MinHeat = 255 + 255 + 255;
                MaxHeat = 0;

                // Start with 3 pixels wide. This width will vary row by row
                int fromX = 0;
                int toX = 3;
                int rectTop = startY;
                int rectLeft = startX + fromX;
                int rectRight = startX + toX - 1;

                // Search down the image
                for (currY = startY; currY < CombProcess.VideoData.ImageHeight; currY++)
                {
                    int hotPixelsInRow = 0;

                    // CASE: EXPAND LEFT
                    // If the FIRST (left edge) pixel being searched on row is hot
                    // EXPAND the left edge, multiple pixels if necessary,
                    // to bring into scope all 'attached' hot pixels on THIS row.
                    while ((imgThreshold.Data[currY, startX + fromX, 0] != 0) && (startX + fromX > 0))
                        fromX--;

                    for (currX = startX + fromX; (currX < startX + toX) && (currX < CombProcess.VideoData.ImageWidth); currX++)
                    {
                        // Set inputSearched[y,x] = true
                        inputSearched[currY * CombProcess.VideoData.ImageWidth + currX] = true;

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

                            MinHeat = Math.Min(MinHeat, currHeat);
                            MaxHeat = Math.Max(MaxHeat, currHeat);
                            Pixels.Add(new PixelHeat(BlockId, FeatureId, currY, currX, currHeat));

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
                    PixelBox = new Rectangle(rectLeft, rectTop, rectRight - rectLeft + 1, rectBottom - rectTop + 1);

                    // If the rectangle density is lower than the density threshold then stop expanding. Avoids large, difuse features.
                    if (!PixelDensityGood)
                        break;

                    // If this fearure is larger than allowed then stop expanding
                    if (FeatureOverSized)
                        break;
                }

                // Is this feature significant?
                bool sizeOk = (NumHotPixels >= CombProcess.ProcessConfig.FeatureMinPixels);
                bool densityOk = PixelDensityGood;
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
        public override void Consume(ProcessFeature otherFeature)
        {
            // Transfer the pixels
            var combFeature = otherFeature as CombFeature;
            if(combFeature.Pixels != null)
                Pixels.AddRange(combFeature.Pixels);
            combFeature.Pixels = null;

            base.Consume(otherFeature);
        }


        // Get the class's settings as datapairs for use in ObjectForm
        public DataPairList GetSettings_ObjectForm(RelativeLocation objectLocation)
        {
            return new DataPairList
            {
                { "Feature", FeatureId },
                { "Attributes", Attributes },
                { "NorthingDiffCM", ( LocationM != null ? (LocationM.NorthingM - objectLocation.NorthingM ) * 100 : 0), 0},
                { "EastingDiffCM", ( LocationM != null ? (LocationM.EastingM - objectLocation.EastingM) * 100 : 0), 0},
                { "HeightCM", HeightM * 100, 0 },
                { "WidthPixels", PixelBox.Width },
                { "DepthPixels", PixelBox.Height },
                { "#HotPixels", NumHotPixels },
            };
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        override public DataPairList GetSettings()
        {
            var settings = base.GetSettings();

            settings.Add("Num Hot Pixels", NumHotPixels);
            settings.Add("Density Perc", DensityPerc); // 0 to 100
            settings.Add("Density Good", PixelDensityGood);
            settings.Add("Leg", (Block != null ? Block.FlightLegId : 0));

            // Horizontal distance from feature to drone.
            settings.Add("RangeM", (Block != null ? RelativeLocation.DistanceM(LocationM, Block.DroneLocnM) : 0 ), LocationNdp);  

            return settings;
        }
    }


    // A list of Comb features bound to a specific Block
    public class CombFeatureLogic 
    {
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
        public static void CreateFeaturesFromImage(
            CombProcess combProcess,
            ProcessFeatureList featuresInBlock,
            ProcessBlock block,
            Image<Bgr, byte> imgOriginal,
            Image<Gray, byte> imgThreshold)
        {
            int y = 0;
            int x = 0;

            try
            {
                // Create an Array[y, x] of bools. Booleans default to false
                int imageHeight = combProcess.VideoData.ImageHeight;
                int imageWidth = combProcess.VideoData.ImageWidth;
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
                                var feature = new CombFeature(combProcess, block, FeatureTypeEnum.Real);

                                feature.PixelNeighborSearch(
                                    ref inputSearched,
                                    imgOriginal, imgThreshold,
                                    y, x);

                                featuresInBlock.AddFeature(feature);
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

    };
}
