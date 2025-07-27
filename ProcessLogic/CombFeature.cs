// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A Comb feature, is a dense cluster of hot pixels, associated 1-1 with a Block
    public class CombFeature : ProcessFeature
    {
        public CombFeature(CombProcess combProcess, ProcessBlock block, FeatureTypeEnum type) : base(combProcess, block.BlockId, type)
        {
            ResetCalcedMemberData();
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
            in Image<Bgr, byte> imgOriginal,        // read-only
            in Image<Gray, byte> imgThreshold,      // read-only
            int startY,
            int startX)
        {
            int currY = 0;
            int currX = 0;
            int imageHeight = imgOriginal.Height;
            int imageWidth = imgOriginal.Width;

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
                for (currY = startY; currY < imageHeight; currY++)
                {
                    int hotPixelsInRow = 0;

                    // CASE: EXPAND LEFT
                    // If the FIRST (left edge) pixel being searched on row is hot
                    // EXPAND the left edge, multiple pixels if necessary,
                    // to bring into scope all 'attached' hot pixels on THIS row.
                    while ((imgThreshold.Data[currY, startX + fromX, 0] != 0) && (startX + fromX > 0))
                        fromX--;

                    for (currX = startX + fromX; (currX < startX + toX) && (currX < imageWidth); currX++)
                    {
                        // Set inputSearched[y,x] = true
                        inputSearched[currY * imageWidth + currX] = true;

                        // If imgInputGray[y,x] is a hot pixel
                        var currPixelIsHot = (imgThreshold.Data[currY, currX, 0] != 0);
                        if (currPixelIsHot)
                        {
                            hotPixelsInRow++;

                            // Evaluate the heat from the original image (not the threshold image)
                            Bgr orgColor = imgOriginal[currY, currX];
                            AddHotPixel(currY, currX, orgColor);

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

                    // If this fearure is larger than allowed then stop expanding
                    if (FeatureOverSized)
                        break;
                }

                // Is this feature significant?
                Significant = (NumHotPixels >= ProcessConfigModel.FeatureMinPixels);
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
            if (otherFeature.Pixels != null)
            {
                Pixels.AddRange(otherFeature.Pixels);
                CalcSumAndNumHotPixels();

                otherFeature.ClearHotPixels();
            }

            base.Consume(otherFeature);
        }
    }

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
            in Image<Bgr, byte> imgOriginal,    // read-only
            in Image<Gray, byte> imgThreshold)  // read-only
        {
            int y = 0;
            int x = 0;
            int imageHeight = imgOriginal.Height; 
            int imageWidth = imgOriginal.Width; 
            // Create an Array[y, x] of bools. Booleans default to false
            bool[] inputSearched = new bool[imageHeight * imageWidth];

            try
            {
                // For each location where inputSearched[y,x] = false
                for (y = 0; y < imageHeight; y++)
                    for (x = 0; x < imageWidth; x++)
                    {
                        var index = y * imageWidth + x;
                        if (!inputSearched[index])
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
                throw BaseConstants.ThrowException("CombFeatureList.Process: y=" + y + " x=" + x, ex);
            }
        }
    }
}
