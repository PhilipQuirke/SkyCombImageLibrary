// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using System.Drawing;

namespace SkyCombImage.DrawSpace
{
    // Code to draw histogram
    public class DrawHistogram : DroneDrawGraph
    {
        public ProcessDrawScope DrawScope { get; }

        private readonly List<int> Values;
        private readonly int MaxFreq;
        private readonly int Scale;


        // Sometimes user chooses to view a subset of the raw data.
        public int FilterMin { get; set; } = UnknownValue;
        public int FilterMax { get; set; } = UnknownValue;


        public DrawHistogram(ProcessDrawScope drawScope, List<int> values, int min, int max, int scale = 1) : base(drawScope, true, true)
        {
            DrawScope = drawScope;

            Values = values;
            MaxFreq = (Values == null || Values.Count == 0 ? 1 : Math.Max(1, Values.Max()));

            VertBottomLabel = "";
            VertTopLabel = MaxFreq.ToString();

            MinHorizRaw = min;
            MaxHorizRaw = max;

            HorizLeftLabel = MinHorizRaw.ToString();
            HorizRightLabel = MaxHorizRaw.ToString();

            Scale = scale;
            TextFontSize = 8;
        }


        public override void Initialise(Size size)
        {
            base.Initialise(size);

            MaxHorizRaw++;
            CalculateStepWidthAndStride(MinHorizRaw, MaxHorizRaw, Scale);
            MaxHorizRaw--;

            if (BaseImage != null)
                DrawAxises(ref BaseImage);
        }


        public override void CurrImage(ref Image<Bgr, byte> image, List<Image>? sizeImages = null)
        {
            try
            {
                if (Values == null || Values.Count <= 2)
                    return;

                // Draw series of rectangles in two passes: Out of filter range first, then in filter range (so on top).
                for (int pass = 0; pass <= 1; pass++)
                    for (int rectNum = 0; rectNum < Values.Count; rectNum++)
                    {
                        int value = Values[rectNum];
                        if (value <= 0)
                            continue;

                        var horizValue = rectNum * Scale + MinHorizRaw;

                        bool outOfFilterRange =
                            ((FilterMin != UnknownValue) && (horizValue < FilterMin)) ||
                            ((FilterMax != UnknownValue) && (horizValue > FilterMax));

                        if (((pass == 0) && outOfFilterRange) ||
                            ((pass == 1) && !outOfFilterRange))
                        {
                            // Show "out of filter range" histogram bars as dark gray rectangles.
                            var theColor = (pass == 0 ? DroneColors.DarkGrayBgr : DroneColors.LegNameBgr);
                            var thickness = -1; // If thickness is less than 1, the rectangle is filled up

                            var pxsDown = RawDataToHeightPixels(value, MaxFreq);

                            var x = StepToWidth(horizValue) / Scale + 1;
                            var width = Math.Max(1, (int)StepWidthPxs - 2);
                            var rect = new Rectangle(x, pxsDown, width, OriginPixel.Y - pxsDown);
                            image.Draw(rect, theColor, thickness);

                            if (sizeImages != null)
                            {
                                // Convert System.Drawing.Image to Emgu.CV Image
                                Bitmap bitmap = new Bitmap(sizeImages[rectNum]);
                                Emgu.CV.Image<Bgra, byte> emguImage = bitmap.Clone(new Rectangle(0, 0, bitmap.Width, bitmap.Height), System.Drawing.Imaging.PixelFormat.Format32bppArgb).ToImage<Bgra, byte>();
                                int imgX = x;
                                int imgY = OriginPixel.Y - emguImage.Height;
                                // When copying, you'll want to use a method that respects alpha
                                for (int y = 0; y < emguImage.Height; y++)
                                {
                                    for (int w = 0; w < emguImage.Width; w++)
                                    {
                                        // Check if pixel is not fully transparent
                                        if (emguImage.Data[y, w, 3] > 0)  // Alpha channel
                                        {
                                            if (imgX + w < image.Width && imgY + y < image.Height)
                                            {
                                                // Blend transparent pixels
                                                byte blue = emguImage.Data[y, w, 0];
                                                byte green = emguImage.Data[y, w, 1];
                                                byte red = emguImage.Data[y, w, 2];
                                                byte alpha = emguImage.Data[y, w, 3];

                                                // Alpha blending
                                                image[imgY + y, imgX + w] = new Bgr(
                                                    (byte)((image[imgY + y, imgX + w].Blue * (255 - alpha) + blue * alpha) / 255),
                                                    (byte)((image[imgY + y, imgX + w].Green * (255 - alpha) + green * alpha) / 255),
                                                    (byte)((image[imgY + y, imgX + w].Red * (255 - alpha) + red * alpha) / 255)
                                                );
                                            }
                                        }
                                    }
                                }
                            }

                        }
                    }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawHistogram.CurrImage", ex);
            }
        }
    }

}