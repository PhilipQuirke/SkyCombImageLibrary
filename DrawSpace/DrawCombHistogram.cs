// Copyright SkyComb Limited 2023. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessLogic;
using System.Drawing;


namespace SkyCombImage.DrawSpace
{
    // Code to draw histogram
    public class DrawHistogram : DroneDrawGraph
    {
        public ProcessDrawScope DrawScope { get; }

        private List<int> Values;
        private int MaxFreq;
        private int Scale;


        // Sometimes user chooses to view a subset of the raw data.
        public int FilterMin { get; set; } = UnknownValue;
        public int FilterMax { get; set; } = UnknownValue;


        public DrawHistogram(ProcessDrawScope drawScope, List<int> values, int min, int max, int scale=1) : base(drawScope, true, true)
        {
            DrawScope = drawScope;

            Values = values;
            MaxFreq = (Values.Count == 0 ? 1 : Math.Max(1, Values.Max()));

            MinHorizRaw = min;
            MaxHorizRaw = max;

            VertBottomLabel = "";
            VertTopLabel = MaxFreq.ToString();

            HorizLeftLabel = MinHorizRaw.ToString();
            HorizRightLabel = MaxHorizRaw.ToString();

            MaxHorizRaw += 1; // Extra room needed to draw the last histogram bar width

            Scale = scale;

            TextFontSize = 8;
        }


        public override void Initialise(Size size)
        {
            base.Initialise(size);

            CalculateStepWidthAndStride(MinHorizRaw, MaxHorizRaw);

            if(BaseImage != null)
                DrawAxisesAndLabels(ref BaseImage);
        }


        public override void CurrImage(ref Image<Bgr, byte> image)
        {
            try
            {
                if (Values.Count <= 2)
                    return;

                // Draw series of rectangles in two passes: Out of filter range first, then in filter range (so on top).
                for (int pass = 0; pass <= 1; pass++)
                    for (int rectNum = 0; rectNum < Values.Count; rectNum++)
                    {
                        int value = Values[rectNum];
                        if( value <= 0)
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
                            var rect = new Rectangle(
                                StepToWidth(horizValue) + 1,
                                pxsDown,
                                Math.Max(1, (int)StepWidthPxs - 2),
                                OriginPixel.Y - pxsDown);

                            image.Draw(rect, theColor, thickness);
                        }
                    }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawHistogram.CurrImage", ex);
            }
        }
    }


    public class DrawHeightHistogram : DrawHistogram
    {
        public DrawHeightHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, CombObjList objs) :
            base(drawScope, objs.HistogramHeightM(), 0, (int)Math.Ceiling(objs.MaxHeightM))
        {
            FilterMin = (drawObjectScope == null ? 0 : drawObjectScope.MinHeightM);
            FilterMax = (drawObjectScope == null ? 10 : drawObjectScope.MaxHeightM);
        }
    }


    public class DrawSizeHistogram : DrawHistogram
    {
        public DrawSizeHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, CombObjList objs) :
            base(drawScope, objs.HistogramSize1000Cm2(), 0, (int)Math.Ceiling(objs.MaxSizeCM2), 1000)
        {
            FilterMin = (drawObjectScope == null ? 0 : drawObjectScope.MinSizeCM2);
            FilterMax = (drawObjectScope == null ? 1000 : drawObjectScope.MaxSizeCM2);
        }
    }


    public class DrawHeatHistogram : DrawHistogram
    {
        public DrawHeatHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, CombObjList objs) :
            base(drawScope, objs.HistogramHeat(), objs.MinHeat, objs.MaxHeat)
        {
            FilterMin = (drawObjectScope == null ? 235 : drawObjectScope.MinHeat);
            FilterMax = (drawObjectScope == null ? 255 : drawObjectScope.MaxHeat);
        }
    }


    public class DrawRangeHistogram : DrawHistogram
    {
        public DrawRangeHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, CombObjList objs) :
            base(drawScope, objs.HistogramRangeM(), 0, objs.MaxRangeM)
        {
            FilterMin = (drawObjectScope == null ? 0 : drawObjectScope.MinRangeM);
            FilterMax = (drawObjectScope == null ? 100 : drawObjectScope.MaxRangeM);
        }
    }
}