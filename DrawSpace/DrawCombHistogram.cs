// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
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


        public DrawHistogram(ProcessDrawScope drawScope, List<int> values, int min, int max, int scale = 1) : base(drawScope, true, true)
        {
            DrawScope = drawScope;

            Values = values;
            MaxFreq = (Values.Count == 0 ? 1 : Math.Max(1, Values.Max()));

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

                            var x = StepToWidth(horizValue) / Scale + 1;
                            var width = Math.Max(1, (int)StepWidthPxs - 2);
                            var rect = new Rectangle( x, pxsDown, width, OriginPixel.Y - pxsDown);
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
        public DrawHeightHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, ProcessObjList objs) :
            base(drawScope, objs.HistogramHeightM(), 0, (int)Math.Ceiling(objs.MaxHeightM))
        {
            FilterMin = (drawObjectScope == null ? ProcessObjectModel.UnknownHeight : drawObjectScope.MinHeightM);
            FilterMax = (drawObjectScope == null ? 10 : drawObjectScope.MaxHeightM);
        }
    }


    public class DrawSizeHistogram : DrawHistogram
    {
        public const int Scale = 100; // Each bar represents a 100cm2 increase in size

        public DrawSizeHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, ProcessObjList objs) :
            base(drawScope, objs.HistogramSizeCm2(Scale), 0, (int)Math.Ceiling(objs.MaxSizeCM2), Scale)
        {
            FilterMin = 0;
            FilterMax = (drawObjectScope == null ? 1000 : drawObjectScope.MaxSizeCM2);
        }
    }


    public class DrawHeatHistogram : DrawHistogram
    {
        public DrawHeatHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, ProcessObjList objs) :
            base(drawScope, objs.HistogramHeat(), objs.MinHeat, objs.MaxHeat)
        {
            FilterMin = (drawObjectScope == null ? 235 : drawObjectScope.MinHeat);
            FilterMax = (drawObjectScope == null ? 255 : drawObjectScope.MaxHeat);
        }

        public override void CurrImage(ref Image<Bgr, byte> image)
        {
            base.CurrImage(ref image);
        }
    }


    public class DrawRangeHistogram : DrawHistogram
    {
        public DrawRangeHistogram(ProcessDrawScope drawScope, ObjectDrawScope drawObjectScope, ProcessObjList objs) :
            base(drawScope, objs.HistogramRangeM(), 0, objs.MaxRangeM)
        {
            FilterMin = (drawObjectScope == null ? 0 : drawObjectScope.MinRangeM);
            FilterMax = (drawObjectScope == null ? 100 : drawObjectScope.MaxRangeM);
        }
    }
}