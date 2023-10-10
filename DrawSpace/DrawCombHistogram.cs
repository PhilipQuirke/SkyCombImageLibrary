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
    public class DrawHistogram : DrawGraph
    {
        public DrawScope DrawScope { get; }

        private List<int> Values;
        private int MaxFreq;
        private int Scale;


        public DrawHistogram(DrawScope drawScope, List<int> values, int min, int max, int scale=1) : base(drawScope, true, true)
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
            Scale = scale;

            TextFontSize = 8;
        }


        public override void Initialise(Size size)
        {
            base.Initialise(size);

            CalculateStepWidthAndStride(MinHorizRaw, MaxHorizRaw);

            DrawAxisesAndLabels(ref BaseImage);
        }


        public override void CurrImage(ref Image<Bgr, byte> image)
        {
            try
            {
                if (Values.Count <= 2)
                    return;

                // Draw series of rectangles
                int numRects = Values.Count;
                for (int rectNum = 0; rectNum < numRects; rectNum++)
                {
                    int value = Values[rectNum];
                    if( value <= 0)
                        continue;

                    var rect = new Rectangle(
                        StepToWidth(rectNum * Scale + MinHorizRaw),
                        OriginPixel.Y,
                        (int)StepWidthPxs,
                        RawDataToHeightPixels(value, MaxFreq) - OriginPixel.Y);

                    image.Draw(rect, DroneColors.LegNameBgr, 1);
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
        public DrawHeightHistogram(DrawScope drawScope, CombObjList objs) :
            base(drawScope, objs.HistogramHeightM(), 0, (int)Math.Ceiling(objs.MaxHeightM))
        {
        }
    }


    public class DrawSizeHistogram : DrawHistogram
    {
        public DrawSizeHistogram(DrawScope drawScope, CombObjList objs) :
            base(drawScope, objs.HistogramSize1000Cm2(), 0, (int)Math.Ceiling(objs.MaxSizeCM2), 1000)
        {
        }
    }


    public class DrawHeatHistogram : DrawHistogram
    {
        public DrawHeatHistogram(DrawScope drawScope, CombObjList objs) :
            base(drawScope, objs.HistogramHeat(), objs.MinHeat, objs.MaxHeat)
        {
        }
    }


    public class DrawRangeHistogram : DrawHistogram
    {
        public DrawRangeHistogram(DrawScope drawScope, CombObjList objs) :
            base(drawScope, objs.HistogramRangeM(), 0, objs.MaxRangeM)
        {
        }
    }
}