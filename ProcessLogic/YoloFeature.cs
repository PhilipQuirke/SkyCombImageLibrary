// Copyright SkyComb Limited 2024. All rights reserved. 
using Compunet.YoloV8.Data;
using Compunet.YoloV8.Metadata;
using Emgu.CV.Structure;
using Emgu.CV;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A Yolo feature 
    public class YoloFeature : ProcessFeature
    {
        // Results of YoloDetect image processing
        public YoloName? YoloName { get; init; } = null;
        public float Confidence { get; init; } = 0f;


        public YoloFeature(YoloProcess yoloProcess, int blockId, Detection box) : base(yoloProcess, blockId, FeatureTypeEnum.Real)
        {
            ResetCalcedMemberData();

            PixelBox = new System.Drawing.Rectangle(box.Bounds.Left, box.Bounds.Top, box.Bounds.Width, box.Bounds.Height);
            Significant = true;
            YoloName = box.Name;
            Confidence = box.Confidence;
        }


        // Constructor used when loaded objects from the datastore
        public YoloFeature(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, settings)
        {
            YoloName = null;
            Confidence = 0f;
        }


        // Evaluate the MinHeat, MaxHeat and PixelSize of this feature
        public void CalculateHeat(
           in Image<Bgr, byte> imgOriginal,
           in Image<Gray, byte> imgThreshold)
        {
            MinHeat = 255 * 3;
            MaxHeat = 0;
            NumHotPixels = 0;

            int left = Math.Max(PixelBox.Left, 0);
            int top = Math.Max(PixelBox.Top, 0);
            int right = Math.Min(PixelBox.Right, imgOriginal.Width);
            int bottom = Math.Min(PixelBox.Bottom, imgOriginal.Height);

            for (int y = top; y < bottom; y++)
            {
                for (int x = left; x < right; x++)
                {
                    if (imgThreshold[y, x].Intensity > 0)
                    {
                        Bgr color = imgOriginal[y, x];
                        int currHeat = (int)((color.Blue + color.Green + color.Red) / 3);
                        MinHeat = Math.Min(MinHeat, currHeat);
                        MaxHeat = Math.Max(MaxHeat, currHeat);
                        NumHotPixels++;
                    }
                }
            }

            Significant = (NumHotPixels >= ProcessAll.ProcessConfig.FeatureMinPixels);
            IsTracked = Significant;
        }
    };
}
