// Copyright SkyComb Limited 2026. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;


namespace SkyCombImage.ProcessModel
{
    public sealed class BlockThermalData
    {
        private readonly byte[,,]? thresholdSource;

        public ushort[] RawData { get; }
        public int RawWidth { get; }
        public int RawHeight { get; }
        public int GlobalMinRadioHeat { get; }
        public int GlobalMaxRadioHeat { get; }


        public BlockThermalData(
            Image<Gray, byte>? thresholdSource,
            ushort[] rawData, int rawWidth, int rawHeight,
            int globalMinRadioHeat, int globalMaxRadioHeat)
        {
            ArgumentNullException.ThrowIfNull(rawData);
            if (rawWidth <= 0 || rawHeight <= 0 || rawData.LongLength != (long)rawWidth * rawHeight)
                throw new ArgumentException("Raw radiometric data must match the image dimensions.", nameof(rawData));
            if (thresholdSource != null && (thresholdSource.Width != rawWidth || thresholdSource.Height != rawHeight))
                throw new ArgumentException("Threshold source must match the raw image dimensions.", nameof(thresholdSource));

            // Keep managed snapshots: the worker disposes its image when it advances to the next frame.
            this.thresholdSource = thresholdSource == null ? null : (byte[,,])thresholdSource.Data.Clone();
            RawData = (ushort[])rawData.Clone();
            RawWidth = rawWidth;
            RawHeight = rawHeight;
            GlobalMinRadioHeat = globalMinRadioHeat;
            GlobalMaxRadioHeat = globalMaxRadioHeat;
        }


        // The caller owns and disposes this temporary image.
        public Image<Gray, byte>? CreateThresholdSource()
        {
            return thresholdSource == null ? null : new Image<Gray, byte>(thresholdSource);
        }
    }
}
