using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDroneLibrary.DroneLogic.DJI;
using System.Runtime.InteropServices;


namespace SkyCombImageLibrary.ProcessLogic
{
    public static class ThermalImageProcessor
    {
        /// <summary>
        /// Processes raw radiometric data to enhance hotspots (animals).
        /// </summary>
        /// <param name="jpgPath">Path to the R-JPEG file.</param>
        /// <param name="percentileLow">Lower percentile for windowing (e.g., 1 or 3).</param>
        /// <param name="percentileHigh">Upper percentile for windowing (e.g., 99 or 97).</param>
        /// <param name="claheTileSize">CLAHE grid size (e.g., 8, 12, or 16).</param>
        /// <param name="claheClipLimit">CLAHE clip limit (e.g., 40-60).</param>
        /// <param name="unsharpAmount">Unsharp mask strength (e.g., 0.4-0.7).</param>
        /// <param name="unsharpRadius">Gaussian blur radius for unsharp mask (e.g., 1-2).</param>
        /// <returns>Processed grayscale image as BGR format.</returns>
        public static Image<Bgr, byte> ProcessThermalImage(
            string jpgPath,
            double percentileLow = 3.0,
            double percentileHigh = 97.0,
            int claheTileSize = 12,
            double claheClipLimit = 50.0,
            double unsharpAmount = 0.5,
            double unsharpRadius = 1.5)
        {
            // Step 1: Load raw radiometric data
            (ushort[] rawData, int width, int height) = 
                DirpApiWrapper.GetRawRadiometricData(jpgPath);

            // Step 3: Convert to Mat (floating point for processing)
            Mat rawMat = ConvertToMat(rawData, width, height);

            // Step 4: Apply percentile windowing
            Mat normalized = ApplyPercentileWindowing(rawMat, percentileLow, percentileHigh);

            // Step 5: Convert to 8-bit for CLAHE
            Mat image8bit = new Mat();
            normalized.ConvertTo(image8bit, DepthType.Cv8U, 255.0);

            // Step 6: Apply CLAHE for local contrast enhancement
            Mat claheResult = ApplyCLAHE(image8bit, claheTileSize, claheClipLimit);

            // Step 7: Apply unsharp masking
            Mat sharpened = ApplyUnsharpMask(claheResult, unsharpAmount, unsharpRadius);

            // Step 8: Convert to Bgr format (grayscale in all channels)
            Image<Gray, byte> grayImage = sharpened.ToImage<Gray, byte>();
            Image<Bgr, byte> bgrImage = grayImage.Convert<Bgr, byte>();

            // Cleanup
            rawMat.Dispose();
            normalized.Dispose();
            image8bit.Dispose();
            claheResult.Dispose();
            sharpened.Dispose();
            grayImage.Dispose();

            return bgrImage;
        }


        /// <summary>
        /// Converts ushort array to OpenCV Mat.
        /// </summary>
        private static Mat ConvertToMat(ushort[] data, int width, int height)
        {
            Mat mat = new Mat(height, width, DepthType.Cv16U, 1);
            byte[] byteData = new byte[data.Length * sizeof(ushort)];
            Buffer.BlockCopy(data, 0, byteData, 0, byteData.Length);
            Marshal.Copy(byteData, 0, mat.DataPointer, byteData.Length);
            return mat;
        }

        /// <summary>
        /// Applies percentile-based windowing to clip outliers.
        /// </summary>
        private static Mat ApplyPercentileWindowing(Mat input, double lowPercentile, double highPercentile)
        {
            // Convert to array for percentile calculation
            ushort[] values = new ushort[input.Rows * input.Cols];
            Marshal.Copy(input.DataPointer, values.Select(x => (short)x).ToArray(), 0, values.Length);

            // Calculate percentiles
            Array.Sort(values);
            int lowIdx = (int)(values.Length * (lowPercentile / 100.0));
            int highIdx = (int)(values.Length * (highPercentile / 100.0));

            ushort lowVal = values[lowIdx];
            ushort highVal = values[highIdx];

            // Normalize to 0-1 range
            Mat normalized = new Mat();
            input.ConvertTo(normalized, DepthType.Cv32F);

            // Apply windowing: (value - low) / (high - low), clipped to [0, 1]
            CvInvoke.Subtract(normalized, new ScalarArray(lowVal), normalized);
            CvInvoke.Divide(normalized, new ScalarArray(highVal - lowVal), normalized);

            // Clip to [0, 1]
            Mat clipped = new Mat();
            CvInvoke.Threshold(normalized, clipped, 1.0, 1.0, ThresholdType.Trunc);
            CvInvoke.Threshold(clipped, normalized, 0.0, 0.0, ThresholdType.ToZero);
            normalized = clipped;

            return normalized;
        }

        /// <summary>
        /// Applies CLAHE (Contrast Limited Adaptive Histogram Equalization).
        /// </summary>
        private static Mat ApplyCLAHE(Mat input, int tileSize, double clipLimit)
        {
            Mat output = new Mat();
            CvInvoke.CLAHE(input, clipLimit, new System.Drawing.Size(tileSize, tileSize), output);
            return output;
        }

        /// <summary>
        /// Applies unsharp masking for edge enhancement.
        /// </summary>
        private static Mat ApplyUnsharpMask(Mat input, double amount, double radius)
        {
            // Create blurred version
            Mat blurred = new Mat();
            System.Drawing.Size kernelSize = new System.Drawing.Size(0, 0);
            CvInvoke.GaussianBlur(input, blurred, kernelSize, radius);

            // Calculate mask: original - blurred
            Mat mask = new Mat();
            CvInvoke.Subtract(input, blurred, mask);

            // Apply: sharpened = original + amount * mask
            Mat weighted = new Mat();
            CvInvoke.AddWeighted(input, 1.0, mask, amount, 0, weighted);

            blurred.Dispose();
            mask.Dispose();

            return weighted;
        }
    }
}