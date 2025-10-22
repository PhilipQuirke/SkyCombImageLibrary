using System;
using System.IO;
using System.Linq;
using System.Text;

namespace SkyCombImageLibrary.ProcessLogic.DJI
{
    /// <summary>
    /// High-level wrapper for DJI Thermal SDK
    /// </summary>
    public class ThermalImageProcessor : IDisposable
    {
        private IntPtr _handle = IntPtr.Zero;
        private bool _disposed = false;


        /// <summary>
        /// Validate if a file is a valid DJI thermal R-JPEG
        /// </summary>
        public static DiagnosticInfo ValidateThermalImage(string filePath)
        {
            var info = new DiagnosticInfo { FilePath = filePath };

            try
            {
                // Check file exists
                if (!File.Exists(filePath))
                {
                    info.IsValid = false;
                    info.ErrorMessage = "File not found";
                    return info;
                }

                // Get file info
                var fileInfo = new FileInfo(filePath);
                info.FileSize = fileInfo.Length;
                info.FileName = fileInfo.Name;

                // Check file size (should be > 100KB for thermal images)
                if (fileInfo.Length < 100000)
                {
                    info.IsValid = false;
                    info.ErrorMessage = "File too small - likely not a thermal image";
                    info.Warnings.Add($"File size: {fileInfo.Length:N0} bytes (expected > 100KB)");
                    return info;
                }

                // Check file extension
                string ext = fileInfo.Extension.ToLowerInvariant();
                if (ext != ".jpg" && ext != ".jpeg")
                {
                    info.Warnings.Add($"Unexpected extension: {ext}");
                }

                // Check filename pattern for thermal images
                string name = fileInfo.Name;
                bool hasRSuffix = name.Contains("_R.") || name.Contains("_T.");
                if (!hasRSuffix)
                {
                    info.Warnings.Add("Filename doesn't contain '_R' or '_T' suffix (expected for thermal images)");
                }

                // Read file header
                byte[] imageData = File.ReadAllBytes(filePath);

                // Check JPEG magic bytes
                if (imageData.Length < 2 || imageData[0] != 0xFF || imageData[1] != 0xD8)
                {
                    info.IsValid = false;
                    info.ErrorMessage = "Not a valid JPEG file (invalid header)";
                    return info;
                }

                info.HasJpegHeader = true;

                // Check for EXIF/thermal metadata markers
                info.HasExifData = ContainsSequence(imageData, new byte[] { 0xFF, 0xE1 }); // EXIF marker
                info.HasApp3Data = ContainsSequence(imageData, new byte[] { 0xFF, 0xE3 }); // APP3 marker (DJI thermal)

                // Look for DJI thermal signature
                string dataString = Encoding.ASCII.GetString(imageData, 0, Math.Min(10000, imageData.Length));
                info.HasDjiSignature = dataString.Contains("DJI") || dataString.Contains("FLIR");

                // Try to create handle
                IntPtr testHandle = IntPtr.Zero;
                var result = DjiThermalApi.dirp_create_from_rjpeg(
                    imageData,
                    imageData.Length,
                    ref testHandle);

                info.DjiSdkResult = (int)result;
                info.DjiSdkResultName = result.ToString();

                if (result == DjiThermalApi.DirpRetCode.DIRP_SUCCESS)
                {
                    info.IsValid = true;
                    info.SuccessMessage = "Valid DJI thermal R-JPEG";

                    // Get image resolution
                    var resolution = new DjiThermalApi.DirpResolution();
                    DjiThermalApi.dirp_get_rjpeg_resolution(testHandle, ref resolution);
                    info.Width = resolution.Width;
                    info.Height = resolution.Height;

                    // Clean up
                    DjiThermalApi.dirp_destroy(testHandle);
                }
                else
                {
                    info.IsValid = false;
                    info.ErrorMessage = $"DJI SDK rejected image: {result} ({(int)result})";

                    // Provide specific guidance based on error code
                    info.ErrorMessage += "\n" + GetErrorGuidance(result);
                }
            }
            catch (Exception ex)
            {
                info.IsValid = false;
                info.ErrorMessage = $"Exception: {ex.Message}";
            }

            return info;
        }

        private static bool ContainsSequence(byte[] data, byte[] sequence)
        {
            for (int i = 0; i <= data.Length - sequence.Length; i++)
            {
                bool match = true;
                for (int j = 0; j < sequence.Length; j++)
                {
                    if (data[i + j] != sequence[j])
                    {
                        match = false;
                        break;
                    }
                }
                if (match) return true;
            }
            return false;
        }

        private static string GetErrorGuidance(DjiThermalApi.DirpRetCode errorCode)
        {
            return errorCode switch
            {
                DjiThermalApi.DirpRetCode.DIRP_ERROR_INVALID_HEADER =>
                    "Image header is invalid. This is not a DJI thermal image.",

                DjiThermalApi.DirpRetCode.DIRP_ERROR_RJPEG_PARSE =>
                    "Failed to parse R-JPEG. Image may be corrupted or from unsupported camera.",

                DjiThermalApi.DirpRetCode.DIRP_ERROR_FORMAT_INPUT =>
                    "Input format error. Ensure this is a thermal R-JPEG from a DJI drone.",

                (DjiThermalApi.DirpRetCode)(-15) =>
                    "Error -15: Not a valid DJI thermal R-JPEG or unsupported camera model.\n" +
                    "Common causes:\n" +
                    "  • File is a regular RGB image, not thermal (_R.JPG)\n" +
                    "  • Camera model not supported by SDK v1.7\n" +
                    "  • Image was processed/edited and lost thermal data\n" +
                    "  • File is corrupted",

                _ => $"Unknown error code: {(int)errorCode}"
            };
        }



        /// <summary>
        /// Load and process a DJI thermal R-JPEG image
        /// </summary>
        public ThermalImageData LoadImage(string filePath)
        {
            // First validate the image
            var validation = ValidateThermalImage(filePath);

            if (!validation.IsValid)
            {
                throw new InvalidDataException(
                    $"Invalid thermal image: {validation.ErrorMessage}\n\n" +
                    $"Diagnostic Report:\n{validation.GetReport()}");
            }

            // Read entire file into memory
            byte[] imageData = File.ReadAllBytes(filePath);

            // Create DIRP handle
            var result = DjiThermalApi.dirp_create_from_rjpeg(
                imageData,
                imageData.Length,
                ref _handle);

            if (result != DjiThermalApi.DirpRetCode.DIRP_SUCCESS)
                throw new Exception($"Failed to create DIRP handle: {result}");

            // Extract all data
            var thermalData = new ThermalImageData();

            // Get resolution
            var resolution = new DjiThermalApi.DirpResolution();
            result = DjiThermalApi.dirp_get_rjpeg_resolution(_handle, ref resolution);
            if (result == DjiThermalApi.DirpRetCode.DIRP_SUCCESS)
            {
                thermalData.Width = resolution.Width;
                thermalData.Height = resolution.Height;
            }

            // Get measurement parameters
            var measParams = new DjiThermalApi.DirpMeasurementParams();
            result = DjiThermalApi.dirp_get_measurement_params(_handle, ref measParams);
            if (result == DjiThermalApi.DirpRetCode.DIRP_SUCCESS)
            {
                thermalData.Distance = measParams.Distance;
                thermalData.Humidity = measParams.Humidity;
                thermalData.Emissivity = measParams.Emissivity;
                thermalData.Reflection = measParams.Reflection;
            }

            // Get color bar info
            var colorBar = new DjiThermalApi.DirpColorBar();
            result = DjiThermalApi.dirp_get_color_bar(_handle, ref colorBar);
            if (result == DjiThermalApi.DirpRetCode.DIRP_SUCCESS)
            {
                thermalData.ColorBarHigh = colorBar.High / 10.0f;
                thermalData.ColorBarLow = colorBar.Low / 10.0f;
                thermalData.ColorBarManual = colorBar.ManualEnable == 1;
            }

            // Extract temperature data
            int pixelCount = thermalData.Width * thermalData.Height;
            thermalData.TemperatureData = new float[pixelCount];

            result = DjiThermalApi.dirp_measure(_handle, thermalData.TemperatureData, pixelCount);
            if (result != DjiThermalApi.DirpRetCode.DIRP_SUCCESS)
                throw new Exception($"Failed to extract temperature data: {result}");

            // Calculate statistics
            thermalData.MinTemperature = thermalData.TemperatureData.Min();
            thermalData.MaxTemperature = thermalData.TemperatureData.Max();
            thermalData.MeanTemperature = thermalData.TemperatureData.Average();

            return thermalData;
        }

        /// <summary>
        /// Get temperature at specific pixel coordinates
        /// </summary>
        public float GetTemperatureAt(ThermalImageData imageData, int x, int y)
        {
            if (x < 0 || x >= imageData.Width || y < 0 || y >= imageData.Height)
                throw new ArgumentOutOfRangeException($"Coordinates out of bounds: ({x},{y})");

            int index = y * imageData.Width + x;
            return imageData.TemperatureData[index];
        }

        /// <summary>
        /// Export temperature data to CSV
        /// </summary>
        public void ExportToCsv(ThermalImageData imageData, string outputPath)
        {
            using (var writer = new StreamWriter(outputPath))
            {
                // Write header
                writer.WriteLine("X,Y,Temperature_C");

                // Write data
                for (int y = 0; y < imageData.Height; y++)
                {
                    for (int x = 0; x < imageData.Width; x++)
                    {
                        int index = y * imageData.Width + x;
                        float temp = imageData.TemperatureData[index];
                        writer.WriteLine($"{x},{y},{temp:F2}");
                    }
                }
            }
        }

        /// <summary>
        /// Export temperature data to 32-bit TIFF (for use in GIS software, etc.)
        /// </summary>
        public void ExportToTiff(ThermalImageData imageData, string outputPath)
        {
            // Note: You'll need a TIFF library like BitMiracle.LibTiff.NET
            // This is a simplified example
            File.WriteAllBytes(outputPath, CreateSimpleTiff(imageData));
        }

        private byte[] CreateSimpleTiff(ThermalImageData imageData)
        {
            // Implement TIFF writer or use library like BitMiracle.LibTiff.NET
            // For now, just save as raw binary
            using (var ms = new MemoryStream())
            using (var writer = new BinaryWriter(ms))
            {
                foreach (float temp in imageData.TemperatureData)
                {
                    writer.Write(temp);
                }
                return ms.ToArray();
            }
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                if (_handle != IntPtr.Zero)
                {
                    DjiThermalApi.dirp_destroy(_handle);
                    _handle = IntPtr.Zero;
                }
                _disposed = true;
            }
        }
    }

    /// <summary>
    /// Contains diagnostic information about a thermal image file
    /// </summary>
    public class DiagnosticInfo
    {
        public string FilePath { get; set; } = "";
        public string FileName { get; set; } = "";
        public long FileSize { get; set; }
        public bool IsValid { get; set; }
        public string ErrorMessage { get; set; } = "";
        public string SuccessMessage { get; set; } = "";
        public List<string> Warnings { get; set; } = new();

        // JPEG analysis
        public bool HasJpegHeader { get; set; }
        public bool HasExifData { get; set; }
        public bool HasApp3Data { get; set; }
        public bool HasDjiSignature { get; set; }

        // DJI SDK results
        public int DjiSdkResult { get; set; }
        public string DjiSdkResultName { get; set; } = "";

        // Image properties (if valid)
        public int Width { get; set; }
        public int Height { get; set; }

        public string GetReport()
        {
            var sb = new StringBuilder();
            sb.AppendLine("═════════════════════════════════════════");
            sb.AppendLine("   Thermal Image Diagnostic Report");
            sb.AppendLine("═════════════════════════════════════════");
            sb.AppendLine();
            sb.AppendLine($"File: {FileName}");
            sb.AppendLine($"Path: {FilePath}");
            sb.AppendLine($"Size: {FileSize:N0} bytes ({FileSize / 1024.0:F1} KB)");
            sb.AppendLine();
            sb.AppendLine("File Analysis:");
            sb.AppendLine($"  JPEG Header: {(HasJpegHeader ? "✓" : "✗")}");
            sb.AppendLine($"  EXIF Data: {(HasExifData ? "✓" : "✗")}");
            sb.AppendLine($"  APP3 Marker: {(HasApp3Data ? "✓" : "✗")}");
            sb.AppendLine($"  DJI Signature: {(HasDjiSignature ? "✓" : "✗")}");
            sb.AppendLine();
            sb.AppendLine($"DJI SDK Result: {DjiSdkResultName} ({DjiSdkResult})");
            sb.AppendLine();

            if (IsValid)
            {
                sb.AppendLine($"✓ {SuccessMessage}");
                sb.AppendLine($"  Resolution: {Width}x{Height}");
            }
            else
            {
                sb.AppendLine($"✗ {ErrorMessage}");
            }

            if (Warnings.Count > 0)
            {
                sb.AppendLine();
                sb.AppendLine("Warnings:");
                foreach (var warning in Warnings)
                {
                    sb.AppendLine($"  ⚠ {warning}");
                }
            }

            sb.AppendLine("═════════════════════════════════════════");
            return sb.ToString();
        }
    }


    /// <summary>
    /// Container for thermal image data and metadata
    /// </summary>
    public class ThermalImageData
    {
        public int Width { get; set; }
        public int Height { get; set; }
        public float[] TemperatureData { get; set; } // Temperature in Celsius for each pixel

        // Measurement parameters
        public float Distance { get; set; }     // meters
        public float Humidity { get; set; }     // percentage
        public float Emissivity { get; set; }   // 0.01 to 1.0
        public float Reflection { get; set; }   // Celsius

        // Color bar settings
        public float ColorBarHigh { get; set; } // Celsius
        public float ColorBarLow { get; set; }  // Celsius
        public bool ColorBarManual { get; set; }

        // Statistics
        public float MinTemperature { get; set; }
        public float MaxTemperature { get; set; }
        public float MeanTemperature { get; set; }
    }
}