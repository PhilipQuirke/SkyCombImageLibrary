using System;
using System.Runtime.InteropServices;
using static SkyCombImageLibrary.ProcessLogic.DJI.DjiThermalApi;

namespace SkyCombImageLibrary.ProcessLogic.DJI
{
    /// <summary>
    /// High-level wrapper for DJI Thermal SDK
    /// </summary>
    public static class UnitTest
    {
        public static string GetSdkVersion()
        {
            var version = new DirpApiVersion();
            var result = dirp_get_api_version(ref version);

            if (result == DirpRetCode.DIRP_SUCCESS)
            {
                return $"{version.major}.{version.minor}.{version.revision} (Magic: {version.magic})";
            }
            return "Unknown";
        }

        public static void Test(string input, string prefix)
        {
            try
            {
                string version = GetSdkVersion();
                Console.WriteLine($"DJI SDK Version: {version}");

                using (var processor = new ThermalImageProcessor())
                {
                    // Load thermal image
                    var thermalData = processor.LoadImage(input);

                    // Display metadata
                    Console.WriteLine($"Image Size: {thermalData.Width}x{thermalData.Height}");
                    Console.WriteLine($"Temperature Range: {thermalData.MinTemperature:F2}°C to {thermalData.MaxTemperature:F2}°C");
                    Console.WriteLine($"Mean Temperature: {thermalData.MeanTemperature:F2}°C");
                    Console.WriteLine();
                    Console.WriteLine("Measurement Parameters:");
                    Console.WriteLine($"  Distance: {thermalData.Distance}m");
                    Console.WriteLine($"  Humidity: {thermalData.Humidity}%");
                    Console.WriteLine($"  Emissivity: {thermalData.Emissivity}");
                    Console.WriteLine($"  Reflection: {thermalData.Reflection}°C");

                    // Get temperature at specific point
                    int centerX = thermalData.Width / 2;
                    int centerY = thermalData.Height / 2;
                    float centerTemp = processor.GetTemperatureAt(thermalData, centerX, centerY);
                    Console.WriteLine($"\nCenter pixel ({centerX},{centerY}): {centerTemp:F2}°C");

                    // Export to CSV
                    processor.ExportToCsv(thermalData, prefix + "thermal_data.csv");
                    Console.WriteLine("\nExported to thermal_data.csv");

                    // Access raw data
                    Console.WriteLine($"\nTotal pixels: {thermalData.TemperatureData.Length}");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
        }
    }
}