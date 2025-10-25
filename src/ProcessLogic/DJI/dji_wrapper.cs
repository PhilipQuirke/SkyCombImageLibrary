using System.Runtime.InteropServices;

namespace SkyCombImageLibrary.ProcessLogic.DJI
{
    public static class DirpApiWrapper
    {
        private const string DllName = "libdirp.dll"; // Ensure this DLL is available in your output directory

        // Native handle type
        private struct SafeDirpHandle : IDisposable
        {
            public IntPtr Handle;
            public void Dispose()
            {
                if (Handle != IntPtr.Zero)
                {
                    dirp_destroy(Handle);
                    Handle = IntPtr.Zero;
                }
            }
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct dirp_resolution_t
        {
            public int width;
            public int height;
        }

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int dirp_create_from_rjpeg(
            byte[] data, int size, out IntPtr ph);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int dirp_destroy(IntPtr h);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int dirp_get_rjpeg_resolution(
            IntPtr h, out dirp_resolution_t resolution);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int dirp_get_original_raw(
            IntPtr h, [Out] ushort[] raw_image, int size);

        /// <summary>
        /// Loads the raw radiometric data from a DJI R-JPEG file.
        /// </summary>
        /// <param name="jpgPath">Path to the R-JPEG file.</param>
        /// <returns>Raw radiometric data as a ushort array.</returns>
        public static ushort[] GetRawRadiometricData(string jpgPath)
        {
            if (string.IsNullOrWhiteSpace(jpgPath))
                throw new ArgumentException("JPG path must not be null or empty.", nameof(jpgPath));
            if (!File.Exists(jpgPath))
                throw new FileNotFoundException("JPG file not found.", jpgPath);

            byte[] rjpegData = File.ReadAllBytes(jpgPath);

            // Create DIRP handle
            var error_code = dirp_create_from_rjpeg(rjpegData, rjpegData.Length, out IntPtr handle);
            if (error_code != 0)
                throw new InvalidOperationException("Failed to create DIRP handle from R-JPEG:" + error_code);

            using (var safeHandle = new SafeDirpHandle { Handle = handle })
            {
                // Get image resolution
                if (dirp_get_rjpeg_resolution(handle, out dirp_resolution_t resolution) != 0)
                    throw new InvalidOperationException("Failed to get R-JPEG resolution.");

                int pixelCount = resolution.width * resolution.height;
                ushort[] rawData = new ushort[pixelCount];

                // Get raw radiometric data
                if (dirp_get_original_raw(handle, rawData, rawData.Length * sizeof(ushort)) != 0)
                    throw new InvalidOperationException("Failed to get original RAW data.");

                return rawData;
            }
        }
    }
}