using System.Runtime.InteropServices;

namespace SkyCombImageLibrary.ProcessLogic.DJI
{
    /// <summary>
    /// Native P/Invoke declarations for DJI Thermal SDK
    /// </summary>
    public static class DjiThermalApi
    {
        private const string DllName = "libdirp.dll";

        #region Enums and Constants

        public enum DirpRetCode
        {
            DIRP_SUCCESS = 0,
            DIRP_ERROR_MALLOC = -1,
            DIRP_ERROR_POINTER_NULL = -2,
            DIRP_ERROR_INVALID_PARAMS = -3,
            DIRP_ERROR_INVALID_RAW = -4,
            DIRP_ERROR_INVALID_HEADER = -5,
            DIRP_ERROR_INVALID_CURVE = -6,
            DIRP_ERROR_RJPEG_PARSE = -7,
            DIRP_ERROR_SIZE = -8,
            DIRP_ERROR_INVALID_HANDLE = -9,
            DIRP_ERROR_FORMAT_INPUT = -10,
            DIRP_ERROR_FORMAT_OUTPUT = -11,
        }

        public enum DirpPseudoColor
        {
            DIRP_PSEUDO_COLOR_WHITEHOT = 0,
            DIRP_PSEUDO_COLOR_FULGURITE = 1,
            DIRP_PSEUDO_COLOR_IRONRED = 2,
            DIRP_PSEUDO_COLOR_HOTIRON = 3,
            DIRP_PSEUDO_COLOR_MEDICAL = 4,
            DIRP_PSEUDO_COLOR_ARCTIC = 5,
            DIRP_PSEUDO_COLOR_RAINBOW1 = 6,
            DIRP_PSEUDO_COLOR_RAINBOW2 = 7,
            DIRP_PSEUDO_COLOR_TINT = 8,
            DIRP_PSEUDO_COLOR_BLACKHOT = 9,
        }

        public enum DirpMeasurementParamsType
        {
            DIRP_MEASUREMENT_PARAMS_TYPE_DISTANCE = 0,
            DIRP_MEASUREMENT_PARAMS_TYPE_HUMIDITY = 1,
            DIRP_MEASUREMENT_PARAMS_TYPE_EMISSIVITY = 2,
            DIRP_MEASUREMENT_PARAMS_TYPE_REFLECTION = 3,
        }

        #endregion

        #region Structures

        [StructLayout(LayoutKind.Sequential)]
        public struct DirpResolution
        {
            public int Width;
            public int Height;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct DirpMeasurementParams
        {
            public float Distance;      // meters
            public float Humidity;      // percentage (0-100)
            public float Emissivity;    // 0.01 to 1.0
            public float Reflection;    // Celsius
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct DirpColorBar
        {
            public short High;          // Temperature * 10
            public short Low;           // Temperature * 10
            public int ManualEnable;    // 0 = auto, 1 = manual
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct DirpApiVersion
        {
            public ushort magic;      // Magic number
            public byte major;        // Major version
            public byte minor;        // Minor version  
            public byte revision;     // Revision
            public byte reserve0;
            public ushort reserve1;
        }
        #endregion

        #region Core API Functions

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_get_api_version(ref DirpApiVersion version);


        /// <summary>
        /// Create DIRP handle from R-JPEG data
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_create_from_rjpeg(
            byte[] data,
            int size,
            ref IntPtr handle);

        /// <summary>
        /// Destroy DIRP handle and free resources
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_destroy(IntPtr handle);

        /// <summary>
        /// Get image resolution
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_get_rjpeg_resolution(
            IntPtr handle,
            ref DirpResolution resolution);

        /// <summary>
        /// Get measurement parameters (distance, humidity, emissivity, reflection)
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_get_measurement_params(
            IntPtr handle,
            ref DirpMeasurementParams parameters);

        /// <summary>
        /// Get color bar settings
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_get_color_bar(
            IntPtr handle,
            ref DirpColorBar colorBar);

        /// <summary>
        /// Extract temperature data from R-JPEG as float array (Celsius)
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_measure(
            IntPtr handle,
            float[] temp_image,
            int size);

        /// <summary>
        /// Get single point temperature at specific coordinates
        /// </summary>
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern DirpRetCode dirp_get_measurement_params_range(
            IntPtr handle,
            DirpMeasurementParamsType type,
            ref float min,
            ref float max);

        #endregion
    }
}