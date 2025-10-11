// Copyright SkyComb Limited 2025. All rights reserved. 

using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessModel;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;


namespace SkyCombImage.PersistModel
{
    /// <summary>
    /// Represents a waypoint location of interest
    /// </summary>
    public class Waypoint
    {
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double AltitudeAgl { get; set; } // Altitude Above Ground Level in meters
        public double Speed { get; set; } = 5.0; // m/s
        public bool TakePicture { get; set; } = false;
        public double? CameraTilt { get; set; } = null; // degrees
        public double? UavYaw { get; set; } = null; // degrees relative to north
        public double? WaitTime { get; set; } = null; // seconds
        public int? WaypointNumber { get; set; } = null;
    }

    /// <summary>
    /// Exports waypoints to UGCS-compatible formats
    /// </summary>
    public static class UgcsWaypointExporter
    {
        /// <summary>
        /// Exports waypoints to CSV without headers (minimal format)
        /// Only includes: Latitude, Longitude, AltitudeAGL, Speed
        /// </summary>
        public static void ExportToCsvMinimal(List<Waypoint> waypoints, string filePath)
        {
            using (var writer = new StreamWriter(filePath, false, Encoding.UTF8))
            {
                foreach (var wp in waypoints)
                {
                    writer.WriteLine($"{wp.Latitude},{wp.Longitude},{wp.AltitudeAgl},{wp.Speed}");
                }
            }
        }

        /// <summary>
        /// Exports waypoints to CSV with headers (full format)
        /// Includes all optional fields
        /// </summary>
        public static void ExportToCsvWithHeaders(List<Waypoint> waypoints, string filePath)
        {
            using (var writer = new StreamWriter(filePath, false, Encoding.UTF8))
            {
                // Write header
                writer.WriteLine("Latitude,Longitude,AltitudeAGL,Speed,Picture,WP,CameraTilt,UavYaw,WaitTime");

                // Write data
                foreach (var wp in waypoints)
                {
                    var line = new StringBuilder();
                    line.Append($"{wp.Latitude},");
                    line.Append($"{wp.Longitude},");
                    line.Append($"{wp.AltitudeAgl},");
                    line.Append($"{wp.Speed},");
                    line.Append($"{(wp.TakePicture ? "TRUE" : "FALSE")},");
                    line.Append($"{wp.WaypointNumber?.ToString() ?? ""},");
                    line.Append($"{wp.CameraTilt?.ToString() ?? ""},");
                    line.Append($"{wp.UavYaw?.ToString() ?? ""},");
                    line.Append($"{wp.WaitTime?.ToString() ?? ""}");

                    writer.WriteLine(line.ToString());
                }
            }
        }

        /// <summary>
        /// Exports waypoints to UGCS JSON format (bare minimum)
        /// No hardcoded vehicle profiles or parameters - UGCS will handle defaults on import
        /// </summary>
        public static void ExportToJson(
            List<Waypoint> waypoints,
            string filePath,
            string routeName = "Interest Points")
        {
            var route = new MinimalUgcsRoute
            {
                Route = new MinimalRoute
                {
                    Name = routeName,
                    Segments = new List<Segment>()
                }
            };

            // Convert waypoints to segments
            foreach (var wp in waypoints)
            {
                route.Route.Segments.Add(new Segment
                {
                    Type = "Waypoint",
                    Point = new Point
                    {
                        Latitude = wp.Latitude,
                        Longitude = wp.Longitude,
                        Altitude = wp.AltitudeAgl,
                        AltitudeType = "AGL"
                    },
                    Parameters = new SegmentParameters
                    {
                        Speed = wp.Speed,
                        AltitudeType = "AGL"
                    }
                });
            }

            // Serialize to JSON with proper formatting
            var options = new JsonSerializerOptions
            {
                WriteIndented = true,
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase
            };

            var json = JsonSerializer.Serialize(route, options);
            File.WriteAllText(filePath, json, Encoding.UTF8);
        }
    }

    public class MinimalUgcsRoute
    {
        public MinimalRoute Route { get; set; }
    }

    public class MinimalRoute
    {
        public string Name { get; set; }
        public List<Segment> Segments { get; set; }
    }

    public class Segment
    {
        public string Type { get; set; }
        public Point Point { get; set; }
        public SegmentParameters Parameters { get; set; }
    }

    public class Point
    {
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double Altitude { get; set; }
        public string AltitudeType { get; set; }
    }

    public class SegmentParameters
    {
        public double Speed { get; set; }
        public string AltitudeType { get; set; }
    }

    // Save animal data to a datastore
    public class AnimalSave : DataStoreAccessor
    {
        public AnimalSave(DroneDataStore data) : base(data)
        {
        }

        // Create an animals tab and save the animal data
        public static void SaveDetail(ImageDataStore dataStore, AnimalModelList animals, string tabName)
        {
            (var newTab, var ws) = dataStore.SelectOrAddWorksheet(tabName);
            if (ws != null)
            {
                int row = 0;
                foreach (var animal in animals)
                    dataStore.SetDataListRowKeysAndValues(ref row, animal.GetSettings());

                dataStore.SetColumnWidth(1, 12);
                dataStore.SetColumnWidth(2, 10);
                dataStore.SetColumnWidth(3, 25);
                dataStore.SetColumnWidth(4, 18);
                dataStore.SetColumnWidth(5, 12);
                dataStore.SetColumnWidth(6, 12);
                dataStore.SetColumnWidth(7, 15);
                dataStore.SetColumnWidth(8, 12);
                dataStore.SetColumnWidth(9, 15);
                dataStore.SetColumnWidth(10, 15);
                dataStore.SetColumnWidth(11, 12);
                dataStore.SetColumnWidth(12, 12);
                dataStore.SetColumnWidth(13, 15);
            }
        }

        public static List<Waypoint> GetWaypoints(AnimalModelList animals, double altitudeAgl = 100, double speed = 5, double waitTime = 2)
        {
            List<Waypoint> waypoints = new();

            foreach (var animal in animals)
                if (animal.GlobalLocation != null)
                {
                    waypoints.Add(new Waypoint
                    {
                        Latitude = animal.GlobalLocation.Latitude,
                        Longitude = animal.GlobalLocation.Longitude,
                        AltitudeAgl = altitudeAgl, // Default altitude above ground level in meters
                        Speed = speed, // Default speed in m/s
                        TakePicture = true,
                        CameraTilt = 90.0, // Point camera straight down
                        UavYaw = null, // No specific yaw
                        WaitTime = waitTime, // Wait 2 seconds at waypoint
                        WaypointNumber = null // Let UGCS assign waypoint numbers
                    });
                }

            return waypoints;
        }
    }
}