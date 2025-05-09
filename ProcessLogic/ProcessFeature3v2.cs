// Copyright SkyComb Limited 2025. All rights reserved.
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundModel;
using SkyCombImage.ProcessModel;
using System.Diagnostics;
using System.Numerics;

/*
SkyComb calculates the location of a "pest animal" object in 3D space using data provided by a drone carrying a thermal camera. 
This code uses dead-reckoning on a single observation from the drone to estimate the location of the animal.
It assumes the animal is on the ground.
It uses a terrian altitude model where the datums are in 1m square grid with a Altitude resolution of 0.2m 
We assume the drone is always at least 40m above the terrian (enforced by the drone operators desire to not crash their drone).
We exclude any animals that are located more than 1000m horizontally from the drone location. The normal locaction range is 0 to 100m
We assume a specific drone camera "intrinsic matrix" K specific to the drone camera that recorded the video images.

The code uses this input data:
- Drone Location(Northing, Easting)(Metres)
- Drone Altitude(Metres)
- Drone camera direction pointed (yaw, in degrees). When not cornering, the camera points in roughly same direction drone is travelling.
- Drone camera down angle (pitch, in degrees)

- Camera image H FOV (horizontal field of vision, degrees, static)
- Camera image V FOV (vertical field of vision, degrees, static)
- Camera image height in pixels
- Camera image width in pixels

- Object position in Camera image H position in pixels
- Object position in Camera image V position in pixels

- Terrain model (Northing, Easting) => Altitude (M) 

The code outputs a LocationResult class object
*/


namespace SkyCombImage.ProcessLogic
{
    public class TerrainGrid
    {
        public float VerticalUnitM = 0.2f;

        private readonly GroundModel GroundModel;

        public TerrainGrid(GroundModel groundModel)
        {
            GroundModel = groundModel;
        }

        public float GetElevation(float easting, float northing)
        {
            DroneLocation positionNE = new(northing, easting);
            return GetElevation(positionNE);
        }

        public float GetElevation(DroneLocation positionNE)
        {
            return GroundModel.GetElevationByDroneLocn(positionNE);
        }
    }

    public class DroneState
    {
        public DroneLocation LocationNE { get; set; }  // Northing, Easting
        public float Altitude { get; set; } // Meters above sea level
        public float Yaw { get; set; } // Degrees, 0 = North, clockwise positive
        public float CameraDownAngle { get; set; } // Degrees down from horizontal, only values in range +30 to +92 are valid.
    }

    public class CameraParameters
    {
        public double FocalLength { get; set; } // mm
        public int ImageWidth { get; set; }  // Pixels
        public int ImageHeight { get; set; } // Pixels
        public double SensorWidth { get; set; }  // mm
        public double SensorHeight { get; set; } // mm
        public float HorizontalFOV { get; set; } // Degrees
        public float VerticalFOV { get; set; } // Degrees
    }

    // Position of an (animal) hotspot in the image
    public class ImagePosition
    {
        public double PixelX { get; set; } // Image X position in pixels
        public double PixelY { get; set; } // Image Y position in pixels
    }

    public class LocationResult
    {
        public DroneLocation LocationNE { get; set; }
        public float Elevation { get; set; } // Object meters above sea level. Also DEM (Digital Ground Model) height
        public float Confidence { get; set; } // 0-1 scale
    }

    public class DroneTargetCalculator
    {
        public readonly Accord.Math.Matrix3x3 CameraK;
        public readonly DroneState DroneState;
        public readonly CameraParameters CameraParams;
        public readonly TerrainGrid Terrain;
        public readonly bool ApplyDistortionCorrection;


        public DroneTargetCalculator(DroneState droneState, CameraParameters cameraParams, TerrainGrid terrain, bool applyDistortionCorrection)
        {
            DroneState = droneState;
            CameraParams = cameraParams;
            Terrain = terrain;
            ApplyDistortionCorrection = applyDistortionCorrection;
            CameraK = CameraIntrinsic.Intrinsic(cameraParams.FocalLength, cameraParams.ImageWidth, cameraParams.ImageHeight, cameraParams.SensorWidth, cameraParams.SensorHeight);
        }


        // The vector origin represents the drone's world position in a coordinate system where:
        // X-component(origin.X) → Easting(Meters) : This represents the drone's position in the east-west direction.
        // Y-component(origin.Y) → Altitude(Meters Above Sea Level) : This represents the drone's height above the terrain.
        // Z-component(origin.Z) → Northing(Meters) : This represents the drone's position in the north-south direction.
        public Vector3 DroneWorldPosition()
        {
            return new Vector3(DroneState.LocationNE.EastingM, DroneState.Altitude, DroneState.LocationNE.NorthingM);
        }


        /// <summary>
        /// Converts pixel coordinates in the camera image to a normalized direction vector in **camera space**.
        /// In **camera space**:
        /// - The **X-axis** points **right**.
        /// - The **Y-axis** points **down**.
        /// - The **Z-axis** points **forward** (out of the camera).
        /// The returned vector is normalized.
        /// </summary>
        public Vector3 ComputeCameraDirection(double pixelX, double pixelY)
        {
            // Start with the given pixel coordinates.
            double px = pixelX;
            double py = pixelY;
            if (ApplyDistortionCorrection)
            {
                // Normalize pixel to camera coordinates using intrinsics.
                double normX = (pixelX - CameraK.V02) / CameraK.V00;  // (u - cx) / fx
                double normY = (pixelY - CameraK.V12) / CameraK.V11;  // (v - cy) / fy

                // Apply radial distortion correction using calibrated coefficients (k1, k2, etc.).
                double r2 = normX * normX + normY * normY;
                double k1 = 0.1f, k2 = 0.1f;  // example distortion coefficients. TODO set from calibration.
                double radialFactor = 1 + k1 * r2 + k2 * r2 * r2;
                double undistX = normX * radialFactor;
                double undistY = normY * radialFactor;
                // Convert undistorted normalized coords back to pixel coordinates.
                px = undistX * CameraK.V00 + CameraK.V02;
                py = undistY * CameraK.V11 + CameraK.V12;
            }
            // Compute direction in camera coordinates (pinhole model) from pixel.
            double x_cam = (px - CameraK.V02) / CameraK.V00; // (px - cx) / fx
            double y_cam = -(py - CameraK.V12) / CameraK.V11; // - (py - cy) / fy
            double z_cam = 1.0;  // assume a point on the image plane at z=1
            Vector3 camDir = new Vector3((float)x_cam, (float)y_cam, (float)z_cam);
            return Vector3.Normalize(camDir);
        }


        /// <summary>
        /// Transforms a direction vector from **camera space** to **world space**.
        /// In **world space**:
        /// - The **X-component** represents **Easting** (meters, left to right).
        /// - The **Y-component** represents **Altitude** (meters above sea level).
        /// - The **Z-component** represents **Northing** (meters, forward/backward).
        /// The function applies the drone's **yaw** and **pitch** to orient the camera-relative vector correctly in the world.
        /// The returned vector is normalized.
        /// </summary>
        public Vector3 CameraToWorldDirection(Vector3 camDir)
        {
            // For a gimbaled camera:
            // 1. The camera is oriented independently of the drone body
            // 2. The yaw represents the camera's heading (0 = North, clockwise positive)
            // 3. The pitch represents how far down the camera is pointing from horizontal

            // Convert angles to radians
            float yawRad = DroneState.Yaw * (MathF.PI / 180f);
            float pitchRad = DroneState.CameraDownAngle * (MathF.PI / 180f);

            // Create rotation matrices for yaw and pitch
            Matrix4x4 yawMatrix = Matrix4x4.CreateRotationY(yawRad);
            Matrix4x4 pitchMatrix = Matrix4x4.CreateRotationX(pitchRad);
            Matrix4x4 combinedRotation = Matrix4x4.Multiply(pitchMatrix, yawMatrix); // PQR

            // In camera space: 
            // X is right, Y is down, Z is forward
            // We need to transform this to world space

            // Convert camera direction to world direction
            Vector3 worldDir = Vector3.Transform(camDir, combinedRotation);

            return Vector3.Normalize(worldDir);
        }


        /// <summary>
        /// Casts a ray from the drone’s position in a given direction and finds the intersection with the terrain.
        /// The function steps along the ray in small increments (1m) until:
        /// - The ray reaches or crosses the ground.
        /// - The search distance exceeds the maximum allowed range (1000m).
        /// If an intersection is detected, it refines the hit position using **linear interpolation** for better accuracy.
        ///
        /// **Parameters:**
        /// - `origin` (Vector3): The starting position of the ray (drone's world position).
        /// - `direction` (Vector3): The direction in which the ray is cast (normalized).
        /// - `terrain` (TerrainGrid): The terrain model used for altitude lookup.
        ///
        /// **Returns:**
        /// - `Vector3?`: The world coordinates of the intersection point if found, otherwise `null`.
        ///
        /// **Edge Cases Handled:**
        /// - If the ray is pointing **upward**, no intersection is found (returns `null`).
        /// - If the drone is looking **straight down**, the function directly returns the terrain altitude.
        /// - If the ray is **nearly horizontal**, it may return `null` if the ground is never reached.
        /// - If the search distance exceeds **1000m**, it stops and returns `null`.
        /// </summary>
        public Vector3? Raycast(Vector3 origin, Vector3 direction)
        {
            const float stepSize = 1.0f; // Step size in meters
            const float maxDistance = 1000.0f; // Max search range
            float distanceTraveled = 0.0f;

            Vector3 currentPos = origin;
            float currentAltitude = origin.Y;
            float terrainAltitude = Terrain.GetElevation(currentPos.X, currentPos.Z);

            // If the ray is pointing upwards, it will never hit the ground.
            if (direction.Y > 0)
                return null;

            // Step along the ray in increments until we reach or cross the ground
            while (distanceTraveled < maxDistance)
            {
                if (currentAltitude <= terrainAltitude)
                {
                    // We have crossed the ground -> Perform linear interpolation for better accuracy
                    float previousAltitude = currentAltitude + stepSize * direction.Y;
                    float previousDistance = distanceTraveled - stepSize;

                    if (previousAltitude == terrainAltitude)
                    {
                        // Direct hit
                        return currentPos;
                    }
                    else
                    {
                        // Linear interpolation to estimate more accurate intersection
                        float t = (terrainAltitude - previousAltitude) / (currentAltitude - previousAltitude);
                        float refinedDistance = previousDistance + t * stepSize;
                        Vector3 refinedPos = origin + direction * refinedDistance;
                        return refinedPos;
                    }
                }

                // Move forward along the ray
                distanceTraveled += stepSize;
                currentPos += direction * stepSize;
                currentAltitude = currentPos.Y;
                terrainAltitude = Terrain.GetElevation(currentPos.X, currentPos.Z);
            }

            // If we exceed max range without hitting the ground, return null
            return null;
        }


        /// <summary>
        /// Casts a ray from the **drone's world position** in a given direction and finds the intersection with the terrain.
        /// The drone's position is represented as a **world space vector**:
        /// - `origin.X` → **Easting** (meters, left/right)
        /// - `origin.Y` → **Altitude** (meters above sea level)
        /// - `origin.Z` → **Northing** (meters, forward/backward)
        /// The function checks if the ray intersects the terrain and returns the ground position if found.
        /// </summary>
        public Vector3? FindGroundIntersection(Vector3 worldDir)
        {
            Vector3 origin = DroneWorldPosition();

            // Avoid divide-by-zero or infinite intersections for horizontal/upward rays.
            if (Math.Abs(worldDir.Y) < 1e-6f)
                // Ray is almost parallel to the ground.
                return null;

            if (worldDir.Y > 0)
                // Ray is pointing upward (above the horizon), so it will not hit the ground.
                return null;

            // Find intersection with terrain:
            if (Terrain != null)
                return Raycast(origin, worldDir);

            // Assume flat ground at y = 0 (or known ground height).
            float groundY = 0.0f;
            float t = (groundY - origin.Y) / worldDir.Y;
            if (t < 0)
                // No valid intersection (drone below ground or ray pointing wrong direction).
                return null;

            return origin + worldDir * t;
        }


        /// <summary>
        /// Estimates the real-world location of an object detected in a drone's thermal camera image
        /// using **dead reckoning**. It projects the object's image position into 3D world space,
        /// casts a ray from the drone's position, and finds where it intersects the ground.
        ///
        /// **Algorithm Steps:**
        /// 1. Converts the object's pixel position in the image into a **normalized camera-space direction**.
        /// 2. Transforms this camera-space direction into **world-space** using the drone's yaw and pitch.
        /// 3. Casts a ray from the drone along this direction and finds the ground intersection.
        /// 4. Checks if the estimated position is **within the valid range (≤1000m horizontal distance)**.
        /// 5. Computes a **confidence score** based on sensitivity to small input changes.
        ///
        /// **Parameters:**
        /// - `imagePosition` (ImagePosition): The object's location in the image (normalized -1 to +1).
        /// - `camera` (CameraParameters): The camera's horizontal and vertical field of view (FOV).
        /// - `ApplyDistortionCorrection`: (bool) Recommend false 
        ///         Tests on CC\2024-04-D videos on 4/5Apr25 show ~9% difference between true and false, with false better.
        ///         The difference in calculated location is very small (~10cm)
        /// - `Terrain` (TerrainGrid): The terrain model, used to determine ground elevation.
        ///
        /// **Returns:**
        /// - `LocationResult?`: The estimated **real-world location** of the detected object if valid, otherwise `null`.
        ///
        /// **Meaning of the Components in the Returned `LocationResult`:**
        /// - `LocationNE` (DroneLocation):
        ///   - NorthingM → Horizontal distance north or south.
        ///   - EastingE → Horizontal distance east or west.
        /// - `Elevation` (float): The estimated **terrain altitude** at the computed object location.
        /// - `Confidence` (float, 0 to 1): A measure of how **sensitive the calculation is** to small changes in inputs.
        ///
        /// **Edge Cases Handled:**
        /// - If the target location is **>1000m away** horizontally, the function returns `null`.
        /// - If the camera **points straight down (90° pitch)**, the function directly estimates the target below the drone.
        /// - If the camera angle is **almost horizontal**, the function prevents unstable calculations.
        /// - If lens distortion is a factor, the function allows separate handling for distortion correction.
        ///
        /// </summary>
        public LocationResult? CalculateTargetLocation(ImagePosition imagePosition)
        {
            double px = imagePosition.PixelX;
            double py = imagePosition.PixelY;

            // **Step 1 :** Convert image position to camera direction.
            Vector3 camDir = ComputeCameraDirection(px, py);

            // **Step 2:** Convert camera direction to world direction.
            Vector3 worldDir = CameraToWorldDirection(camDir);

            // **Step 3:** Raycast from the drone along this direction to find ground intersection.
            Vector3? targetLocation = FindGroundIntersection(worldDir);

            // **Step 4:** Handle edge cases for camera orientation.
            if (Math.Abs(DroneState.CameraDownAngle - 90.0f) < 1e-3f)
            {
                // If camera is pointing straight down (pitch ~ 90°), set target directly below drone.
                Vector3 dronePos2 = DroneWorldPosition();
                float groundY = Terrain?.GetElevation(dronePos2.X, dronePos2.Z) ?? 0.0f;
                targetLocation = new Vector3(dronePos2.X, groundY, dronePos2.Z);
            }

            // (If the ray is nearly horizontal, FindGroundIntersection will return null or a very distant point,
            // which is handled by the range check next.)

            // **Step 5:** Validate the estimated position range (within 1000 m horizontally from the drone).
            if (!targetLocation.HasValue)
                // No intersection found (e.g., ray parallel to ground or pointing upward).
                return null;

            Vector3 target = targetLocation.Value;
            Vector3 dronePos = DroneWorldPosition();
            // Horizontal distance (ignore altitude difference).
            float horizontalDist = new Vector3(target.X - dronePos.X, 0, target.Z - dronePos.Z).Length();
            if (horizontalDist > 1000.0f)
                // If beyond 1000m, treat as invalid detection.
                return null;

            // **Step 6:** Compute a confidence metric based on sensitivity to small input changes.
            float confidence = 0.0f;
            if (targetLocation.HasValue)
            {
                Vector3 baseTarget = targetLocation.Value;
                float deltaPixel = 1.0f;  // small change in pixels for sensitivity test

                // Recompute target location for a small change in the image X coordinate.
                Vector3 camDir_dx = ComputeCameraDirection(px + deltaPixel, py);
                Vector3 worldDir_dx = CameraToWorldDirection(camDir_dx);
                Vector3? target_dx = FindGroundIntersection(worldDir_dx);

                // Recompute target location for a small change in the image Y coordinate.
                Vector3 camDir_dy = ComputeCameraDirection(px, py + deltaPixel);
                Vector3 worldDir_dy = CameraToWorldDirection(camDir_dy);
                Vector3? target_dy = FindGroundIntersection(worldDir_dy);

                // If an offset ray didn't produce a valid target (out of range), confidence is remains zero
                if (target_dx.HasValue && target_dy.HasValue)
                {
                    // Calculate average shift caused by the 1-pixel change.
                    float shiftX = (target_dx.Value - baseTarget).Length();
                    float shiftY = (target_dy.Value - baseTarget).Length();
                    float avgShift = (shiftX + shiftY) / 2.0f;
                    // Confidence inversely proportional to shift: large shift -> low confidence, small shift -> high confidence.
                    confidence = 1.0f / (1.0f + avgShift);
                    confidence = Math.Clamp(confidence, 0, 1);
                }
            }

            return new LocationResult()
            {
                LocationNE = new DroneLocation(targetLocation.Value.Z, targetLocation.Value.X),
                Elevation = targetLocation.Value.Y,
                Confidence = confidence,
            };
        }


#if DEBUG
        // Unit test: With camera almost straight down, an object near the image center
        // should be in roughly same location as the drone, and at same DEM as Block
        public void UnitTest_Centroid(ProcessBlock block)
        {
            // Move object to very near the centre of the image
            ImagePosition imagePosition = new();
            imagePosition.PixelX = CameraParams.ImageWidth / 2.0 - 1;
            imagePosition.PixelY = CameraParams.ImageHeight / 2.0 - 1;

            // Set camera to straight down 
            var old_value = DroneState.CameraDownAngle;
            DroneState.CameraDownAngle = 89.9f;


            LocationResult? result = CalculateTargetLocation(imagePosition);
            if (result != null)
            {
                var locnDiff = block.DroneLocnM.Subtract(result.LocationNE);
                var locnDist = locnDiff.DiagonalM;

                var heightDiff = Math.Abs(block.FlightStep.DemM - result.Elevation);

                if (locnDist > 2)
                {
                    Debug.Print(block.DroneLocnM.ToString(), result.LocationNE.ToString(), locnDist);
                    BaseConstants.Assert(false, "ProcessFeature3v2 bad location");
                }

                if (heightDiff > 4)
                {
                    LocationResult? result2 = CalculateTargetLocation(imagePosition);

                    Debug.Print(block.FlightStep.DemM.ToString(), result.Elevation, heightDiff);
                    BaseConstants.Assert(false, "ProcessFeature3v2 bad height");
                }
            }

            DroneState.CameraDownAngle = old_value;
        }

#endif
    }
}
