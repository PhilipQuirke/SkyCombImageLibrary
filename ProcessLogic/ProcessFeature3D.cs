using SkyCombGround.CommonSpace;
using SkyCombGround.GroundModel;


namespace SkyCombImage.ProcessLogic
{
    public class TerrainGrid
    {
        private readonly GroundModel GroundModel;

        public TerrainGrid(GroundModel groundModel)
        {
            GroundModel = groundModel;
        }

        public float GetElevation(DroneLocation positionNE)
        {
            return GroundModel.GetElevationByDroneLocn(positionNE);
        }

        public float VerticalUnitM { get { return GroundModel.VerticalUnitM; } }
    }

    public class DroneState
    {
        public DroneLocation LocationNE { get; set; }  // Northing, Easting
        public float Altitude { get; set; } // Meters above sea level
        public float Yaw { get; set; } // Degrees, 0 = North, clockwise positive
        public float CameraDownAngle { get; set; } // Degrees from horizontal
    }

    public class CameraParameters
    {
        public float HorizontalFOV { get; set; } // Degrees
        public float VerticalFOV { get; set; } // Degrees
    }

    public class ImagePosition
    {
        public float HorizontalFraction { get; set; } // -1 to +1, left to right
        public float VerticalFraction { get; set; } // -1 to +1, bottom to top
    }

    public class LocationResult
    {
        public DroneLocation LocationNE { get; set; }
        public float Elevation { get; set; }
        public float Confidence { get; set; } // 0-1 scale
    }

    public class DroneTargetCalculator
    {
        private readonly TerrainGrid terrain;
        private readonly float minDroneHeight = 40; // Minimum height above terrain
        private readonly float maxSearchDistance = 1000; // Maximum search distance in meters
        private readonly float baseStepSize = 1; // Base step size for iteration

        public DroneTargetCalculator(TerrainGrid terrain)
        {
            this.terrain = terrain;
        }

        public LocationResult? CalculateTargetLocation(
            DroneState drone,
            CameraParameters camera,
            ImagePosition targetImage)
        {
            // Validate inputs
            ValidateInputs(drone, camera, targetImage);

            // Calculate viewing angles
            float targetDownAngle = CalculateDownAngle(drone.CameraDownAngle, camera.VerticalFOV, targetImage.VerticalFraction);
            float targetDirection = CalculateDirection(drone.Yaw, camera.HorizontalFOV, targetImage.HorizontalFraction);

            // Convert angles to radians for calculations
            float downAngleRad = targetDownAngle * (float)Math.PI / 180.0f;
            float directionRad = targetDirection * (float)Math.PI / 180.0f;

            return FindTerrainIntersection(drone, downAngleRad, directionRad);
        }

        private void ValidateInputs(DroneState drone, CameraParameters camera, ImagePosition targetImage)
        {
            if (drone.Altitude < minDroneHeight)
                throw new ArgumentException("Drone altitude is below minimum safe height");

            if (targetImage.HorizontalFraction < -1 || targetImage.HorizontalFraction > 1 ||
                targetImage.VerticalFraction < -1 || targetImage.VerticalFraction > 1)
                throw new ArgumentException("Image position fractions must be between -1 and 1");
        }

        private float CalculateDownAngle(float cameraAngle, float vFov, float verticalFraction)
        {
            // Convert FOV to radians
            float halfVFovRad = vFov * 0.5f * (float)Math.PI / 180.0f;

            // Calculate the actual angle from the center using tangent relationship
            float angleFromCenterRad = (float)Math.Atan(verticalFraction * Math.Tan(halfVFovRad));

            // Convert back to degrees and add to camera angle
            return cameraAngle - angleFromCenterRad * 180.0f / (float)Math.PI;
        }

        private float CalculateDirection(float yaw, float hFov, float horizontalFraction)
        {
            // Convert FOV to radians
            float halfHFovRad = hFov * 0.5f * (float)Math.PI / 180.0f;

            // Calculate the actual angle from the center using tangent relationship
            float angleFromCenterRad = (float)Math.Atan(horizontalFraction * Math.Tan(halfHFovRad));

            // Convert back to degrees and add to yaw
            float direction = yaw - angleFromCenterRad * 180.0f / (float)Math.PI;

            // Normalize to 0-360 degrees
            return ((direction % 360) + 360) % 360;
        }


        private LocationResult? FindTerrainIntersection(DroneState drone, float downAngleRad, float directionRad)
        {
            DroneLocation currentPosition = drone.LocationNE;
            float currentAltitude = drone.Altitude;
            float totalDistance = 0;
            float confidence = 0;
            float currTerrainHeight = 0;
            int num_steps = 0;

            // Calculate initial height above terrain
            float terrainHeightAtStart = terrain.GetElevation(currentPosition);
            float heightAboveTerrain = currentAltitude - terrainHeightAtStart;

            // Pre-calculate trig values for efficiency
            float tanDownAngle = (float)Math.Tan(downAngleRad);
            DroneLocation directionVector = new(
                (float)Math.Sin(directionRad),  // East component
                (float)Math.Cos(directionRad)   // North component
            );

            // Initial step size based on height
            float horizontalStepSize = CalculateAdaptiveStepSize(heightAboveTerrain);

            while (totalDistance < maxSearchDistance)
            {
                num_steps++;

                // Calculate vertical drop for this step using tangent
                float verticalStep = horizontalStepSize * tanDownAngle;

                // Update position and height
                currentPosition = currentPosition.Add( directionVector.Multiply(horizontalStepSize) );
                currentAltitude -= verticalStep;
                totalDistance += horizontalStepSize;

                // Get terrain height at current position
                float terrainHeight = terrain.GetElevation(currentPosition);
                if (terrainHeight < 0)
                {
                    if (num_steps >= 10)
                        // Return the rough position. This is the last step that was within the terrain grid
                        return new LocationResult
                        {
                            LocationNE = currentPosition,
                            Elevation = currTerrainHeight,
                            Confidence = 0
                        };

                    return null; // Position outside terrain grid
                }
                currTerrainHeight = terrainHeight;

                // Check for intersection
                if (currentAltitude + terrain.VerticalUnitM <= terrainHeight)
                {
                    // Interpolate to find more precise intersection point
                    float overshoot = terrainHeight - currentAltitude;
                    float stepBack = overshoot / verticalStep * horizontalStepSize;
                    currentPosition = currentPosition.Add( directionVector.Multiply(-stepBack) );

                    // Recalculate final elevation
                    terrainHeight = terrain.GetElevation(currentPosition);

                    // Calculate confidence based on distance
                    confidence = CalculateConfidence(totalDistance, drone.Altitude);

                    return new LocationResult
                    {
                        LocationNE = currentPosition,
                        Elevation = terrainHeight,
                        Confidence = confidence
                    };
                }

                // Update step size based on current height above terrain
                heightAboveTerrain = currentAltitude - terrainHeight;
                horizontalStepSize = CalculateAdaptiveStepSize(heightAboveTerrain);
            }

            // No intersection found within max distance
            return new LocationResult
            {
                LocationNE = currentPosition,
                Elevation = currTerrainHeight,
                Confidence = 0
            };
        }


        private float CalculateAdaptiveStepSize(float heightAboveTerrain)
        {
            // Smaller steps when closer to ground, larger steps when higher
            return Math.Max(terrain.VerticalUnitM, Math.Min(baseStepSize * (heightAboveTerrain / 50.0f), 5.0f));
        }

        private float CalculateConfidence(float distance, float initialHeight)
        {
            // Confidence decreases with distance and initial height
            float distanceFactor = 1.0f - (distance / maxSearchDistance);
            float heightFactor = 1.0f - (Math.Min(initialHeight, 300) / 300);
            return distanceFactor * 0.7f + heightFactor * 0.3f;
        }
    }
}
