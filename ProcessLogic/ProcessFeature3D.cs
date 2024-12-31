using SkyCombGround.CommonSpace;
using SkyCombGround.GroundModel;


/*
SkyComb calculates the location of a "pest animal" object in 3D space using data provided by a drone carrying a thermal camera. 

The code is based on:
- Drone Location(Northing, Easting)(Metres)
- Drone Altitude(Metres)
- Drone camera direction pointed (yaw, in degrees). When not cornering, the camera points in roughly same direction drone is travelling.
- Drone camera down angle (pitch, in degrees)

- Camera image H FOV (horizontal field of vision, degrees, static)
- Camera image V FOV (vertical field of vision, degrees, static)

- Object position in Camera image H fraction (unitless, left -1 to right +1) 
- Object position in Camera image V fraction (unitless, bottom -1 to top +1) 

- Terrain model (Northing, Easting) => Altitude (M)   

The code process is:
*Drone to Object down angle = 
   * Drone camera down angle - 0.5 * Camera image V FOV * Object V fraction 
* Drone to Object direction = 
   * Drone camera direction pointed - 0.5 * Camera image H FOV * Object H fraction 
* Calculate intercept from camera to terrain as:
   *Set CurrLocation to Drone Location
   * Set CurrAltitude to Drone Altitude
   * If Drone to Object down angle < 70 degrees
      * Move CurrLocation in 1m in “Drone to object” direction 
      * Decrease CurrAltitude by 1m * tan(Drone to Object down angle)
      * If Terrain(CurrLocation).CurrAltitude < CurrAltitude halt.
   * Else
      * Decrease CurrAltitude by 1m
      * Move CurrLocation in 1m / tan(Drone to Object down angle) in “Drone to object” direction 
      * If Terrain(CurrLocation).CurrAltitude < CurrAltitude halt.
*/


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
            // Convert angle back to degrees to compare easily
            float downAngleDeg = downAngleRad * 180.0f / (float)Math.PI;

            DroneLocation currentPosition = drone.LocationNE;
            float currentAltitude = drone.Altitude;
            float totalDistance = 0;
            float confidence = 0;
            float currTerrainHeight = 0;
            int numSteps = 0;

            // Calculate initial height above terrain
            float terrainHeightAtStart = terrain.GetElevation(currentPosition);
            float heightAboveTerrain = currentAltitude - terrainHeightAtStart;

            DroneLocation directionVector = new(
                (float)Math.Sin(directionRad),  // East component
                (float)Math.Cos(directionRad)   // North component
            );

            // Initial step size based on height
            float horizontalStepSize = CalculateAdaptiveStepSize(heightAboveTerrain);

            // Check if we should use near-vertical logic
            bool isNearVertical = (downAngleDeg > 85); 

            if (!isNearVertical)
            {
                //
                // -----------------
                // Normal Approach
                // -----------------
                //
                float tanDownAngle = (float)Math.Tan(downAngleRad);
                while (totalDistance < maxSearchDistance)
                {
                    numSteps++;

                    // Move horizontally, drop altitude by tan(angle) * horizontalStep
                    float verticalStep = horizontalStepSize * tanDownAngle;

                    currentPosition = currentPosition.Add(directionVector.Multiply(horizontalStepSize));
                    currentAltitude -= verticalStep;
                    totalDistance += horizontalStepSize;

                    float terrainHeight = terrain.GetElevation(currentPosition);
                    if (terrainHeight < 0)
                    {
                        // If out of bounds, bail out
                        if (numSteps >= 10)
                        {
                            return new LocationResult
                            {
                                LocationNE = currentPosition,
                                Elevation = currTerrainHeight,
                                Confidence = 0
                            };
                        }
                        return null;
                    }
                    currTerrainHeight = terrainHeight;

                    if (currentAltitude + terrain.VerticalUnitM <= terrainHeight)
                    {
                        // Interpolate back
                        float overshoot = terrainHeight - currentAltitude;
                        float stepBack = overshoot / verticalStep * horizontalStepSize;
                        currentPosition = currentPosition.Add(directionVector.Multiply(-stepBack));

                        terrainHeight = terrain.GetElevation(currentPosition);

                        confidence = CalculateConfidence(totalDistance, drone.Altitude);
                        return new LocationResult
                        {
                            LocationNE = currentPosition,
                            Elevation = terrainHeight,
                            Confidence = confidence
                        };
                    }

                    // Recompute step size based on current height above terrain
                    heightAboveTerrain = currentAltitude - terrainHeight;
                    horizontalStepSize = CalculateAdaptiveStepSize(heightAboveTerrain);
                }

                // Max distance reached with no intersection
                return new LocationResult
                {
                    LocationNE = currentPosition,
                    Elevation = currTerrainHeight,
                    Confidence = 0
                };
            }
            else
            {
                //
                // ----------------------------------------------
                // Near-Vertical Approach
                // ----------------------------------------------
                //
                // Here, we rely mostly on decreasing altitude in small steps and only 
                // moving horizontally enough to keep things stable.

                // Example: take small vertical steps, compute horizontal offset from tan
                // The smaller the step, the less "blow-up" from tan(near 90°).
                float verticalStepSize = 1.0f;  // or terrain.VerticalUnitM, etc.

                // if downAngleDeg is exactly 90°, tanDownAngle would be infinite
                // so we clamp to a safe "maxTan" if angle is above 89° or so:
                float safeTan = (float)Math.Tan(Math.Min(downAngleRad, 1.55334f));
                // 1.55334 rad ~ 89 degrees, to prevent infinite blow-up

                while (totalDistance < maxSearchDistance)
                {
                    numSteps++;

                    // Step down in altitude by a small amount
                    currentAltitude -= verticalStepSize;

                    // Horizontal movement: 
                    float horizontalStep = verticalStepSize / safeTan;

                    currentPosition = currentPosition.Add(directionVector.Multiply(horizontalStep));
                    totalDistance += horizontalStep;

                    float terrainHeight = terrain.GetElevation(currentPosition);
                    if (terrainHeight < 0)
                    {
                        if (numSteps >= 10)
                        {
                            return new LocationResult
                            {
                                LocationNE = currentPosition,
                                Elevation = currTerrainHeight,
                                Confidence = 0
                            };
                        }
                        return null;
                    }
                    currTerrainHeight = terrainHeight;

                    // Check for intersection
                    if (currentAltitude + terrain.VerticalUnitM <= terrainHeight)
                    {
                        // Interpolate back to find exact intersection
                        float overshoot = terrainHeight - currentAltitude;
                        float stepBackFrac = overshoot / verticalStepSize; // fraction of the vertical step
                                                                           // horizontal step back
                        float backDist = horizontalStep * stepBackFrac;

                        currentPosition = currentPosition.Add(directionVector.Multiply(-backDist));
                        terrainHeight = terrain.GetElevation(currentPosition);

                        confidence = CalculateConfidence(totalDistance, drone.Altitude);

                        return new LocationResult
                        {
                            LocationNE = currentPosition,
                            Elevation = terrainHeight,
                            Confidence = confidence
                        };
                    }

                    // Update step size if we want finer stepping 
                    heightAboveTerrain = currentAltitude - terrainHeight;
                    if (heightAboveTerrain < 10f)
                    {
                        // Take smaller vertical steps if close to ground
                        verticalStepSize = 0.2f;
                    }
                    else if (heightAboveTerrain < 50f)
                    {
                        verticalStepSize = 0.5f;
                    }
                    else
                    {
                        verticalStepSize = 1.0f;
                    }
                }

                // If no intersection up to maxSearchDistance
                return new LocationResult
                {
                    LocationNE = currentPosition,
                    Elevation = currTerrainHeight,
                    Confidence = 0
                };
            }
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
