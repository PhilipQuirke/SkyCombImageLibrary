// Copyright SkyComb Limited 2025. All rights reserved.
using SkyCombGround.CommonSpace;


/*
SkyComb calculates the location of a "pest animal" object in 3D space using data provided by a drone carrying a thermal camera. 

The code uses this input data:
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
* Drone to Object down angle = 
   * Drone camera down angle - 0.5 * Camera image V FOV * Object V fraction 
* Drone to Object direction = 
   * Drone camera direction pointed - 0.5 * Camera image H FOV * Object H fraction 
* Calculate intercept from camera to terrain as:
   * Set CurrLocation to Drone Location
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
    public class DroneTargetCalculatorV1
    {
        private readonly float MinDroneHeight = 40; // Minimum height above terrain
        private readonly float MaxSearchDistanceM = 1000; // Maximum search distance in meters
        private readonly float BaseStepSizeM = 1; // Base step size for iteration

        private readonly DroneState DroneState;
        private readonly CameraParameters CameraParams;

        public DroneTargetCalculatorV1(DroneState droneState, CameraParameters cameraParams)
        {
            DroneState = droneState;
            CameraParams = cameraParams;
        }

        public LocationResult? CalculateTargetLocation(ImagePosition targetImage, bool applyDistortionCorrection, TerrainGrid terrain)
        {
            ValidateInputs(targetImage);

            // Calculate viewing angles
            var targetDownAngle = CalculateDownAngle(DroneState.CameraDownAngle, CameraParams.VerticalFOV, targetImage.VerticalFraction);
            var targetDirection = CalculateDirection(DroneState.Yaw, CameraParams.HorizontalFOV, targetImage.HorizontalFraction);

            var downAngleRad = targetDownAngle * Math.PI / 180.0f;
            var directionRad = targetDirection * Math.PI / 180.0f;

            return FindTerrainIntersection(terrain, targetDirection, downAngleRad, directionRad);
        }

        private void ValidateInputs( ImagePosition targetImage)
        {
            if (DroneState.Altitude < MinDroneHeight)
                throw new ArgumentException("Drone altitude is below minimum safe height");

            if (targetImage.HorizontalFraction < -1 || targetImage.HorizontalFraction > 1 ||
                targetImage.VerticalFraction < -1 || targetImage.VerticalFraction > 1)
                throw new ArgumentException("Image position fractions must be between -1 and 1");
        }

        private double CalculateDownAngle(float cameraAngleDeg, float vFov, double verticalFraction)
        {
            // Convert FOV to radians
            var halfVFovRad = vFov * 0.5f * Math.PI / 180.0f;

            // Calculate the actual angle from the center using tangent relationship
            var angleFromCenterRad = Math.Atan(verticalFraction * Math.Tan(halfVFovRad));

            // Convert back to degrees and add to camera angle
            return cameraAngleDeg - angleFromCenterRad * 180.0f / Math.PI;
        }

        private double CalculateDirection(float yawDeg, float hFov, double horizontalFraction)
        {
            // Convert FOV to radians
            double halfHFovRad = hFov * 0.5f * Math.PI / 180.0f;

            // Calculate the actual angle from the center using tangent relationship
            double angleFromCenterRad = Math.Atan(horizontalFraction * Math.Tan(halfHFovRad));

            // Convert back to degrees and add to yaw
            double direction = yawDeg + angleFromCenterRad * 180.0f / Math.PI;

            // Normalize to 0-360 degrees
            return ((direction % 360) + 360) % 360;
        }

        // The code does NOT depend on FlightStep.InputImageCenter/InputImageSize/InputImageUnitVector/InputImageDemM/InputImageDsmM
        private LocationResult? FindTerrainIntersection(TerrainGrid terrain, double targetDirection, double downAngleRad, double directionRad)
        {
            // Convert angle back to degrees to compare easily
            var downAngleDeg = downAngleRad * 180.0f / Math.PI;

            DroneLocation currentPosition = DroneState.LocationNE;
            double currentAltitude = DroneState.Altitude;
            double totalDistance = 0;
            double confidence = 0;
            float currTerrainHeight = 0;
            int numSteps = 0;

            // Calculate initial height above terrain
            var terrainHeightAtStart = terrain.GetElevation(currentPosition);
            var heightAboveTerrain = currentAltitude - terrainHeightAtStart;

            DroneLocation directionVector = new(
                (float)Math.Cos(directionRad),   // North component
                (float)Math.Sin(directionRad)    // East component
            );

            // Initial step size based on height
            var horizontalStepSize = CalculateAdaptiveStepSize(terrain, heightAboveTerrain);

            var tanDownAngle = Math.Tan(downAngleRad);
            while (totalDistance < MaxSearchDistanceM)
            {
                numSteps++;

                // Move horizontally, drop altitude by tan(angle) * horizontalStep
                var verticalStep = horizontalStepSize * tanDownAngle;

                currentPosition = currentPosition.Add(directionVector.Multiply((float)horizontalStepSize));
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
                            Confidence = 0,
                            Method = "Norm",
                        };
                    }
                    return null;
                }
                currTerrainHeight = terrainHeight;

                if (currentAltitude + terrain.VerticalUnitM <= terrainHeight)
                {
                    // Interpolate back
                    var overshoot = terrainHeight - currentAltitude;
                    var stepBack = overshoot / verticalStep * horizontalStepSize;
                    currentPosition = currentPosition.Add(directionVector.Multiply((float)-stepBack));

                    terrainHeight = terrain.GetElevation(currentPosition);

                    confidence = CalculateConfidence(totalDistance, DroneState.Altitude);
                    return new LocationResult
                    {
                        LocationNE = currentPosition,
                        Elevation = terrainHeight,
                        Confidence = (float)confidence,
                        Method = "Norm",
                    };
                }

                // Recompute step size based on current height above terrain
                heightAboveTerrain = currentAltitude - terrainHeight;
                horizontalStepSize = CalculateAdaptiveStepSize(terrain, heightAboveTerrain);
            }

            // Max distance reached with no intersection
            return new LocationResult
            {
                LocationNE = currentPosition,
                Elevation = currTerrainHeight,
                Confidence = 0,
                Method = "Norm",
            };
         }


        private double CalculateAdaptiveStepSize(TerrainGrid terrain, double heightAboveTerrain)
        {
            // Smaller steps when closer to ground, larger steps when higher
            return Math.Max(terrain.VerticalUnitM, Math.Min(BaseStepSizeM * (heightAboveTerrain / 50.0), 5.0));
        }


        private double CalculateConfidence(double distance, float initialHeight)
        {
            // Confidence decreases with distance and initial height
            var distanceFactor = 1.0 - (distance / MaxSearchDistanceM);
            var heightFactor = 1.0 - (Math.Min(initialHeight, 300) / 300);
            return distanceFactor * 0.7 + heightFactor * 0.3;
        }
    }
}
