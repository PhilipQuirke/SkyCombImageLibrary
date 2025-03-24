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

            // Check if we should use near-vertical logic
            bool isNearVertical = (downAngleDeg > 85);

            string theMethod = "Norm";
            //if (!isNearVertical)
            //{
                //
                // -----------------
                // Normal Approach
                // -----------------
                //
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
                                Method = theMethod,
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
                            Method = theMethod,
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
                    Method = theMethod,
                };
            /*
             * This code is buggy - it produced an "L" shaped flight path
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
                    theMethod = "Vert";

                    // Example: take small vertical steps, compute horizontal offset from tan
                    // The smaller the step, the less "blow-up" from tan(near 90°).
                    float verticalStepSize = 1.0f;  // or terrain.VerticalUnitM, etc.

                    // if downAngleDeg is exactly 90°, tanDownAngle would be infinite
                    // so we clamp to a safe "maxTan" if angle is above 89° or so:
                    float safeTan = (float)Math.Tan(Math.Min(downAngleRad, 1.55334f));
                    // 1.55334 rad ~ 89 degrees, to prevent infinite blow-up


                    float droneYaw = drone.Yaw;          // 0°=North, clockwise
                    float cameraYaw = targetDirection;   // 0°=North, clockwise

                    // Put both in range [0..360) — though your code does this normalizing step for targetDirection
                    // Then compute a difference in the range -180..+180:
                    float rawDiff = cameraYaw - droneYaw;
                    float relDiff = ((rawDiff + 180) % 360) - 180;
                    // relDiff is now between -180 and +180, with negative meaning camera is to the “left” 
                    // or behind, positive meaning “right” or ahead, depending on your perspective

                    bool isCameraBehindDrone = (Math.Abs(relDiff) > 90.0f);
                    theMethod += isCameraBehindDrone ? "Behind" : "Front";

                    while (totalDistance < MaxSearchDistanceM)
                    {
                        numSteps++;

                        // Step down in altitude by a small amount
                        currentAltitude -= verticalStepSize;

                        // Horizontal movement: 
                        float horizontalStep = verticalStepSize / safeTan;

                        // Decide if we apply +horizontalStep or -horizontalStep:
                        float signedStep = horizontalStep; // Run 1, 4 & 6
                        //float signedStep = isCameraBehindDrone ? -horizontalStep : horizontalStep; // Run 2
                        //float signedStep = ! isCameraBehindDrone ? -horizontalStep : horizontalStep; // Run 3
                        //float signedStep = - horizontalStep; // Run 5

                        currentPosition = currentPosition.Add(directionVector.Multiply(signedStep));
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
                                    Confidence = 0,
                                    Method = theMethod,
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
                                Confidence = confidence,
                                Method = theMethod,
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
                        Confidence = 0,
                        Method = theMethod,
                    };
                }
            */
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
