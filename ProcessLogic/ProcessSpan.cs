// Copyright SkyComb Limited 2025. All rights reserved. 
using OpenCvSharp;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;
using System.Diagnostics;
using System.Drawing;


// Q1: Is zoom not zero? zoom is none or 1-1 or 100: "dzoom_ratio: 1.00"
// Q2: Is there some factor in image I am mising related to elevation? Why does image appear 70 CDDown when it is 90 CDDown?
// Q3: Save the object DEM in the Imagedata datastore tab.
// Q4: Draw DEM values on output image.


namespace SkyCombImage.ProcessLogic
{
    // ProcessSpan relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpan analyses ProcessObjects to refine/correct the flight altitude data using FlightStep.FixAltM/FixYawDeg/FixPitchDeg.
    public class ProcessSpan : ProcessSpanModel
    {
        // Parent process
        private ProcessAll Process { get; }


        public ProcessSpan(ProcessAll process, int spanId, List<string>? settings = null) : base(settings)
        {
            Process = process;
            ProcessSpanId = spanId;

            if (settings != null)
                LoadSettings(settings);
        }

        public void AssertGood()
        {
            Assert(ProcessSpanId > 0, "ProcessSpan.AssertGood: Bad SpanId");
            Assert(MinBlockId > 0, "ProcessSpan.AssertGood: Bad MinBlockId");
            Assert(MaxBlockId > 0, "ProcessSpan.AssertGood: Bad MaxBlockId");
        }


        // Apply FixValues to theSteps and on to the ProcessObjects and their ProcessFeatures
        public void CalculateSettings_ApplyFixValues_Core(int hFOVDeg, float fixAltM, float fixYawDeg, float fixPitchDeg, FlightStepList theSteps, ProcessObjList objs)
        {
            // The image area of the camera now covers a slightly different area (m2) of the ground
            Process.Drone.InputVideo.HFOVDeg = hFOVDeg;
            Assert(Process.VideoData.HFOVDeg == hFOVDeg, "CalculateSettings_ApplyFixValues_Core");

            // The image associated with each leg step now covers a slightly different area
            // Recalculate InputImageCenter Dem and Dsm based on FixAltM/FixYawDeg/FixPitchDeg
            theSteps.CalculateSettings_FixValues(fixAltM, fixYawDeg, fixPitchDeg, Process.VideoData, Process.Drone.GroundData);

            foreach (var theObject in objs)
            {
                var theObj = theObject.Value;

                // Clone the list of features (not the features themselves) claimed by the object 
                var objectFeatures = theObj.ProcessFeatures.Clone();

                // Eliminate all object summary data.
                theObj.ResetCalcedMemberData();

                // Recalc each feature - which will have a slightly different location
                foreach (var theFeature in objectFeatures)
                {
                    var theFeat = theFeature.Value;
                    theFeat.ResetCalcedMemberData();

                    // This code depends on ProcessAll.VideoData.HFOVDeg, FlightStep.FixAltM, FixYawDeg and FixPitchDeg  
                    // This code does NOT depend on FlightStep.InputImageCenter/InputImageSize/InputImageUnitVector/InputImageDemM/InputImageDsmM
                    theFeat.CalculateSettings_LocationM_HeightM_LOS(Process.GroundData);

                    theObj.ClaimFeature(theFeat);
                }
            }

            // Calculate the revised object-list location and height errors.
            objs.CalculateSettings();
        }


        // Apply FixValues to theSteps and on to the ProcessObjects and their ProcessFeatures //PQ??
        public bool CalculateSettings_ApplyFixValues(int hFOVDeg, float fixAltM, float fixYawDeg, float fixPitchDeg, FlightStepList theSteps, ProcessObjList objs)
        {
            CalculateSettings_ApplyFixValues_Core(hFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, theSteps, objs);

            // Do we see a drop in location error sum?
            // The location error assumes the objects are mostly stationary over the time they are observed.
            var improvementM = BestSumLocnErrM - objs.SumLocationErrM;
            if (improvementM >= 0.1f) // at least a 10cm improvement
            {
                SetBest(hFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, objs);
                return true;
            }

            return false;
        }


        // Analyse ProcessObjects in the selected steps by assuming the drone altitude is inaccurate.
        // Apply various "Fix*" trial values to the FlightSteps, ProcessFeatures & ProcessObjects.
        // For each trial, measure the sum of the object location errors.
        // Lock in the legSteps.Fix* value that reduces the error most.
        public void CalculateSettings_FixValues(FlightStepList theSteps, ProcessObjList theObjs)
        {
            try
            {
                Debug.Print("CalculateSettings_FixAltM_Start");
                NumSignificantObjects = theObjs.Count;

                if ((theSteps.Count == 0) || (NumSignificantObjects == 0))
                    return;

                //NQ Optimise
                float altRangeM = 25;
                float yawRangeDeg = 10;
                float pitchRangeDeg = 20;

                int theHFOVDeg = Process.Drone.InputVideo.HFOVDeg;

                ResetBest();
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, theSteps, theObjs);
                OrgSumLocnErrM = BestSumLocnErrM;
                OrgSumHeightErrM = BestSumHeightErrM;

                if (true)
                {
                    // DIMENSION 1: Video HFOVDeg
                    // The drone.InputVideo.HFOVDeg value is guessed from attributes of the SRT file. This is weak.
                    // Known hardware values for HFOVDeg are 38, 42 & 57. Test reasonable values around these.
                    CalculateSettings_ApplyFixValues(36, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(38, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(40, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(42, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(44, 0, 0, 0, theSteps, theObjs);
                    CalculateSettings_ApplyFixValues(57, 0, 0, 0, theSteps, theObjs);
                    theHFOVDeg = BestHFOVDeg;

                    // DIMENSION 2: Drone altitude 
                    ResetBest();
                    for (float fixAltM = -altRangeM; fixAltM <= altRangeM; fixAltM += 1)
                        CalculateSettings_ApplyFixValues(theHFOVDeg, fixAltM, 0, 0, theSteps, theObjs);
                    var bestAltM = BestFixAltM;
                    var bestAltErr = BestSumLocnErrM;

                    // DIMENSION 3: Drone yaw 
                    ResetBest();
                    for (float fixYawDeg = -yawRangeDeg; fixYawDeg <= yawRangeDeg; fixYawDeg += 1)
                        CalculateSettings_ApplyFixValues(theHFOVDeg, 0, fixYawDeg, 0, theSteps, theObjs);
                    var bestYawDeg = BestFixYawDeg;
                    var bestYawErr = BestSumLocnErrM;

                    // DIMENSION 4: Drone pitch 
                    ResetBest();
                    for (float fixPitchDeg = -pitchRangeDeg; fixPitchDeg <= pitchRangeDeg; fixPitchDeg += 1)
                        CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, fixPitchDeg, theSteps, theObjs);
                    var bestPitchDeg = BestFixPitchDeg;
                    var bestPitchErr = BestSumLocnErrM;

                    // DIMENSION 2..4: Fine tune
                    // Vary dimensions 2 to 4 around the above rough values
                    ResetBest();
                    CalculateSettings_ApplyFixValues(theHFOVDeg, bestAltM, bestYawDeg, bestPitchDeg, theSteps, theObjs);
                    for (float fixAltM = bestAltM - 1.25f; fixAltM <= bestAltM + 1.25f; fixAltM += 0.25f)
                        for (float fixYawDeg = bestYawDeg - 1.25f; fixYawDeg <= bestYawDeg + 1.25f; fixYawDeg += 0.25f)
                            for (float fixPitchDeg = bestPitchDeg - 1.25f; fixPitchDeg <= bestPitchDeg + 1.25f; fixPitchDeg += 0.25f)
                                CalculateSettings_ApplyFixValues(theHFOVDeg, fixAltM, fixYawDeg, fixPitchDeg, theSteps, theObjs);
                }

                // Lock in the best single value across the leg steps
                CalculateSettings_ApplyFixValues_Core(theHFOVDeg, BestFixAltM, BestFixYawDeg, BestFixPitchDeg, theSteps, theObjs);
                BestSumLocnErrM = theObjs.SumLocationErrM;
                BestSumHeightErrM = theObjs.SumHeightErrM;
                Process.ProcessObjects.CalculateSettings();

                Debug.Print("CalculateSettings_FixValues: theHFOVDeg=" + theHFOVDeg.ToString() + "BestFixAltM=\" + BestFixAltM.ToString() + \"BestFixYawDeg=" + BestFixYawDeg.ToString() + " BestFixPitchDeg=" + BestFixPitchDeg.ToString());
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessSpan.CalculateSettings_FixValues", ex);
            }
        }


        // Summarise the blocks in this leg
        private void SummariseSteps(FlightStepList steps)
        {
            if (steps.Count > 0)
            {
                MinStepId = steps.First().Value.StepId;
                MaxStepId = steps.Last().Value.StepId;
            }

            ResetTardis();
            foreach (var step in steps)
                SummariseTardis(step.Value);
        }


        // Analyse ProcessObjects in the FlightLeg, assuming various inaccuracies.
        // Lock in the FlightSteps.FixAltM/FixYawDeg/FixPitchDeg values that reduces the location wobble most.
        public void CalculateSettings_from_FlightLeg()
        {
            ResetBest();
            ResetTardis();

            if (ProcessSpanId == UnknownValue)
                return;

            var legSteps = Process.Drone.FlightSteps.Steps.GetLegSteps(ProcessSpanId);
            var theObjs = Process.ProcessObjects.FilterByLeg(ProcessSpanId);

            if (false)
            {
                // nq new method
                TriangulateSpanObjectsFeaturesLocationAndHeight(theObjs);
                foreach (var theObj in theObjs)
                    theObj.Value.Calculate_RealObject_SimpleMemberData();
                theObjs.CalculateSettings();
            }
            else
            {
                // Old method
                //CalculateSettings_FixValues(legSteps, theObjs);

                // No method
                int theHFOVDeg = Process.Drone.InputVideo.HFOVDeg;
                ResetBest();
                CalculateSettings_ApplyFixValues(theHFOVDeg, 0, 0, 0, legSteps, theObjs);
                OrgSumLocnErrM = BestSumLocnErrM;
                OrgSumHeightErrM = BestSumHeightErrM;
            }

            SummariseSteps(legSteps);
        }


        // Analyse ProcessObjects in the Block series - assuming the drone altitude is inaccurate.
        // Lock in the FlightSteps.FixAltM/FixYawDeg/FixPitchDeg values that reduces the location most.
        public void CalculateSettings_from_FlightSteps(int minStepId, int maxStepId)
        {
            try
            {
                ResetBest();
                ResetTardis();

                if ((Process.Drone.FlightSteps != null) && (Process.ProcessObjects.Count > 0))
                {
                    // Get the FlightSteps corresponding to the block range
                    FlightStepList theSteps = new();
                    for (int stepId = minStepId; stepId <= maxStepId; stepId++)
                    {
                        FlightStep theStep;
                        if (Process.Drone.FlightSteps.Steps.TryGetValue(stepId, out theStep))
                            theSteps.AddStep(theStep);
                    }
                    if (theSteps.Count >= 4)
                    {
                        // Get the Objects that exist inside the block range (not overlapping the block range).
                        // We may be analyzing 30 minutes of video.
                        // Recall that objects are ordered by the BlockId of the first feature in the object.
                        // For speed, scan backwards through ProcessObjList
                        // until we find an object that ends before minBlockId.
                        var allObjs = Process.ProcessObjects;
                        ProcessObjList theObjs = new();
                        for (int objectId = allObjs.Last().Key; objectId >= 0; objectId--)
                        {
                            ProcessObject theObject;
                            if (allObjs.TryGetValue(objectId, out theObject))
                            {
                                var firstFeat = theObject.FirstFeature;
                                var lastFeat = theObject.LastRealFeature;
                                if ((firstFeat != null) && (lastFeat != null))
                                {
                                    var firstStepId = firstFeat.Block.FlightStepId;
                                    var lastRealStepId = lastFeat.Block.FlightStepId;

                                    if ((firstStepId >= minStepId) && (lastRealStepId <= maxStepId))
                                        // Object lies fully within the specified block range
                                        theObjs.AddObject(theObject);

                                    if (lastRealStepId < minStepId)
                                        break; // We've gone back far enough
                                }
                            }

                        }


                        CalculateSettings_FixValues(theSteps, theObjs);
                        SummariseSteps(theSteps);
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessSpan.CalculateSettings_from_FlightSteps", ex);
            }
        }


        public int PercentOverlapWithRunFromTo(Drone drone)
        {
            return FlightLeg.PercentOverlapWithRunFromTo(drone, MinStepId, MaxStepId);
        }
        public bool OverlapsRunFromTo(Drone drone)
        {
            return PercentOverlapWithRunFromTo(drone) >= FlightLegModel.MinOverlapPercent;
        }

//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        // Recalculate the Span.Objects.Features.LocationM and HeightM using triangulation.
        public Mat K = Intrinsic(9.1, 640, 512, 7.68, 6.144);
        public void TriangulateSpanObjectsFeaturesLocationAndHeight(ProcessObjList theObjs)
        {
            var compareInterval = 3; // Frame pair intervals constant, 3 frames is 1/20th of second. 5 frames is 1/6th of a second.
            var totBlocks = Process.Blocks.Count;
            foreach (var block in Process.Blocks.Values)
            {
                var blockId = block.BlockId;
                var compareBlockId = blockId + compareInterval;
                if (compareBlockId > totBlocks) break; // not enough compare interval left
                var compareBlock = Process.Blocks[compareBlockId];
                if (block.MinFeatureId < 1 || compareBlock.MinFeatureId < 1) continue; // no features in either this or compare
                if (block.DroneLocnM.EastingM == compareBlock.DroneLocnM.EastingM
                    && block.DroneLocnM.NorthingM == compareBlock.DroneLocnM.NorthingM) continue; // not enough drone location difference for the compare
                var BlocksInfo = new BlockInfo();

                for (var id = block.MinFeatureId ; id <= block.MaxFeatureId; id++)
                {
                    var feature = Process.ProcessFeatures[id];
                    if (!feature.Significant) continue;
                    var objid = feature.ObjectId;
                    for (var idC = compareBlock.MinFeatureId; idC <= compareBlock.MaxFeatureId; idC++)
                    {
                        var featureC = Process.ProcessFeatures[idC];
                        var objidC = featureC.ObjectId;
                        if (objid != objidC) continue;
                        BlocksInfo.fromObs.Add(objid);
                        BlocksInfo.fromFeatures.Add(objid, feature);
                        BlocksInfo.toFeatures.Add(objid, featureC);
                    }
                }
                if (BlocksInfo.fromObs.Count == 0) continue;
                Debug.WriteLine("+++++++++++++++++++++++++");
                using var Points1 = ToTriangulationFormat(BlocksInfo.CreatePoints(true));
                using var Points2 = ToTriangulationFormat(BlocksInfo.CreatePoints(false));
                using var Projection1 = BlocksInfo.CreateProjectionMatrix(block.DroneLocnM.EastingM, block.DroneLocnM.NorthingM, block.AltitudeM, block.RollDeg, block.PitchDeg, block.YawDeg, K);
                using var Projection2 = BlocksInfo.CreateProjectionMatrix(compareBlock.DroneLocnM.EastingM, compareBlock.DroneLocnM.NorthingM, compareBlock.AltitudeM, compareBlock.RollDeg, compareBlock.PitchDeg, compareBlock.YawDeg, K);
                using Mat homogeneousPoints = new Mat();
                Debug.WriteLine(blockId.ToString());
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        Debug.Write(Projection1.At<double>(i, j).ToString() + " ");
                    }
                    Debug.WriteLine("");
                }

                Cv2.TriangulatePoints(Projection1, Projection2, Points1, Points2, homogeneousPoints);

                // Convert homogeneous coordinates to 3D and update the locations into YoloProcessFeature
                int counter = 0;
                foreach (var thisfeature in BlocksInfo.fromFeatures.Values)
                {
                    thisfeature.realLocation = [(float)(homogeneousPoints.At<double>(0, counter) / homogeneousPoints.At<double>(3, counter)),
                            (float)(homogeneousPoints.At<double>(1, counter) / homogeneousPoints.At<double>(3, counter)),
                            (float)(homogeneousPoints.At<double>(2, counter) / homogeneousPoints.At<double>(3, counter))];
                    counter++;
                }

            }

            // Overwriting existing feature LocationM and HeightM
            double[] lastlocation = [0,0,0];
            foreach (var obj in theObjs)
                foreach (ProcessFeature thisfeature in obj.Value.ProcessFeatures.Values)
                {
                    if (thisfeature.realLocation[0] == 0 && thisfeature.realLocation[1] == 0) //this happens because of the compare interval, the location is written to the first feature of the pair.
                    {
                        thisfeature.LocationM = new DroneLocation((float)lastlocation[1], (float)lastlocation[0]);
                        thisfeature.HeightM = (float)lastlocation[2];
                    }
                    else
                    {
                        thisfeature.LocationM = new DroneLocation((float)thisfeature.realLocation[1], (float)thisfeature.realLocation[0]);
                        thisfeature.HeightM = (float)thisfeature.realLocation[2];
                        lastlocation = thisfeature.realLocation;
                    }
                        Debug.WriteLine(thisfeature.BlockId.ToString() + "," + obj.Key.ToString() + "," + thisfeature.LocationM.EastingM.ToString() + "," + thisfeature.LocationM.NorthingM.ToString() + ",");
                     
                }

        }

        // This was ChatGPT's formulation for the intrinsic matrix.
        private static Mat Intrinsic(double focalLength, double imageWidth, double imageHeight, double sensorWidth, double sensorHeight)
        {
            var Cx = imageWidth / 2; var Cy = imageHeight / 2;
            var Fx = focalLength * imageWidth / sensorWidth; // F * pixels per mm = focal length in mm x image width px / sensor width mm
            var Fy = focalLength * imageHeight / sensorHeight;
            Mat K = new Mat(3, 3, MatType.CV_64F);
            K.At<double>(0, 0) = Fx;
            K.At<double>(0, 1) = 0;
            K.At<double>(0, 2) = Cx;
            K.At<double>(1, 0) = 0;
            K.At<double>(1, 1) = Fy;
            K.At<double>(1, 2) = Cy;
            K.At<double>(2, 0) = 0;
            K.At<double>(2, 1) = 0;
            K.At<double>(2, 2) = 1;
            return K;
        }

        // Convert collected points to OpenCV Mat format for triangulation
        public Mat ToTriangulationFormat(List<Point2d> points1)
        {
            var points1Mat = new Mat(2, points1.Count, MatType.CV_64F);

            for (int i = 0; i < points1.Count; i++)
            {
                points1Mat.Set<double>(0, i, points1[i].X);
                points1Mat.Set<double>(1, i, points1[i].Y);
            }

            return points1Mat;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------

    public class BlockInfo
    {
        private bool disposed = false;
        public int BlockNumber;
        public double EastingM, NorthingM, AltitudeM, RollDeg, PitchDeg, YawDeg;
        public SortedList<int, ProcessFeature> fromFeatures = new SortedList<int, ProcessFeature>();
        public SortedList<int, ProcessFeature> toFeatures = new SortedList<int, ProcessFeature>();
        public List<int> fromObs = new List<int>();
        // Create the points list
        public List<Point2d> CreatePoints(bool from)
        {
            var pointlist = new List<Point2d>();
            var feats = from ? fromFeatures : toFeatures;
            foreach (var obj in fromObs)
            {
                Debug.Write((from? "From,":"To,") +obj.ToString()+",");
                pointlist.Add(centroid(feats[obj].PixelBox));
            }
            return pointlist;
        }

        // Calculate the centroid of the pixelbox and convert location to real pixels, half that given, because the image has been double sized by DGI.
        private Point2d centroid(Rectangle rectangle)
        {
            Point2d centroid = new Point2d();
            centroid.X = (rectangle.X + rectangle.Width / 2)/2;
            centroid.Y = (rectangle.Y + rectangle.Height / 2)/2;
            Debug.WriteLine(centroid.X.ToString() + "," + centroid.Y.ToString() + ",");

            return centroid;

        }
        // Using camera drone information, determine camera heading/position
        private static Mat CreateRotationMatrix(double rollDegrees, double pitchDegrees, double yawDegrees)
        {
            // Convert angles to radians
            double roll = rollDegrees * Math.PI / 180.0;
            double pitch = pitchDegrees * Math.PI / 180.0;
            double yaw = yawDegrees * Math.PI / 180.0;

            // Create rotation matrices
            using Mat Rx = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(Rx);
            Rx.At<double>(1, 1) = Math.Cos(roll);
            Rx.At<double>(1, 2) = -Math.Sin(roll);
            Rx.At<double>(2, 1) = Math.Sin(roll);
            Rx.At<double>(2, 2) = Math.Cos(roll);

            using Mat Ry = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(Ry);
            Ry.At<double>(0, 0) = Math.Cos(pitch);
            Ry.At<double>(0, 2) = Math.Sin(pitch);
            Ry.At<double>(2, 0) = -Math.Sin(pitch);
            Ry.At<double>(2, 2) = Math.Cos(pitch);

            using Mat Rz = new Mat(3, 3, MatType.CV_64F);
            Cv2.SetIdentity(Rz);
            Rz.At<double>(0, 0) = Math.Cos(yaw);
            Rz.At<double>(0, 1) = -Math.Sin(yaw);
            Rz.At<double>(1, 0) = Math.Sin(yaw);
            Rz.At<double>(1, 1) = Math.Cos(yaw);

            // Combine rotations
            return Rz * Ry * Rx;
        }

        /* Calulate the projection matrices from K, using the camera intrinsic matrix, and the drone camera's rotation matrix.
             The general form of the projection matrix P is P=K⋅[R∣t], where [R∣t] is the camera's extrinsic matrix, composed of the rotation matrix R and the translation vector t. 
             The translation vector t is derived from the camera's position C in the world as follows: t =  − R⋅C.   */
        public Mat CreateProjectionMatrix(
            double easting, double northing, double altitude,
            double rollDegrees, double pitchDegrees, double yawDegrees, Mat K)
        {
            // Create rotation matrix
            using Mat R = CreateRotationMatrix(rollDegrees, pitchDegrees, yawDegrees);

            // Create translation vector
            using Mat t = new Mat(3, 1, MatType.CV_64F);
            t.At<double>(0, 0) = easting;
            t.At<double>(1, 0) = northing;
            t.At<double>(2, 0) = altitude;

            // Create [R|t] matrix
            using Mat Rt = new Mat(3, 4, MatType.CV_64F);
            using Mat negRt = -R * t;

            for (int i = 0; i < 3; i++) // Copy rotation matrix
                for (int j = 0; j < 3; j++)
                    Rt.At<double>(i, j) = R.At<double>(i, j);
            
            for (int i = 0; i < 3; i++) // Copy negative translation
                Rt.At<double>(i, 3) = negRt.At<double>(i, 0);

            // Calculate P = K[R|t]
            return K * Rt;

        }
    }

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // A list of ProcessSpan objects
    public class ProcessSpanList : SortedList<int, ProcessSpan>
    {
        public ProcessSpanList()
        {
        }


        public void AddSpan(ProcessSpan processSpan)
        {
            BaseConstants.Assert(processSpan.ProcessSpanId > 0, "ProcessSpanList.AddLeg: No Id");
            Add(processSpan.ProcessSpanId, processSpan);
        }


        // Return the index of the first and last ProcessSpan overlap of this leg with the RunVideoFromS / RunVideoToS range
        public (int, int) OverlappingSpansRange(Drone drone)
        {
            int firstSpanId = BaseConstants.UnknownValue;
            int lastSpanId = BaseConstants.UnknownValue;

            foreach (var span in this)
            {
                if (span.Value.OverlapsRunFromTo(drone))
                {
                    if (firstSpanId == BaseConstants.UnknownValue)
                        firstSpanId = span.Value.ProcessSpanId;
                    lastSpanId = span.Value.ProcessSpanId;
                }
                else
                {
                    if (firstSpanId != BaseConstants.UnknownValue)
                        break;
                }
            }

            return (firstSpanId, lastSpanId);
        }


        public void SetFixValuesAfterLoad(VideoModel videoData, Drone drone)
        {
            if (drone.FlightSteps == null)
                return;

            var steps = drone.FlightSteps.Steps;

            foreach (var theSpan in this)
                if (Math.Abs(theSpan.Value.BestFixAltM) + Math.Abs(theSpan.Value.BestFixYawDeg) + Math.Abs(theSpan.Value.BestFixPitchDeg) > 0)
                    // PQR TODO Should we use BestFixHFOV to set videoData.HFOVDeg?
                    for (int stepId = theSpan.Value.MinStepId; stepId <= theSpan.Value.MaxStepId; stepId++)
                    {
                        if (steps.TryGetValue(stepId, out var step))
                        {
                            step.FixAltM = theSpan.Value.BestFixAltM;
                            step.FixYawDeg = theSpan.Value.BestFixYawDeg;
                            step.FixPitchDeg = theSpan.Value.BestFixPitchDeg;
                            step.CalculateSettings_InputImageCenterDemDsm(videoData, drone.GroundData);
                        }
                    }
        }
    }

}
