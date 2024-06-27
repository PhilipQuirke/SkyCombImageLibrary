// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using System.Drawing;


namespace SkyCombImage.ProcessModel
{
    // Represents an Optical Flow processing block 
    public class FlowBlock : ProcessBlock
    {
        // Static config data shared by all Flow blocks
        private static FlowProcess FlowProcess;


        // Ground velocity/speed for this block in pixels per frame, based on 25% percentile.
        // Based on the output of the Flow process.
        // This is an "instant" velocity - based on difference between prev and this block - without any multi-block smoothing.
        public VelocityF? FlowGroundVelInPixelsPerBlock { get; set; }
        // Average velocity/speed of all features in this block, based on 50% percentile. Higher than GroundVelocityInPixels. This is an "instant" velocity. 
        public VelocityF? FlowAverageVelInPixelsPerBlock { get; set; }
        // Surface (tree-top) velocity/speed for this block, based on 75% percentile. Higher than AverageVelocityInPixels. This is an "instant" velocity.
        public VelocityF? FlowSurfaceVelInPixelsPerBlock { get; set; }


        // On rare occassions (e.g. DJI_0088, from 140s to 144s) CalcOpticalFlowPyrLK will decide that an existing 
        // (previous) feature has been found again in the next frame, but a substantial distance from its location in
        // the previous frame! And do this jump for several features in the same frame!!
        //
        // CalcOpticalFlowPyrLK is not aware that we are processing images from a slow-moving drone with a
        // downward-looking camera, processing 8 to 30 frames per second.
        // While we expect tree-top features to move faster than ground features, 
        // we do not expect excessive feature location movements - particularly inside flight legs.
        // We de-select features with an excessive movement above a "cutoff" value
        // based on the median movement metric.
        public int MedianMovePx = UnknownValue;

        // Some details of the features associated with this block.
        private int NumFeatures = 0;
        private List<int> FeatDeltaX;
        private List<int> FeatDeltaY;


        public FlowBlock(FlowProcess flowProcess, ProcessScope scope) : base(scope)
        {
            FeatDeltaX = new();
            FeatDeltaY = new();
            FlowProcess = flowProcess;
        }


        // Store data on a pixel's movement in this block.
        public void AddDelta(int deltaY, int deltaX)
        {
            NumFeatures++;
            FeatDeltaX.Add(deltaX);
            FeatDeltaY.Add(deltaY);
        }


        // Calculate the velocity at the 1st, 2nd or 3rd quartile (will crash on 0th or 4th quartile).
        public VelocityF QuartileVelocity(int quartile, List<int> theDeltaX, List<int> theDeltaY)
        {
            if (NumFeatures == 0)
                return new VelocityF();

            int range = NumFeatures / 10;
            int middle = quartile * NumFeatures / 4;

            int units = 0;
            int sumDeltaX = 0;
            int sumDeltaY = 0;
            for (int i = middle - range; i <= middle + range; i++)
            {
                Assert(i >= 0, "FlowBlock.QuartileVelocity: Bad logic 1");
                Assert(i < NumFeatures, "FlowBlock.QuartileVelocity: Bad logic 2");

                units++;
                sumDeltaX += theDeltaX[i];
                sumDeltaY += theDeltaY[i];
            }

            return new VelocityF((1.0F * sumDeltaX) / units, (1.0F * sumDeltaY) / units);
        }


        // Calculate the GroundVelocityInPixels, AverageVelocityInPixels and SurfaceVelocityInPixels
        public void CalculateFlowVelocities()
        {
            try
            {
                // The accuracy of the Flow speeds relies on the whole screen moving in same direction.
                // A spin event or dynamically changing the camera down angle breaks the Flow speed calculations.
                // Only calculate flow speed during legs
                if (FlightLegId <= 0)
                {
                    FlowGroundVelInPixelsPerBlock = null;
                    FlowAverageVelInPixelsPerBlock = null;
                    FlowSurfaceVelInPixelsPerBlock = null;
                }
                else
                {
                    // It feels wrong to sort these independently. We should sort these 'together', based on their diagonal size
                    // We are ordering these Feature movements in increasing order. The slower movements are likely to be features
                    // on the ground (furthest from the camera), whilst faster moving features are objects at a height off the ground

                    // was FeatDeltaX.Sort();
                    // was FeatDeltaY.Sort();

                    // Create a new array of the diagonal values (squared)
                    Assert(FeatDeltaX.Count == FeatDeltaY.Count, "CalculateFlowVelocities: Should have FeatDeltaX size == FeatDeltaY size");
                    List<(float, int)> diags = new List<(float, int)>();
                    for (int i = 0; i < FeatDeltaX.Count; i++)
                    {
                        float diagonalValue = FeatDeltaX[i] * FeatDeltaX[i] + FeatDeltaY[i] * FeatDeltaY[i];
                        diags.Add(new(diagonalValue, i));
                    }
                    diags.Sort(); // works on first element of tuple 

                    // Now re-order our Features
                    List<int> FeatDeltaX_new = new List<int>();
                    List<int> FeatDeltaY_new = new List<int>();
                    for (int i = 0; i < FeatDeltaX.Count; i++)
                    {
                        int index = diags[i].Item2;
                        FeatDeltaX_new.Add(FeatDeltaX[index]);
                        FeatDeltaY_new.Add(FeatDeltaY[index]);
                    }
                    FeatDeltaX = FeatDeltaX_new;
                    FeatDeltaY = FeatDeltaY_new;

                    // Calculate GroundVelocityInPixels at the 1st quartile (aka 25% percentile).
                    // Visually, the flow process seems to have some "Stuck" features, which dont move, even as video is moving.
                    // Using the 1st quartile ignores the slowest moving features. 
                    FlowGroundVelInPixelsPerBlock = QuartileVelocity(1, FeatDeltaX, FeatDeltaY);

                    // Calculate AverageVelocityInPixels at the 2nd quartile (aka 50% percentile).
                    FlowAverageVelInPixelsPerBlock = QuartileVelocity(2, FeatDeltaX, FeatDeltaY);

                    // Calculate SurfaceVelocityInPixels at the 3rd quartile (aka 75% percentile).
                    // Visually, the flow process seems to have some "pin ball" features, which zap around randomly.
                    // Using the 3rd quartile ignores the fastest moving features.
                    FlowSurfaceVelInPixelsPerBlock = QuartileVelocity(3, FeatDeltaX, FeatDeltaY);

                    // The above code assumes that FeatDeltaX (and ditto FeatDeltaY) is mostly positive or mostly negative.
                    // If FeatDeltaX is evenly spread across positive and negative values (during a turning event),
                    // then Quartile1 will be negative, Quartile2 will be around zero, and Quartile3 will be positive,
                    // breaking the below Asserts!
                    // Assert(FlowGroundInPixelsPerBlock.Speed() <= FlowAverageInPixelsPerBlock.Speed(), "CalculateFlowVelocities: Bad logic 1");
                    // Assert(FlowAverageInPixelsPerBlock.Speed() <= FlowSurfaceInPixelsPerBlock.Speed(), "CalculateFlowVelocities: Bad logic 2");
                }

                FeatDeltaX = null;
                FeatDeltaY = null;
            }
            catch (Exception ex)
            {
                throw ThrowException("FlowBlock.CalculateFlowGroundAndSurfaceVelocity", ex);
            }
        }


        // Return the data required to draw the FRAME Point of View speed/direction as arrow and speed text.
        // This is NOT the drone speed/direction. 
        public (bool, VelocityF?, string) GetFlowGroundVelocity()
        {
            return ((FlowGroundVelInPixelsPerBlock != null), FlowGroundVelInPixelsPerBlock, "px/frame");
        }
    }


    public class FlowBlockList : ProcessBlockList
    {
        private FlowProcess FlowProcess;


        public FlowBlockList(FlowProcess flowProcess) 
        {
            FlowProcess = flowProcess;
        }


        public FlowBlock AddBlock(ProcessScope scope, Drone drone)
        {
            var newBlock = ProcessFactory.NewFlowBlock(FlowProcess, scope);
            AddBlock(newBlock, scope, drone);
            return newBlock;
        }
    };


    // A "Good Feature To Track" feature 
    public class FlowFeature : ProcessFeatureModel
    {
        public FlowFeature(int blockId, Point location) : base(blockId, CombFeatureTypeEnum.Real)
        {
            ResetMemberData();
            LocationM = new DroneLocation(location.Y,location.X);
            Significant = true;
        }
    };


    public class FlowFeatureList : List<FlowFeature>
    {
        private static ProcessConfigModel ProcessConfig;


        public FlowFeatureList(ProcessConfigModel config)
        {
            ProcessConfig = config;
        }


        public FlowFeature AddFeature(int blockId, Point location)
        {
            var answer = new FlowFeature(blockId, location);
            this.Add(answer);
            return answer;
        }
    };


    // A class to hold an Optical Flow object - layer over a sequence of GFTT Flow features.
    public class FlowObject : ProcessObject
    {
        private static readonly RNG rng = new();


        // Random color to draw the flow feature in 
        public Bgr Color { get; }
        // First feature claimed by this object
        private FlowFeature FirstFeature { get; set; } = null;
        // Last feature claimed by this object
        public FlowFeature LastFeature { get; set; } = null;
        // Movement of this object over all blocks in pixels
        private int MinAbsDeltaY { get; set; } = UnknownValue;
        private int MinAbsDeltaX { get; set; } = UnknownValue;
        public int MaxAbsDeltaY { get; set; } = UnknownValue;
        private int MaxAbsDeltaX { get; set; } = UnknownValue;
        private int SumDeltaY { get; set; } = UnknownValue;
        private int SumDeltaX { get; set; } = UnknownValue;

        // Last (previous) position of object in pixels, with sub-pixel accuracy.
        public float PrevPointY = UnknownValue;
        public float PrevPointX = UnknownValue;


        public FlowObject(FlowProcess flowProcess, ProcessScope scope) : base(flowProcess.ProcessConfig, scope)
        {
            int r = rng.Uniform(0, 256);
            int g = rng.Uniform(0, 256);
            int b = rng.Uniform(0, 256);
            this.Color = new Bgr(b, g, r);

            if (this.ObjectId == ProcessConfig.FocusObjectId)
                this.Color = new Bgr(0, 0, 255);

            Significant = true;
        }


        // Does the last feature occupy the same block & location as the params?
        public bool LastFeatureIntersects(int blockId, int theY, int theX)
        {
            if (LastFeature.BlockId < blockId)
                return false;

            // Euclidian distance test without use of slow square root function.
            var lastEntry = LastFeature.LocationM;
            return
                Math.Pow(lastEntry.EastingM - theX, 2) +
                Math.Pow(lastEntry.NorthingM - theY, 2) <
                ProcessConfig.GfttMinDistance * ProcessConfig.GfttMinDistance;
        }


        // This object claims this feature
        public void ClaimFeature(FlowFeature theFeature)
        {
            Assert(theFeature.ObjectId <= 0, "FlowObject.ClaimFeature: Feature is already owned.");
            theFeature.ObjectId = this.ObjectId;

            if (FirstFeature == null)
            {
                FirstFeature = theFeature;
                LastFeature = theFeature;

                SumDeltaY = 0;
                SumDeltaX = 0;
            }
            else
            {
                var prevLastFeature = LastFeature;
                LastFeature = theFeature;

                int deltaY = (int)(theFeature.LocationM.NorthingM - prevLastFeature.LocationM.NorthingM);
                int deltaX = (int)(theFeature.LocationM.EastingM - prevLastFeature.LocationM.EastingM);

                SumDeltaY += deltaY;
                SumDeltaX += deltaX;

                if (MinAbsDeltaX == UnknownValue)
                    MinAbsDeltaX = Math.Abs(deltaX);
                else
                    MinAbsDeltaX = Math.Min(MinAbsDeltaX, Math.Abs(deltaX));
                if (MinAbsDeltaY == UnknownValue)
                    MinAbsDeltaY = Math.Abs(deltaY);
                else
                    MinAbsDeltaY = Math.Min(MinAbsDeltaY, Math.Abs(deltaY));

                if (MaxAbsDeltaX == UnknownValue)
                    MaxAbsDeltaX = Math.Abs(deltaX);
                else
                    MaxAbsDeltaX = Math.Max(MaxAbsDeltaX, Math.Abs(deltaX));
                if (MaxAbsDeltaY == UnknownValue)
                    MaxAbsDeltaY = Math.Abs(deltaY);
                else
                    MaxAbsDeltaY = Math.Max(MaxAbsDeltaY, Math.Abs(deltaY));
            }

            NumSigBlocks = LastFeature.BlockId - FirstFeature.BlockId + 1;
        }


        // Returns total distance travelled by feature in pixels and average speed in pixels / block
        public (double, double) DistanceSpeedBlocks()
        {
            if ((SumDeltaY == UnknownValue) || (SumDeltaX == UnknownValue) || (FirstFeature == null) || (FirstFeature == null))
                return (0, 0);

            double distanceTraveled = Math.Sqrt(SumDeltaY * SumDeltaY + SumDeltaX * SumDeltaX);
            double averageSpeed = distanceTraveled / NumSigBlocks;

            return (distanceTraveled, averageSpeed);
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        override public DataPairList GetSettings()
        {
            (var featDistance, var featSpeed) = DistanceSpeedBlocks();

            return new DataPairList
            {
                { "Object", ObjectId },
                { "FromS", RunFromVideoS, SecondsNdp },
                { "ToS", RunToVideoS, SecondsNdp },
                { "Attributes", Attributes },
                { "Significant", Significant },
                { "#SigBlocks", NumSigBlocks },
                { "Distance", featDistance, 2 },
                { "Speed", featSpeed, 2 },
                { "AvgDelta", (NumSigBlocks > 0) ? 1.0 * Math.Abs(SumDeltaY + SumDeltaX) / NumSigBlocks : 0.0, 2 },
                { "MinDeltaY", MinAbsDeltaY },
                { "MinDeltaX", MinAbsDeltaX },
                { "MaxDeltaY", MaxAbsDeltaY },
                { "MaxDeltaX", MaxAbsDeltaX },
                { "SumDeltaY", Math.Abs(SumDeltaY) },
                { "SumDeltaX", Math.Abs(SumDeltaX) },
                { "Color", Color.ToString() },
                { "FirstFeat", (FirstFeature == null) ? -1 : FirstFeature.FeatureId },
                { "LastFeat", (LastFeature == null) ? -1 : LastFeature.FeatureId },
            };
        }
    };


    public class FlowObjectList : List<FlowObject>
    {
        private FlowProcess FlowProcess;


        // We do not process objects below this index in the object array.
        public int LegFirstIndex;


        public FlowObjectList(FlowProcess flowProcess)
        {
            FlowProcess = flowProcess;
        }


        public FlowObject AddObject(ProcessScope scope)
        {
            var answer = new FlowObject(FlowProcess, scope);
            Add(answer);
            return answer;
        }


        // Return the list of features that are significant
        public int SignificantCount()
        {
            int answer = 0;

            foreach (var theObject in this)
                if (theObject.Significant)
                    answer++;

            return answer;
        }


        // No existing objects should be live at the start of a new leg
        public void ProcessLegStart()
        {
            foreach (var theObject in this)
                theObject.Significant = false;

            LegFirstIndex = Count;
        }


        // Return the list of features that are currently significant
        public FlowObjectList SignificantList()
        {
            FlowObjectList answer = new(FlowProcess);

            for (int index = LegFirstIndex; index < Count; index++)
            {
                var theObject = this[index];
                if (theObject.Significant)
                    answer.Add(theObject);
            }

            return answer;
        }


        public Emgu.CV.Util.VectorOfPointF PrevPoints()
        {
            var answer = new Emgu.CV.Util.VectorOfPointF();

            foreach (var theObject in this)
                answer.Push(new[] { new PointF(theObject.PrevPointX, theObject.PrevPointY) });

            return answer;
        }
    };



    // A class to hold all Optical Flow feature and block data associated with a video
    public class FlowProcess : ProcessAll
    {
        public FlowBlockList FlowBlocks;
        public FlowObjectList FlowObjects;
        public FlowFeatureList FlowFeatures;


        public FlowProcess(ProcessConfigModel config, Drone drone) : base(config, drone.InputVideo, drone)
        {
            FlowBlocks = new(this);
            FlowObjects = new(this);
            FlowFeatures = new(config);

            FlowObjects.LegFirstIndex = 0;
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public override void ResetModel()
        {
            FlowBlocks.Clear();
            FlowFeatures.Clear();
            FlowObjects.Clear();

            FlowObjects.LegFirstIndex = 0;

            base.ResetModel();
        }


        // For process robustness, we want to process each leg independently.
        // So when a new drone flight leg starts, we "reset" the Gftt/OpticalFlow calculations.
        public override void ProcessFlightLegStart(int legId)
        {
            // No existing objects should be live at the start of a new leg
            FlowObjects.ProcessLegStart();
        }


        // Create a flow feature and add it to the flow object. Update the block totals
        public FlowFeature ObjectClaimsNewFeature(FlowObject theObject, FlowBlock block, float currPointX, float currPointY)
        {
            try
            {
                var newFeature = FlowFeatures.AddFeature(block.BlockId, new Point((int)currPointX, (int)currPointY));

                block.AddFeature(newFeature);
                theObject.ClaimFeature(newFeature);

                if (theObject.PrevPointY != UnknownValue)
                {
                    var deltaX = currPointX - theObject.PrevPointX;
                    var deltaY = currPointY - theObject.PrevPointY;
                    block.AddDelta((int)deltaY, (int)deltaX);
                }

                theObject.PrevPointY = currPointY;
                theObject.PrevPointX = currPointX;

                return newFeature;
            }
            catch (Exception ex)
            {
                throw ThrowException("FlowProcessModel.ObjectClaimsNewFeature", ex);
            }
        }


        // Process new "Good features to track" features and add the genuinely new ones into this model.
        // Returns current number of significant Flow objects 
        public int ProcessNewGftt(MKeyPoint[] newGftts, ProcessScope scope)
        {
            int sizeOnEntry = FlowObjects.Count;

            // Add new points into Objects & Features
            for (int i = 0; i < newGftts.Length; i++)
            {
                var thePoint = newGftts[i].Point;
                int pointX = (int)thePoint.X;
                int pointY = (int)thePoint.Y;

                // If thePoint is already being tracked, then no need to add it in.
                bool match = false;
                for (int j = FlowObjects.LegFirstIndex; j < sizeOnEntry; j++)
                    if (FlowObjects[j].LastFeatureIntersects(scope.PSM.CurrBlockId - 1, pointY, pointX))
                    {
                        match = true;
                        break;
                    }
                if (match)
                    continue;

                // We have found a new feature to track.
                var newObject = FlowObjects.AddObject(scope);
                ObjectClaimsNewFeature(newObject, FlowBlocks.LastBlock as FlowBlock, pointX, pointY);
            }

            return FlowObjects.SignificantCount();
        }


        // On rare occassions (e.g. DJI_0088, from 140s to 144s) CalcOpticalFlowPyrLK will decide that an existing 
        // (previous) feature has been found again in this frame, but a substantial distance from its location in
        // the previous frame. And do this jump for several features in the same frame.
        // We de-select features with an excessive movement.
        //
        // Calculate the cutoff and mark points moving faster than that as insignificant.
        public static int MarkVeryFastObjectsInsignificant(
            int previousMedianMovePx,
            FlowObjectList significantObjects,
            Emgu.CV.Util.VectorOfPointF prevPoints,
            Emgu.CV.Util.VectorOfPointF currPoints,
            Emgu.CV.Util.VectorOfByte statuses)
        {
            try
            {
                // There should be 30 to 40 points in the list
                int points = significantObjects.Count;
                if (points < 10)
                    return UnknownValue;

                Assert(prevPoints.Size == points, "FlowProcessModel.MarkVeryFastObjectsInsignificant: prevPoints bad");
                Assert(currPoints.Size == points, "FlowProcessModel.MarkVeryFastObjectsInsignificant: currPoints bad");
                Assert(statuses.Size == points, "FlowProcessModel.MarkVeryFastObjectsInsignificant: statuses bad");

                // We use simple distance metric: the sum of DeltaX and DeltaY
                List<int> deltas = new();
                for (int index = 0; index < points; index++)
                    if (statuses[index] == 1)
                        deltas.Add((int)(
                            Math.Abs(currPoints[index].X - prevPoints[index].X) +
                            Math.Abs(currPoints[index].Y - prevPoints[index].Y)));

                if (deltas.Count == 0)
                    return UnknownValue;

                deltas.Sort();
                int medianDeltaPx = deltas[deltas.Count / 2];

                // Don't let the cut off grow more than 50% per step.            
                if ((previousMedianMovePx > 0) && (medianDeltaPx > 1.5 * previousMedianMovePx))
                    medianDeltaPx = (int)(1.5 * previousMedianMovePx);
                if (medianDeltaPx < 10)
                    medianDeltaPx = 10;

                // Cut off any features moving at more than 2x the median speed
                int cutoffDeltaPx = 2 * medianDeltaPx;
                Assert(cutoffDeltaPx > 0, "FlowProcessModel.MarkVeryFastObjectsInsignificant: CutoffDeltas bad");

                for (int index = 0; index < points; index++)
                    if (statuses[index] == 1)
                        if (Math.Abs(currPoints[index].X - prevPoints[index].X) +
                            Math.Abs(currPoints[index].Y - prevPoints[index].Y) > cutoffDeltaPx)
                            significantObjects[index].Significant = false;

                return medianDeltaPx;
            }
            catch (Exception ex)
            {
                throw ThrowException("FlowProcessModel.MarkVeryFastObjectsInsignificant", ex);
            }
        }


        // Use CalcOpticalFlowPyrLK (aka Optical Flow) process to detect
        // movements of the GFTT features between the images.
        public int ProcessBlock(
            ProcessScope scope,
            Image<Gray, byte> PrevGray,
            Image<Gray, byte> CurrGray)
        {
            try
            {
                var significantObjects = FlowObjects.SignificantList();
                var numSignificantObjects = significantObjects.Count;
                if (numSignificantObjects > 0)
                {
                    var prevPoints = significantObjects.PrevPoints();

                    // CalcOpticalFlowPyrLK throws exception if prevPoints is empty.
                    // Case activeFeatures=10 and prevPoints.Size=0 can occur after a DetectGfttAndAddPoints call.
                    if (prevPoints.Size > 0)
                    {
                        var currPoints = new Emgu.CV.Util.VectorOfPointF();

                        var statuses = new Emgu.CV.Util.VectorOfByte();
                        var errors = new Emgu.CV.Util.VectorOfFloat();

                        // Specifies when the iteration process of finding the flow for each point on each pyramid level should be stopped.
                        var criteria = new MCvTermCriteria(
                            ProcessConfig.FlowMaxIterations, 0.03); // Maximum iterations, epsilon

                        var searchWindow = new Size(
                            ProcessConfig.FlowSearchWindow, ProcessConfig.FlowSearchWindow);


                        // Implements sparse iterative version of Lucas-Kanade optical flow in pyramids ([Bouguet00]). 
                        // It calculates coordinates of the feature points on the current video frame given their 
                        // coordinates on the previous frame. The function finds the coordinates with sub-pixel accuracy.
                        // Refer https://www.emgu.com/wiki/files/4.5.1/document/html/24d49782-e45c-3e24-dbe1-61e1897328ef.htm
                        CvInvoke.CalcOpticalFlowPyrLK(
                            PrevGray,               // First frame, at time t.
                            CurrGray,               // Second frame, at time t + dt 
                            prevPoints,             // Array of points for which the flow needs to be found.
                            currPoints,             // Array of 2D points containing calculated new positions of input
                            statuses,               // Array element is set to 1 if the flow for the corresponding feature has been found, 0 otherwise.
                            errors,                 // Array of double numbers containing difference between patches around the original and moved points. Optional parameter; can be NULL
                            searchWindow,           // Size of the search window of each pyramid level.
                            ProcessConfig.FlowMaxPyramid,  // Maximal pyramid level number. If 0, pyramids are not used (single level), if 1, two levels are used, etc.
                            criteria,               // Specifies when the iteration process of finding the flow for each point on each pyramid level should be stopped.
                            LKFlowFlag.Default,     // Miscellaneous flags
                            ProcessConfig.FlowMinEigThreshold); // MinEigThreshold:
                                                                // The algorithm calculates the minimum eigen value of a 2x2 normal matrix of optical
                                                                // flow equations (this matrix is called a spatial gradient matrix in [Bouguet00]),
                                                                // divided by number of pixels in a window; if this value is less than minEigThreshold,
                                                                // then a corresponding feature is filtered out and its flow is not processed, so
                                                                // it allows to remove bad points and get a performance boost.

                        Assert(prevPoints.Size == currPoints.Size,
                            "FlowProcessModel.ProcessBlock: prevPoints.Size=" + prevPoints.Size + " currPoints.Size=" + currPoints.Size);

                        // Calculate the cutoff and mark points moving faster than that as insignificant.
                        var currBlock = FlowBlocks.LastBlock as FlowBlock;
                        currBlock.MedianMovePx =
                            MarkVeryFastObjectsInsignificant(
                                (FlowBlocks[currBlock.BlockId-1] as FlowBlock).MedianMovePx,
                                significantObjects, prevPoints, currPoints, statuses);

                        // Update Features with flow information
                        for (int i = 0; i < prevPoints.Size; i++)
                        {
                            var thisObject = significantObjects[i];

                            if (statuses[i] == 0)
                            {
                                // Feature can not be found this frame.
                                thisObject.Significant = false;
                                numSignificantObjects--;
                            }
                            else if (!thisObject.Significant)
                            {
                                // Feature has moved very fast - more than is sensible.
                                numSignificantObjects--;
                            }
                            else
                            {
                                // Debugging - Set breakpoint on assignment.
                                if ((ProcessConfig.FocusObjectId != 0) && (thisObject.ObjectId == ProcessConfig.FocusObjectId))
                                    thisObject = significantObjects[i];

                                float currPointY = currPoints[i].Y;
                                float currPointX = currPoints[i].X;

                                ObjectClaimsNewFeature(thisObject, currBlock, currPointX, currPointY);
                            }
                        }
                    }
                }

                return numSignificantObjects;
            }
            catch (Exception ex)
            {
                throw ThrowException("FlowProcessModel.ProcessBlock" +
                    "(CurrBlockId=" + scope.PSM.CurrBlockId +
                    ",LastBlockId=" + scope.PSM.LastBlockId + ")", ex);
            }
        }


        public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "# Blocks", FlowBlocks.Count },
                { "# Features", FlowFeatures.Count},
                { "# Objects", FlowObjects.Count},
            };
        }

    };

}
