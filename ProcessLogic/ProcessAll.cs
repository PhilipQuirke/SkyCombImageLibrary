// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;


namespace SkyCombImage.ProcessLogic
{
    // Hook types
    public enum ProcessEventEnum
    {
        RunStart,
        IntervalStart,
        LegStart_Before,
        LegStart_After,
        LegEnd_Before,
        LegEnd_After,
        IntervalEnd,
        RunEnd
    }


    // Hook custom event data
    public class ProcessEventArgs : EventArgs
    {
        public ProcessScope Scope { get; private set; }
        public int LegId { get; private set; }

        public ProcessEventArgs(ProcessScope scope, int legId)
        {
            Scope = scope;
            LegId = legId;
        }
    }


    // Hook signature
    public delegate void ObservationHandler<T>(T sender, ProcessEventEnum processEvent, ProcessEventArgs e) where T : class;


    // All processing models derive from this class.
    public class ProcessAll : BaseConstants
    {
        // Ground (DEM, DSM and Swathe) data under the drone flight path
        public GroundData GroundData { get; set; }

        // The main input video being processed
        public VideoData VideoData { get; }

        public Drone Drone { get; }

        public RunUserInterface RunUI { get; }


        public ProcessConfigModel ProcessConfig;

        // List of Blocks (aka frames processed)  
        public ProcessBlockList Blocks { get; private set; }

        // List of features found. Each has a bounding retangle
        public ProcessFeatureList ProcessFeatures { get; private set; }
        // List of logical objects found - derived from overlapping features over successive frames. 
        public ProcessObjList ProcessObjects { get; private set; }
        // List of ProcessSpans that analsyse ProcessObjects to generate FixAltM/FixYawDeg/FixPitchDeg data
        public ProcessSpanList ProcessSpans { get; set; }


        // Hooks for testing 
        public event ObservationHandler<ProcessAll> Observation;


        // If UseFlightLegs, how many significant objects have been found in this FlightLeg?
        public int FlightLeg_SigObjects { get; set; }

        // If !UseFlightLegs, we need to generate ProcessSpans for each cluster of significant objects (over a series of FlightSteps)
        public int FlightSteps_PrevSigObjects { get; set; }
        public int FlightSteps_MinStepId { get; set; }
        public int FlightSteps_MaxStepId { get; set; }

        public int NumInsignificantObjects { get; set; } = 0;


        public ProcessAll(GroundData groundData, VideoData video, Drone drone, ProcessConfigModel config, RunUserInterface runUI)
        {
            GroundData = groundData;
            VideoData = video;
            Drone = drone;
            RunUI = runUI;

            ProcessConfig = config;
            Blocks = new();
            ProcessFeatures = new(config);
            ProcessObjects = new();
            ProcessSpans = new();
            FlightLeg_SigObjects = 0;
            ResetSpanData();

            ProcessObject.NextObjectId = 0;
            ProcessFeature.NextFeatureId = 0;
        }


        protected void ResetSpanData()
        {
            FlightSteps_PrevSigObjects = 0;
            FlightSteps_MinStepId = UnknownValue;
            FlightSteps_MaxStepId = UnknownValue;
        }


        // Ensure each object has at least an "insignificant" name e.g. #16
        public virtual void EnsureObjectsNamed()
        {
            ProcessObjects.EnsureObjectsNamed(this);
        }


        // Ensure that significant objects, in the current FlightStep, have a name
        protected int EnsureObjectsNamed(int sigObjects, ProcessObjList theObjects, FlightStep? currRunFlightStep)
        {
            foreach (var theObject in theObjects)
            {
                var theObj = theObject.Value;
                if ((theObj.FlightLegId > 0) &&
                   ((currRunFlightStep == null) || (theObj.FlightLegId == currRunFlightStep.FlightLegId)) &&
                   (theObj.Significant) &&
                   (theObj.Name == ""))
                {
                    sigObjects++;
                    theObj.SetName(theObj.FlightLegName, sigObjects);
                }
            }

            return sigObjects;
        }


        // Add a new block
        public ProcessBlock AddBlock(ProcessScope scope)
        {
            var currBlock = ProcessFactory.NewBlock(scope);

            Blocks.AddBlock(currBlock, scope, Drone);

            return currBlock;
        }


        public virtual ProcessFeature? NewPersistFeature() { return null; }


        // Create an unreal feature, with no pixels, with a rectangle calculated from the object's 
        // last bounding rectangle, maximum pixel box, and the average object movement.
        public void AddPersistFeature(ProcessObject theObject)
        {
            var theBlock = Blocks.LastBlock;

            var theFeature = NewPersistFeature();
            theFeature.PixelBox = theObject.ExpectedLocationThisBlock();
            ProcessFeatures.AddFeature(theFeature);

            Assert(Blocks.Count == theFeature.Block.BlockId, "AddPersistFeature: Bad Blocks count");

            theObject.ClaimFeature(theFeature);
        }


        public void OnObservation(ProcessEventEnum processEvent, ProcessEventArgs args = null)
        {
            if (args == null)
                args = new ProcessEventArgs(null, 0);

            Observation?.Invoke(this, processEvent, args);
        }


        // Do pre-run processing 
        public virtual void PreRunStart(ProcessScope scope)
        {
        }


        // Reset the process model 
        public virtual void RunStart(ProcessScope scope)
        {
            ProcessObject.NextObjectId = 0;
            ProcessFeature.NextFeatureId = 0;

            Blocks.Clear();
            ProcessFeatures = new(ProcessConfig);
            ProcessObjects.Clear();
            ProcessObject.NextObjectId = 0;
            ProcessSpans.Clear();
            FlightLeg_SigObjects = 0;
            ResetSpanData();
        }


        // A new drone flight leg has started.
        protected virtual void ProcessFlightLegStart(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                ProcessObjects.StopTracking();
                FlightLeg_SigObjects = 0;
            }
        }


        public void ProcessFlightLegStartWrapper(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegStart_Before, new ProcessEventArgs(scope, legId));

            ProcessFlightLegStart(scope, legId);

            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegStart_After, new ProcessEventArgs(scope, legId));
        }


        // Process the features found in the current block/frame, which is part of a leg,
        // by preference adding them to existing objects (created in previous blocks/frames),
        // else creating new objects to hold the features.
        public (ProcessObjList, ProcessObjList, ProcessFeatureList) ProcessBlockForObjects_Core(ProcessScope scope, ProcessFeatureList featuresInBlock)
        {
            int Phase = 0;

            try
            {
                Phase = 1;
                var currBlock = Blocks.LastBlock;
                int blockID = currBlock.BlockId;


                // We only want to consider objects that are active.
                // For long flights most objects will have become inactive seconds or minutes ago.
                Phase = 2;
                ProcessObjList inScopeObjects = new();
                ProcessObjList availObjects = new();
                foreach (var theObject in ProcessObjects)
                {
                    var thisObject = theObject.Value;
                    if ((thisObject.LastFeature != null) &&
                        (thisObject.LastFeature.Block.BlockId == blockID - 1) &&
                        thisObject.VaguelySignificant())
                    {
                        inScopeObjects.AddObject(thisObject);
                        availObjects.AddObject(thisObject);
                    }
                }

                // Each feature can only be claimed once. Clone the list (not the features).
                ProcessFeatureList availFeatures = featuresInBlock.Clone();


                // For each existing active object, consider each feature (significant or not)
                // found in this frame to see if the pixel box overlaps.
                // This privileges objects with multiple real features,
                // as they can more accurately estimate their expected location.
                // We privilege objects with a "real" last feature over objects with a "unreal" last feature.
                Phase = 3;
                for (int pass = 0; pass < 2; pass++)
                    foreach (var theObject in inScopeObjects)
                    {
                        var thisObject = theObject.Value;
                        var lastFeat = thisObject.LastFeature;
                        if ((lastFeat.Block.BlockId == blockID - 1) &&
                            (pass == 0 ? lastFeat.Type == FeatureTypeEnum.Real : lastFeat.Type != FeatureTypeEnum.Unreal))
                        {
                            // If one or more features overlaps the object's expected location,
                            // claim ownership of the feature(s), and mark them as Significant.
                            var expectedObjectLocation = thisObject.ExpectedLocationThisBlock();

                            bool claimedFeatures = false;
                            foreach (var feature in featuresInBlock)
                            {
                                if (feature.Value.ObjectId > 0)
                                    // Feature has already been claimed.
                                    continue;

                                // Object will claim feature if the object remains viable after claiming feature
                                if (thisObject.MaybeClaimFeature(feature.Value, expectedObjectLocation))
                                {
                                    availFeatures.Remove(feature.Value.FeatureId);
                                    claimedFeatures = true;

                                    // With this process, need to claim multiple features per object per block
                                    // break;
                                }
                            }
                            if (claimedFeatures)
                                availObjects.Remove(thisObject.ObjectId);
                        }
                    }

                // An active object with exactly one real feature can't estimate its expected location at all.
                // An active object with two features has a lot of wobble in its expected movement/location.
                // If the object is moving in the image quickly, the object location
                // and feature location will not overlap. Instead the feature will (usually)
                // be vertically below the object estimated location.
                Phase = 4;
                foreach (var theObject in availObjects)
                {
                    var thisObject = theObject.Value;
                    if (thisObject.NumRealFeatures() <= 2)
                    {
                        // If one or more features overlaps the object's expected location,
                        // claim ownership of the feature(s), and mark them as Significant.
                        var expectedObjectLocation = thisObject.ExpectedLocationThisBlock();

                        // Search higher in the image 
                        expectedObjectLocation = new System.Drawing.Rectangle(
                            expectedObjectLocation.X,
                            expectedObjectLocation.Y + ProcessConfigModel.SearchHigherPixels, // Higher
                            expectedObjectLocation.Width,
                            expectedObjectLocation.Height);

                        foreach (var feature in availFeatures)
                            thisObject.MaybeClaimFeature(feature.Value, expectedObjectLocation);
                    }
                }

                Phase = 5;
                currBlock.AddFeatureList(featuresInBlock);
                ProcessFeatures.AddFeatureList(featuresInBlock);

                // For each active object, where the above code did not find an 
                // overlapping feature in this Block, if it is worth continuing tracking...
                Phase = 6;
                foreach (var theObject in inScopeObjects)
                {
                    var thisObject = theObject.Value;
                    if (thisObject.BeingTracked &&
                       (thisObject.LastRealFeatureId > 0) &&
                       (thisObject.LastRealFeature.Block.BlockId < blockID) &&
                       thisObject.KeepTracking(blockID))
                        // ... persist this object another Block. Create an unreal feature, with no pixels, with a rectangle   
                        // calculated from the object's last bounding rectangle and the average frame movement.
                        AddPersistFeature(thisObject);
                }

                return (inScopeObjects, availObjects, availFeatures);
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessBlockForObjects.Phase=" + Phase, ex);
            }
        }


        // A drone flight leg has finished. 
        protected virtual void ProcessFlightLegEnd(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
            {
                // For process robustness, we want to process each leg independently.
                // So at the start and end of each leg we stop tracking all objects.
                ProcessObjects.StopTracking();

                // If we are lacking the current ProcessSpan then create it.
                if ((legId > 0) && !ProcessSpans.TryGetValue(legId, out _))
                {
                    // Post process the objects found in the leg & maybe set FlightLegs.FixAltM/FixYawDeg/FixPitchDeg 
                    var theSpan = ProcessFactory.NewProcessSpan(this, legId);
                    ProcessSpans.AddSpan(theSpan);
                    theSpan.CalculateSettings_from_FlightLeg();
                    theSpan.AssertGood();
                }
            }

            EnsureObjectsNamed();
        }


        public void ProcessFlightLegEndWrapper(ProcessScope scope, int legId)
        {
            try
            {
                if (Drone.UseFlightLegs)
                    OnObservation(ProcessEventEnum.LegEnd_Before, new ProcessEventArgs(scope, legId));

                ProcessFlightLegEnd(scope, legId);

                if (Drone.UseFlightLegs)
                {
                    // After the leg "FixAltM/FixYawDeg/FixPitchDeg" calculations have been done and object locations updated,
                    // we check that the objects are not more than ProcessConfig.ObjectMaxRangeM metres from the drone.
                    foreach (var theObject in ProcessObjects)
                        if (theObject.Value.Significant && (theObject.Value.AvgRangeM > ProcessConfigModel.ObjectMaxRangeM))
                            theObject.Value.Significant = false;

                    OnObservation(ProcessEventEnum.LegEnd_After, new ProcessEventArgs(scope, legId));
                }

                BaseConstants.GcFreeMemory();
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessFlightLegEndWrapper", ex);
            }
        }


        // If we have been tracking some significant objects, create a ProcessSpan for them
        public void ProcessSpan_Create()
        {
            if ((!Drone.UseFlightLegs) && (FlightSteps_PrevSigObjects > 0))
            {
                var theSpan = ProcessFactory.NewProcessSpan(this, ProcessSpans.Count() + 1);
                ProcessSpans.AddSpan(theSpan);
                theSpan.CalculateSettings_from_FlightSteps(FlightSteps_MinStepId, FlightSteps_MaxStepId);
                ResetSpanData();
            }
        }


        // The process run has ended
        public void EndInterval()
        {
            EnsureObjectsNamed();

            // If we have been tracking some significant objects, create a ProcessSpan for them
            ProcessSpan_Create();

            Drone.InputVideo?.ResetCurrFrame();

            OnObservation(ProcessEventEnum.IntervalEnd);
        }


        // If !UseFlightLegs, track significant objects, based on their corresponding FlightSteps.
        // Does not make use of FlightLegs.
        protected void ProcessObjectsFlightSteps(ProcessObjList inScopeObjects, ProcessBlock currBlock)
        {
            if (!Drone.UseFlightLegs)
            {
                inScopeObjects.EnsureObjectsNamed(this);

                int sigObjects = inScopeObjects.NumSignificantObjects;
                if (sigObjects > 0)
                {
                    if (FlightSteps_PrevSigObjects == 0)
                        // PROCESS SPAN START EVENT
                        // Object(s) have just become significant. Last frame there were no significant objects.
                        // It takes a few steps to become significant, so get the minimal FlightStep.StepID of the object(s).
                        // ProcessSpan_MinFlightStepId is the starting step of a future ProcessSpan object.
                        FlightSteps_MinStepId = inScopeObjects.GetMinStepId();

                    FlightSteps_PrevSigObjects = Math.Max(FlightSteps_PrevSigObjects, sigObjects);
                    FlightSteps_MaxStepId = currBlock.FlightStepId;
                }
                else
                {
                    // We have no significant objects 

                    if (FlightSteps_PrevSigObjects > 0)
                    {
                        // We have unprocessed significant objects from previous blocks. 

                        if (currBlock.FlightStepId - FlightSteps_MaxStepId > 8)
                        {
                            // PROCESS SPAN END EVENT
                            // We tracked some significant objects, then they all became insignificant, and 8 frames have passed
                            ProcessSpan_Create();

                            ResetSpanData();
                        }
                    }
                }
            }
        }


        // A drone flight leg has finished &/or started. 
        public void ProcessFlightLegStartAndEnd(ProcessScope scope, int prevLegId, int currLegId)
        {
            if ((prevLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegEndWrapper(scope, prevLegId);

            if ((currLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegStartWrapper(scope, currLegId);
        }


        public virtual DataPairList GetSettings()
        {
            int numHotPixels = 0;
            foreach (var feature in ProcessFeatures)
                numHotPixels += feature.Value.NumHotPixels;

            return new DataPairList
            {
                { "# Blocks", Blocks.Count },
                { "# Objects", ProcessObjects.Count },
                { "# Significant Objects", ProcessObjects.NumEverSignificantObjects },
                { "# Features", ProcessFeatures.Count },
                { "# Significant Features", ProcessFeatures.NumSig },
                { "# Hot Pixels", numHotPixels },
            };
        }
    };


    // A class to hold the location and heat of a pixel 
    public class PixelHeat
    {
        // The BlockId that this pixel was detected in
        public int BlockId { get; } = 0;
        // The FeatureId (if any) associated with this pixel
        public int FeatureId { get; } = 0;
        public int Y { get; } = 0;
        public int X { get; } = 0;
        public int Heat { get; } = 0;


        public PixelHeat(int blockId, int featureId, int y, int x, int heat = 0)
        {
            BlockId = blockId;
            FeatureId = featureId;
            Y = y;
            X = x;
            Heat = heat;
        }


        public virtual DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Block", BlockId },
                { "Feature", FeatureId },
                { "Y", Y},
                { "X", X },
                { "Heat", Heat },
            };
        }
    };


    public class PixelHeatList : List<PixelHeat>
    {
        public PixelHeatList() : base()
        {
        }
    };
}
