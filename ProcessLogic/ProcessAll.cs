// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;



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


        public ProcessConfigModel ProcessConfig;

        // List of Blocks (aka frames processed)  
        public ProcessBlockList Blocks { get; private set; }

        // List of features found. Each has a bounding retangle
        public ProcessFeatureList ProcessFeatures { get; private set; }
        // List of logical objects found - derived from overlapping features over successive frames. 
        public ProcessObjList ProcessObjects { get; private set; }
        // List of ProcessSpans that analsyse ProcessObjects to generate FixAltM data
        public ProcessSpanList ProcessSpans { get; set; }


        // Hooks for testing 
        public event ObservationHandler<ProcessAll> Observation;


        // If !UseFlightLegs, we need to generate ProcessSpans for each cluster of significant objects (over a series of FlightSteps)
        public int FlightSteps_PrevSigObjects { get; set; }
        public int FlightSteps_MinStepId { get; set; }
        public int FlightSteps_MaxStepId { get; set; }



        public ProcessAll(GroundData groundData, VideoData video, Drone drone, ProcessConfigModel config)
        {
            GroundData = groundData;
            VideoData = video;
            Drone = drone;

            ProcessConfig = config;
            Blocks = new();
            ProcessFeatures = new(config);
            ProcessObjects = new();
            ProcessSpans = new();
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


        public virtual void EnsureObjectsNamed() { }


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
                    theObj.SetName(sigObjects);
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


        public void OnObservation(ProcessEventEnum processEvent, ProcessEventArgs args = null)
        {
            if (args == null)
                args = new ProcessEventArgs(null, 0);

            Observation?.Invoke(this, processEvent, args);
        }


        // Reset the process model, ready for a process run to start
        public virtual void RunStart()
        {
            ProcessObject.NextObjectId = 0;
            ProcessFeature.NextFeatureId = 0;

            Blocks.Clear();
            ProcessFeatures = new(ProcessConfig);
            ProcessObjects.Clear();
            ProcessObject.NextObjectId = 0;
            ProcessSpans.Clear();
            ResetSpanData();
        }


        // A new drone flight leg has started.
        protected virtual void ProcessFlightLegStart(ProcessScope scope, int legId)
        {
        }


        public void ProcessFlightLegStartWrapper(ProcessScope scope, int legId)
        {
            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegStart_Before, new ProcessEventArgs(scope, legId));

            ProcessFlightLegStart(scope, legId);

            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegStart_After, new ProcessEventArgs(scope, legId));
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
                    // Post process the objects found in the leg & maybe set FlightLegs.FixAltM 
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
                    OnObservation(ProcessEventEnum.LegEnd_After, new ProcessEventArgs(scope, legId));

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
            Drone.DisplayVideo?.ResetCurrFrame();

            OnObservation(ProcessEventEnum.IntervalEnd);
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
