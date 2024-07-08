// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;


namespace SkyCombImage.ProcessLogic
{
    // Hook types
    public enum ProcessEventEnum
    {
        ProcessStart,
        LegStart_Before,
        LegStart_After,
        LegEnd_Before,
        LegEnd_After,
        ProcessEnd
    }


    // Hook custom event data
    public class ProcessEventArgs : EventArgs
    {
        public ProcessScope Scope { get; private set; }
        public int LegId{ get; private set; }

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


        // Hooks for testing 
        public event ObservationHandler<ProcessAll> Observation;


        public ProcessAll(GroundData groundData, VideoData video, Drone drone, ProcessConfigModel config)
        {
            GroundData = groundData;
            VideoData = video;
            Drone = drone;

            ProcessConfig = config;
            Blocks = new();
            ProcessFeatures = new(config);
            ProcessObjects = new();

            ProcessObject.NextObjectId = 0;
        }


        public virtual void EnsureObjectsNamed() { }


        // Ensure that significant objects, in the current FlightStep, have a name
        protected int EnsureObjectsNamed(int sigObjects, ProcessObjList theObjects, FlightStep? currRunFlightStep)
        {
            foreach (var theObject in theObjects)
                if ((theObject.Value.FlightLegId > 0) &&
                   ((currRunFlightStep == null) || (theObject.Value.FlightLegId == currRunFlightStep.FlightLegId)) &&
                   (theObject.Value.Significant) &&
                   (theObject.Value.Name == ""))
                {
                    sigObjects++;
                    theObject.Value.SetName(sigObjects);
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
            if( args == null)
                args = new ProcessEventArgs(null, 0);

            Observation?.Invoke(this, processEvent, args);
        }


        // Reset the process model, ready for a process run to start
        protected virtual void ProcessStart()
        {
            Blocks.Clear();
            ProcessFeatures = new(ProcessConfig);
            ProcessObjects.Clear();
            ProcessObject.NextObjectId = 0;
        }


        public void ProcessStartWrapper()
        {
            ProcessStart();
            OnObservation(ProcessEventEnum.ProcessStart);
        }


        // A new drone flight leg has started.
        protected virtual void ProcessFlightLegStart(ProcessScope scope, int LegId) 
        {
        }


        public void ProcessFlightLegStartWrapper(ProcessScope scope, int LegId)
        {
            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegStart_Before, new ProcessEventArgs(scope, LegId));

            ProcessFlightLegStart(scope, LegId);

            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegStart_After, new ProcessEventArgs(scope, LegId));
        }


        // A drone flight leg has finished. 
        protected virtual void ProcessFlightLegEnd(ProcessScope scope, int LegId) { }


        public void ProcessFlightLegEndWrapper(ProcessScope scope, int LegId)
        {
            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegEnd_Before, new ProcessEventArgs(scope, LegId));

            ProcessFlightLegEnd(scope, LegId);

            if (Drone.UseFlightLegs)
                OnObservation(ProcessEventEnum.LegEnd_After, new ProcessEventArgs(scope, LegId));
        }


        // The process run has ended
        protected virtual void ProcessEnd()
        {
            EnsureObjectsNamed();
        }


        public void ProcessEndWrapper()
        {
            EnsureObjectsNamed();

            OnObservation(ProcessEventEnum.ProcessEnd);
        }


        // A drone flight leg has finished &/or started. 
        public void ProcessFlightLegStartAndEnd(ProcessScope scope, int prevLegId, int currLegId)
        {
            if ((prevLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegEndWrapper(scope, prevLegId);

            if ((currLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegStartWrapper(scope, currLegId);
        }


        virtual public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "# Blocks", Blocks.Count },
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


        virtual public DataPairList GetSettings()
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
