// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;


namespace SkyCombImage.ProcessLogic
{
    public enum ProcessEventEnum
    {
        ProcessStart,
        LegStart_Before,
        LegStart_After,
        LegEnd_Before,
        LegEnd_After,
        ProcessEnd
    }


    public delegate void ObservationHandler<T>(T sender, ProcessEventEnum processEvent, EventArgs e) where T : class;


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
        public ProcessBlockList Blocks { get; set; }

        // List of features found. Each has a bounding retangle
        public ProcessFeatureList ProcessFeatures { get; set; }


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

            ProcessObject.NextObjectId = 0;
        }


        public virtual void EnsureObjectsNamed() { }


        public void OnObservation(ProcessEventEnum processEvent, EventArgs args)
        {
            Observation?.Invoke(this, processEvent, args);
        }


        // Reset the process model, ready for a process run to start
        protected virtual void ProcessStart()
        {
            Blocks.Clear();
            ProcessFeatures = new(ProcessConfig);
            ProcessObject.NextObjectId = 0;
        }


        public void ProcessStartWrapper()
        {
            ProcessStart();
            OnObservation(ProcessEventEnum.ProcessStart, EventArgs.Empty);
        }


        // A new drone flight leg has started.
        protected virtual void ProcessFlightLegStart(int LegId) 
        {
        }


        public void ProcessFlightLegStartWrapper(int LegId)
        {
            if (Drone.UseFlightLegs)
                Observation?.Invoke(this, ProcessEventEnum.LegStart_Before, EventArgs.Empty);

            ProcessFlightLegStart(LegId);

            if (Drone.UseFlightLegs)
                Observation?.Invoke(this, ProcessEventEnum.LegStart_After, EventArgs.Empty);
        }


        // A drone flight leg has finished. 
        protected virtual void ProcessFlightLegEnd(int LegId) { }


        public void ProcessFlightLegEndWrapper(int LegId)
        {
            if (Drone.UseFlightLegs)
                Observation?.Invoke(this, ProcessEventEnum.LegEnd_Before, EventArgs.Empty);

            ProcessFlightLegEnd(LegId);

            if (Drone.UseFlightLegs)
                Observation?.Invoke(this, ProcessEventEnum.LegEnd_After, EventArgs.Empty);
        }


        // The process run has ended
        protected virtual void ProcessEnd()
        {
            EnsureObjectsNamed();
        }


        public void ProcessEndWrapper()
        {
            EnsureObjectsNamed();

            OnObservation(ProcessEventEnum.ProcessEnd, EventArgs.Empty);
        }


        // A drone flight leg has finished &/or started. 
        public void ProcessFlightLegStartAndEnd(int prevLegId, int currLegId)
        {
            if ((prevLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegEndWrapper(prevLegId);

            if ((currLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegStartWrapper(currLegId);
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
