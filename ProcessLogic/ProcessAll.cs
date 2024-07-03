// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;


namespace SkyCombImage.ProcessLogic
{
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

        // List of comb features found. Each feature is a cluster of hot pixels, with a bounding retangle
        public ProcessFeatureList ProcessFeatures { get; set; }



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


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public virtual void ResetModel()
        {
            Blocks.Clear();
            ProcessFeatures = new(ProcessConfig);
            ProcessObject.NextObjectId = 0;
        }


        // A new drone flight leg has started.
        public virtual void ProcessFlightLegStart(int LegId) { }


        // A drone flight leg has finished. 
        public virtual void ProcessFlightLegEnd(int LegId) { }


        public virtual void EnsureObjectsNamed() { }



        // A drone flight leg has finished &/or started. 
        public void ProcessFlightLegStartAndEnd(int prevLegId, int currLegId)
        {
            if ((prevLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegEnd(prevLegId);

            if ((currLegId > 0) && (prevLegId != currLegId))
                ProcessFlightLegStart(currLegId);
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
