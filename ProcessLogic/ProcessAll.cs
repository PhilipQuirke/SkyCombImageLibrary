// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using Emgu.CV.XFeatures2D;


namespace SkyCombImage.ProcessLogic
{
    // All processing models derive from this class.
    public class ProcessAll : BaseConstants
    {
        public static ProcessConfigModel ProcessConfig = null;

        // The main input video being processed
        public VideoData VideoData { get; }

        public Drone Drone { get; }


        public ProcessAll(ProcessConfigModel config, VideoData video, Drone drone)
        {
            ProcessConfig = config;
            VideoData = video;
            Drone = drone;

            ProcessObject.NextObjectId = 0;
        }


        // Reset any internal state of the model, so it can be re-used in another run immediately
        public virtual void ResetModel()
        {
            ProcessObject.NextObjectId = 0;
        }


        // A new drone flight leg has started.
        public virtual void ProcessLegStart(int LegId) { }


        // A drone flight leg has finished. 
        public virtual void ProcessLegEnd(int LegId) { }


        public virtual void EnsureObjectsNamed() { }



        // A drone flight leg has finished &/or started. 
        public void ProcessLegStartAndEnd(int prevLegId, int currLegId)
        {
            if ((prevLegId > 0) && (prevLegId != currLegId))
                ProcessLegEnd(prevLegId);

            if ((currLegId > 0) && (prevLegId != currLegId))
                ProcessLegStart(currLegId);
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
