// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using SkyCombImage.ProcessModel;
using System;


namespace SkyCombImage.ProcessLogic
{
    // A significant object - a logical object derived from overlapping features over successive frames. 
    public class ProcessObject : ProcessObjectModel
    {
        // Static config data shared by all objects
        public static ProcessConfigModel Config = null;
        // Static NextObjectID shared by all objects
        public static int NextObjectId = 0;
        // Static random number generator
        protected static readonly RNG Rng = new();


        public ProcessObject(ProcessScope scope) : base()
        {
            ObjectId = ++NextObjectId;
            if (scope != null)
            {
                LegId = scope.PSM.CurrRunLegId;
                RunFromVideoS = (float)(scope.PSM.CurrInputFrameMs / 1000.0);
                RunToVideoS = RunFromVideoS;
            }
        }


        // Is this object in the RunFrom/To scope?
        public bool InRunScope(ProcessScope scope)
        {
            var maxFromMs = Math.Max(RunFromVideoS * 1000, scope.PSM.FirstVideoFrameMs);
            var minToMs = Math.Min(RunToVideoS * 1000, scope.PSM.LastVideoFrameMs);

            var overlapMs = minToMs - maxFromMs;
            if (overlapMs <= 0)
                return false;

            // To cope with edge cases, returns true if > 50% of the objects duration is within the RunFrom/To scope
            var durationMs = RunToVideoS * 1000 - RunFromVideoS * 1000;
            return overlapMs / durationMs >= 0.5;
        }
    }
}
