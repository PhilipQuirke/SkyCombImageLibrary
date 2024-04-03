// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    public class ProcessFactory : BaseConstants
    {
        public static ProcessBlock NewBlock(ProcessScope scope)
        {
            return new ProcessBlock(scope);
        }


        public static ProcessBlock NewBlock(int blockId, List<string> settings, Drone drone)
        {
            return new ProcessBlock(blockId, settings, drone);
        }


        public static FlowBlock NewFlowBlock(ProcessScope scope)
        {
            return new FlowBlock(scope);
        }


        public static FlowProcessAll NewFlowProcessModel(ProcessConfigModel config, Drone drone)
        {
            return new FlowProcessAll(config, drone);
        }


        public static CombProcessAll NewCombProcessModel(ProcessConfigModel config, VideoData videoData, GroundData groundData, Drone drone)
        {
            return new CombProcessAll(config, videoData, groundData, drone);
        }


        public static CombFeatureList NewCombFeatureList()
        {
            return new CombFeatureList();
        }


        public static CombFeature NewCombFeature(CombProcessAll model, List<string> settings)
        {
            return new CombFeature(model, settings);
        }


        public static CombObject NewCombObject(CombProcessAll model, List<string> settings)
        {
            return new CombObject(model, settings);
        }


        public static CombObject NewCombObject(ProcessScope scope, CombProcessAll model, CombFeature firstFeature)
        {
            return new CombObject(scope, model, firstFeature);
        }


        public static CombSpan NewCombSpan(CombProcessAll model, int legId, List<string>? settings = null)
        {
             return new CombSpan(model, legId, settings);
        }
    }
}
