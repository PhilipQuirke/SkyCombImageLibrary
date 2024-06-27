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


        public static FlowBlock NewFlowBlock(FlowProcess flowProcess, ProcessScope scope)
        {
            return new FlowBlock(flowProcess, scope);
        }


        public static FlowProcess NewFlowProcessModel(ProcessConfigModel config, Drone drone)
        {
            return new FlowProcess(config, drone);
        }


        public static YoloProcess NewYoloProcessModel(ProcessConfigModel config, Drone drone, string yoloDirectory)
        {
            return new YoloProcess(config, drone, yoloDirectory);
        }


        public static CombProcess NewCombProcessModel(ProcessConfigModel config, VideoData videoData, GroundData groundData, Drone drone)
        {
            return new CombProcess(config, videoData, groundData, drone);
        }


        public static CombFeatureList NewCombFeatureList(ProcessConfigModel config)
        {
            return new CombFeatureList(config);
        }


        public static CombFeature NewCombFeature(CombProcess model, List<string> settings)
        {
            return new CombFeature(model, settings);
        }


        public static CombObject NewCombObject(CombProcess model, List<string> settings)
        {
            return new CombObject(model, settings);
        }


        public static CombObject NewCombObject(ProcessScope scope, CombProcess model, CombFeature firstFeature)
        {
            return new CombObject(scope, model, firstFeature);
        }


        public static CombSpan NewCombSpan(CombProcess model, int legId, List<string>? settings = null)
        {
             return new CombSpan(model, legId, settings);
        }
    }
}
