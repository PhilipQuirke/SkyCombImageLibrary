// Copyright SkyComb Limited 2024. All rights reserved. 
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


        public static ProcessFeatureList NewProcessFeatureList(ProcessConfigModel config)
        {
            return new ProcessFeatureList(config);
        }


        public static ProcessSpan NewProcessSpan(ProcessAll processAll, int legId, List<string>? settings = null)
        {
            return new ProcessSpan(processAll, legId, settings);
        }


        public static ProcessAll NewProcessModel(ProcessConfigModel config, Drone drone)
        {
            return new ProcessAll(null, drone.InputVideo, drone, config);
        }


        public static YoloProcess NewYoloProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, string yoloDirectory)
        {
            return new YoloProcess(ground, video, drone, config, yoloDirectory);
        }


        public static YoloFeature NewYoloFeature(YoloProcess model, List<string> settings)
        {
            return new YoloFeature(model, settings);
        }


        public static YoloObject NewYoloObject(YoloProcess model, List<string> settings)
        {
            return new YoloObject(model, settings);
        }


        public static CombProcess NewCombProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config )
        {
            return new CombProcess(ground, video, drone, config);
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
    }
}
