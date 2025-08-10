// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombGround.GroundLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    internal class ProcessFactory : BaseConstants
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


        public static ProcessAll NewProcessModel(ProcessConfigModel config, Drone drone, RunUserInterface runUI)
        {
            return new ProcessAll(null, drone.InputVideo, drone, config, runUI);
        }


        public static YoloProcess NewYoloProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, RunUserInterface runUI, string yoloDirectory)
        {
            return new YoloProcess(ground, video, drone, config, runUI, yoloDirectory);
        }


        public static YoloFeature NewYoloFeature(YoloProcess model, List<string> settings)
        {
            return new YoloFeature(model, settings);
        }


        public static YoloObject NewYoloObject(YoloProcess model, List<string> settings)
        {
            return new YoloObject(model, settings);
        }


        public static YoloObject NewYoloObject(YoloProcess yoloProcess, ProcessScope scope, int legId, YoloFeature firstFeature, string className, Color classColor, double classConfidence)
        {
            return new YoloObject(yoloProcess, scope, legId, firstFeature, className, classColor, classConfidence);
        }


        public static CombProcess NewCombProcess(GroundData ground, VideoData video, Drone drone, ProcessConfigModel config, RunUserInterface runUI)
        {
            return new CombProcess(ground, video, drone, config, runUI);
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
