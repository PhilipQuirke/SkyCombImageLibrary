// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;


// Namespace for analysis of a SINGLE image.
// This namespace does NOT include code related to "persisting" information between images.
namespace SkyCombImage.RunSpace
{
    class CombImage : DrawImage
    {
        // Analyse input image using Comb specific approach, to generate a list of features.
        public static ProcessFeatureList Process(
            RunConfig config,
            CombProcess model,
            ProcessBlock block,
            Image<Bgr, byte> imgOriginal)
        {
            var imgInput = imgOriginal.Clone();

            Smooth(config.ProcessConfig, ref imgInput);

            var imgThreshold = ToGrayScale(imgInput);

            Threshold(config.ProcessConfig, ref imgThreshold);

            // Set pixels hotter than TruncThresholdValue to 0.
            // Currently TruncThresholdValue can only be set by editing the datastore.
            // Used with DJI_0114 experimentally to eliminate unwanted hot house roof. Unsuccessful. 
            if (config.ProcessConfig.TruncThresholdValue != BaseConstants.UnknownValue)
                imgThreshold = imgThreshold.ThresholdToZeroInv(new Gray(config.ProcessConfig.TruncThresholdValue));

            // Set pixels hotter than ThresholdValue to 1. Set other pixels to 0.
            DrawImage.Threshold(config.ProcessConfig, ref imgThreshold);

            var dataModel = model;
            if (dataModel == null)
                dataModel = ProcessFactory.NewCombProcessModel(null,
                    new VideoData(imgOriginal.Height, imgOriginal.Width), model.Drone, config.ProcessConfig);

            var featuresInBlock = ProcessFactory.NewProcessFeatureList(model.ProcessConfig);
            CombFeatureLogic.CreateFeaturesFromImage(dataModel, featuresInBlock, block, imgOriginal, imgThreshold);
            return featuresInBlock;
        }
    }
}