// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImageLibrary.RunSpace;


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
            Image<Bgr, byte> imgInput = imgOriginal.Clone();
            Smooth(config.ProcessConfig, ref imgInput);

            Image<Gray, byte> imgThreshold = ToGrayScale(imgInput);
            Threshold(config.ProcessConfig, ref imgThreshold);

            ProcessFeatureList featuresInBlock = ProcessFactory.NewProcessFeatureList(model.ProcessConfig);
            int num_sig = featuresInBlock.NumSig;

            CombFeatureLogic.CreateFeaturesFromImage(model, featuresInBlock, block, imgOriginal, imgThreshold);
            num_sig = featuresInBlock.NumSig;

            return featuresInBlock;
        }
    }
}