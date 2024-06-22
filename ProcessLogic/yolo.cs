using Compunet.YoloV8;
// using Compunet.YoloV8.Plotting;
using SkyCombGround.CommonSpace;
// using SixLabors.ImageSharp;
using Compunet.YoloV8.Data;


namespace SkyCombImageLibrary.ProcessLogic
{
    internal class YoloV8 : BaseConstants
    {
        YoloV8Predictor? Predictor = null;


        YoloV8(string modelDirectory)
        {
            Assert(modelDirectory != "", "modelDirectory is not specified");

            // If modelDirectory doesnt end in ".pt" append "\model.pt" or "model.pt"
            if (!modelDirectory.EndsWith(".pt"))
            {
                if (modelDirectory.EndsWith("\\") || modelDirectory.EndsWith("/"))
                    modelDirectory += "model.pt";
                else
                    modelDirectory += "\\model.pt";
            }

            try
            {
                // Load the model
                Predictor = YoloV8Predictor.Create(modelDirectory);
            }
            catch
            {
                Predictor = null;
            }
        }


        public static ImageSelector ConvertToImageSelector(System.Drawing.Image image)
        {
            // Convert System.Drawing.Image to byte array
            using (MemoryStream ms = new MemoryStream())
            {
                image.Save(ms, image.RawFormat);
                byte[] imageBytes = ms.ToArray();

                // Create ImageSelector from byte array
                return new ImageSelector(imageBytes);
            }
        }


        async Task<PoseResult?> Predict(System.Drawing.Image image)
        {
            if (Predictor == null)
                return null;

            try
            {
                ImageSelector imageSelector = ConvertToImageSelector(image);

                // Predict result
                PoseResult PoseResult = await Predictor.PoseAsync(imageSelector);

                // Plot the result
                //var plotted = await result.PlotImageAsync(image);

                // Save the result
                //plotted.Save(image.Path + "_pose.jpg");

                return PoseResult;
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloV8.Pose failed: " + ex.Message);
            }
        }
    }
}
