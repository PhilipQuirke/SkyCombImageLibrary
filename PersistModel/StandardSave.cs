using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;
using System.Drawing.Imaging;


namespace SkyCombImage.PersistModel
{
    // Save processing model data to a datastore
    public class StandardSave : BlockSave
    {
        public StandardSave(Drone drone, DroneDataStore data) : base(drone, data)
        {
        }


        // Save image to disk
        public static void ImageAsJpg(RunConfig Config, Image<Bgr, byte> imgOutput)
        {
            string outputImageFilename = "";

            try
            {
                if (!Config.ProcessConfig.SaveAnnotatedVideo)
                    return;

                outputImageFilename =
                        Config.InputFileName.Substring(0, Config.InputFileName.Length - 4) +
                        "_Image.JPG";

                imgOutput.ToBitmap().Save(outputImageFilename, ImageFormat.Jpeg);
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("RunSave.ImageAsJpg: filename=" + outputImageFilename, ex);
            }
        }


        // Create an output video file writer
        public static (VideoWriter?, string) CreateVideoWriter(
            RunConfig Config, string inputFileName, double Fps, Size frameSize)
        {
            if (!Config.ProcessConfig.SaveAnnotatedVideo || Fps <= 0.1 || frameSize.Width == 0 || frameSize.Height == 0)
                return (null, "");

            return VideoData.CreateVideoWriter(inputFileName, Config.OutputElseInputDirectory(), Fps, frameSize);
        }


        // Save common model data
        public void SaveCommonSummaryAndClearDetail(RunConfig runConfig, DataPairList effort, DataPairList settings)
        {
            Data.SelectOrAddWorksheet(ProcessTabName);

            Data.ClearWorksheet();

            Data.SetTitles(ProcessSummaryTitle);

            Data.SetTitleAndDataListColumn(ProcessConfigTitle, ModelTitleRow, LhsColOffset, runConfig.ProcessConfig.GetModelSettings());

            Data.SetTitleAndDataListColumn(RunConfigTitle, RunTitleRow, MidColOffset, runConfig.GetSettings());

            Data.SetTitleAndDataListColumn(EffortTitle, EffortTitleRow, MidColOffset, effort);

            Data.SetTitleAndDataListColumn(OutputConfigTitle, OutputTitleRow, MidColOffset, runConfig.ProcessConfig.GetOutputSettings());

            Data.SetTitleAndDataListColumn(DrawTitle, DrawTitleRow, MidColOffset, runConfig.ImageConfig.GetSettings());

            Data.SetTitleAndDataListColumn(ModelFlightStepSummaryTitle, ModelTitleRow, RhsColOffset, settings);


            // We may be swapping from Yolo to Comb process or vica versa, so clear all existing "detail" model tabs
            if (Data.SelectWorksheet(Blocks1TabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(Blocks2TabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(Objects1TabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(Objects2TabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(FeaturesTabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(PixelsTabName))
                Data.ClearWorksheet();

            Data.SelectWorksheet(IndexTabName);
        }


        // Save the video run & model data to the dataStore
        public void StandardProcess(RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, ProcessAll model)
        {
            try
            {
                Data.SelectOrAddWorksheet(ProcessTabName);
                SaveCommonSummaryAndClearDetail(runConfig, effort, settings);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, model.GetSettings());
                Data.FormatSummaryPage();
                Data.SetLastUpdateDateTime(ProcessTabName);


                // Save the Block data
                AddBlockList(model.Blocks);


                // No Pixel data 
                if (Data.SelectWorksheet(PixelsTabName))
                    Data.ClearWorksheet();


                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSave.StandardProcess", ex);
            }
        }
    }
}