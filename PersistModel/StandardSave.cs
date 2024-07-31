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
using SkyCombImageLibrary.RunSpace;


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


        // Create an output video file writer (if user wants MP4 output)
        public static (VideoWriter?, string) CreateVideoWriter(
            RunConfig Config, string inputFileName, double Fps, Size frameSize)
        {
            if (!Config.ProcessConfig.SaveAnnotatedVideo || Fps <= 0.1 || frameSize.Width == 0 || frameSize.Height == 0)
                return (null, "");

            return VideoData.CreateVideoWriter(inputFileName, Config.OutputElseInputDirectory, Fps, frameSize);
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


        // Save the Run & Model data to the dataStore
        public void ProcessAll(DroneDataStore data, RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, ProcessAll process, bool fullSave)
        {
            try
            {
                SaveCommonSummaryAndClearDetail(runConfig, effort, settings);

                Data.SelectOrAddWorksheet(ProcessTabName);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, process.GetSettings());
                Data.FormatSummaryPage();
                Data.SetLastUpdateDateTime(ProcessTabName);

                if (fullSave)
                {
                    // Save the Block data 
                    // Changing OnGroundAt or CameraDownDeg changes the Step data values like AltitudeM that are copied to block settings
                    AddBlockList(process.Blocks);

                    // Add the Block charts
                    AddBlocks2Tab(summary);

                    var saveAllObjects = (runConfig.ProcessConfig.SaveObjectData == SaveObjectDataEnum.All);

                    ObjectSave SaveProcess = new(data);

                    // Save the Feature data 
                    var saveFeatures = ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (process.ProcessFeatures.Count > 0));
                    if (saveFeatures)
                        SaveProcess.SaveFeatureList(process, saveAllObjects);

                    // Save the Object data 
                    var saveObjects = ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (process.ProcessObjects.Count > 0));
                    if (saveObjects)
                        SaveProcess.SaveObjectList(process, saveAllObjects);

                    // Add the Object/Feature charts
                    SaveProcess.SaveObjectGraphs(MaxDatumId);

                    // Save the ProcessSpan data 
                    SaveProcess.SaveSpanList(process);

                    if (saveObjects)
                        SaveProcess.SavePopulation(process.ProcessObjects);
                }

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("StandardSave.ProcessAll", ex);
            }
        }
    }
}