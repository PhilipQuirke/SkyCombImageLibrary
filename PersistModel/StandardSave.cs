using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;


namespace SkyCombImage.PersistModel
{
    // Save processing model data to a datastore
    public class StandardSave : BlockSave
    {
        public StandardSave(Drone drone, DroneDataStore data) : base(drone, data)
        {
        }


        // Create an output video file writer (if user wants MP4 output)
        public static (VideoWriter?, string) CreateVideoWriter(
            RunConfig Config, string inputFileName, double Fps, Size frameSize)
        {
            if (!Config.ProcessConfig.SaveAnnotatedVideo || Fps <= 0.1 || frameSize.Width == 0 || frameSize.Height == 0)
                return (null, "");

            return VideoData.CreateVideoWriter(inputFileName, Config.OutputElseInputDirectory, Fps, frameSize);
        }


        // Save process settings and clear the related data table tabs
        public void SaveProcessSettingsAndClearDetail(RunConfig runConfig, DataPairList effort, DataPairList settings)
        {
            Data.SelectOrAddWorksheet(ProcessSettingsTabName);

            Data.ClearWorksheet();

            Data.SetLargeTitle(ProcessSummaryTitle);

            Data.SetTitleAndDataListColumn(ProcessConfigTitle, ModelTitleRow, LhsColOffset, runConfig.ProcessConfig.GetModelSettings());

            Data.SetTitleAndDataListColumn(RunConfigTitle, RunTitleRow, MidColOffset, runConfig.GetSettings());

            Data.SetTitleAndDataListColumn(EffortTitle, EffortTitleRow, MidColOffset, effort);

            Data.SetTitleAndDataListColumn(OutputConfigTitle, OutputTitleRow, MidColOffset, runConfig.ProcessConfig.GetOutputSettings());

            Data.SetTitleAndDataListColumn(DrawTitle, DrawTitleRow, MidColOffset, runConfig.ImageConfig.GetSettings());

            Data.SetTitleAndDataListColumn(ModelFlightStepSummaryTitle, ModelTitleRow, RhsColOffset, settings);


            // We may be swapping from Yolo to Comb process or vica versa, so clear all existing "detail" model tabs
            if (Data.SelectWorksheet(ObjectsReportTabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(ObjectsDataTabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(FeaturesDataTabName))
                Data.ClearWorksheet();
            if (Data.SelectWorksheet(BlockDataTabName))
                Data.ClearWorksheet();

            Data.SelectWorksheet(IndexTabName);
        }


        // Save the video run & model data to the dataStore
        public void StandardProcess(RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, ProcessAll model)
        {
            try
            {
                Data.SelectOrAddWorksheet(ProcessSettingsTabName);
                SaveProcessSettingsAndClearDetail(runConfig, effort, settings);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, model.GetSettings());
                Data.FormatSummaryPage();

                // Save the Block data
                AddBlockList(model.Blocks);

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSave.StandardProcess", ex);
            }
        }


        // Save the Run & Model data to the dataStore
        public void ProcessAll(RunVideoPersist runVideo, bool fullSave)
        {
            try
            {
                var process = runVideo.ProcessAll;
                var runConfig = runVideo.RunConfig;
                var effort = runVideo.GetEffort();
                var settings = runVideo.GetSettings();
                var data = runVideo.DataStore;

                SaveProcessSettingsAndClearDetail(runConfig, effort, settings);

                Data.SelectOrAddWorksheet(ProcessSettingsTabName);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, process.GetSettings());
                Data.FormatSummaryPage();

                if (fullSave)
                {
                    // Save the Block data 
                    // Changing OnGroundAt or CameraDownDeg changes the Step data values like AltitudeM that are copied to block settings
                    AddBlockList(process.Blocks);

                    var saveAllObjects = (runConfig.ProcessConfig.SaveObjectData == SaveObjectDataEnum.All);

                    ObjectSave SaveProcess = new(data);

                    // Save the Feature data 
                    var saveFeatures = ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (process.ProcessFeatures.Count > 0));
                    if (saveFeatures)
                        SaveProcess.SaveFeatureList(runVideo.ProcessAll, saveAllObjects);

                    // Save the Object data 
                    var saveObjects = ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (process.ProcessObjects.Count > 0));
                    if (saveObjects)
                        SaveProcess.SaveObjectList(process, saveAllObjects);

                    // Add the Object/Feature charts
                    SaveProcess.SaveObjectReport(MaxDatumId, runVideo);

                    // Save the ProcessSpan data 
                    SaveProcess.SaveSpanList(process);

                    Data.SelectWorksheet(ObjectsReportTabName);

                    Data.HideWorksheet(BlockDataTabName);
                    Data.HideWorksheet(FeaturesDataTabName);
                    Data.HideWorksheet(SpanDataTabName);
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