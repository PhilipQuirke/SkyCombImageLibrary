// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;



namespace SkyCombImage.PersistModel
{
    // Save Yolo processing model data to a datastore
    public class YoloSave : StandardSave
    {
        ProcessSaveObject SaveProcess;


        public YoloSave(Drone drone, DroneDataStore data) : base(drone, data)
        {
            SaveProcess = new(data);
        }


        // Save the Run & Model data to the dataStore
        public void Yolo(RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, YoloProcess process)
        {
            try
            {
                SaveCommonSummaryAndClearDetail(runConfig, effort, settings);

                Data.SelectOrAddWorksheet(ProcessTabName);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, process.GetSettings());
                Data.FormatSummaryPage();
                Data.SetLastUpdateDateTime(ProcessTabName);

                // Save the Block data 
                // Changing OnGroundAt or CameraDownDeg changes the Step data values like AltitudeM that are copied to block settings
                AddBlockList(process.Blocks);

                // Add the Block charts
                AddBlocks2Tab(summary);

                var saveAllObjects = (runConfig.ProcessConfig.SaveObjectData == SaveObjectDataEnum.All);

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

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("YoloSave.Yolo", ex);
            }
        }
    }
}