using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;



namespace SkyCombImage.PersistModel
{
    // Save Comb processing model data to a datastore
    public class YoloSave : FlowSave
    {
        public YoloSave(Drone drone, DroneDataStore data) : base(drone, data)
        {
        }


        // Save the Run & Model data to the dataStore
        public void Yolo(RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, YoloProcess yoloProcess)
        {
            try
            {
                SaveCommonSummaryAndClearDetail(runConfig, effort, settings);

                Data.SelectOrAddWorksheet(ProcessTabName);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, yoloProcess.GetSettings());
                Data.FormatSummaryPage();
                Data.SetLastUpdateDateTime(ProcessTabName);

                // Save the Block data 
                // Changing OnGroundAt or CameraDownDeg changes the Step data values like AltitudeM that are copied to block settings
                AddBlockList(yoloProcess.Blocks);

                // Add the Block charts
                AddBlocks2Tab(summary);

                var saveObjects = ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (yoloProcess.YoloObjects.Count > 0));
                var saveAllObjects = (runConfig.ProcessConfig.SaveObjectData == SaveObjectDataEnum.All);

                // Save the Feature data 
                if ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (yoloProcess.YoloFeatures.Count > 0))
                {
                    Data.SelectOrAddWorksheet(FeaturesTabName);
                    int featureRow = 0;
                    foreach (var feature in yoloProcess.YoloFeatures)
                        if (runConfig.ProcessConfig.SaveObjectData == SaveObjectDataEnum.All || feature.Value.Significant)
                            Data.SetDataListRowKeysAndValues(ref featureRow, feature.Value.GetSettings());

                    Data.SetColumnWidth(ProcessFeatureModel.NotesSetting, 20);

                    Data.SetNumberColumnNdp(ProcessFeatureModel.NorthingMSetting, LocationNdp);
                    Data.SetNumberColumnNdp(ProcessFeatureModel.EastingMSetting, LocationNdp);
                    Data.SetNumberColumnNdp(ProcessFeatureModel.HeightMSetting, HeightNdp);

                    Data.SetColumnColor(ProcessFeatureModel.FeatureIdSetting, featureRow, Color.Blue);
                    Data.SetColumnColor(ProcessFeatureModel.ObjectIdSetting, featureRow, Color.Blue);
                    Data.SetColumnColor(ProcessFeatureModel.BlockIdSetting, featureRow, Color.Blue);
                    Data.SetColumnColor(ProcessFeatureModel.HeightMSetting, featureRow, Color.Blue);
                    Data.SetColumnColor(ProcessFeatureModel.LegIdSetting, featureRow, Color.Blue);

                    Data.SetLastUpdateDateTime(FeaturesTabName);
                }

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSave.Yolo", ex);
            }
        }
    }
}