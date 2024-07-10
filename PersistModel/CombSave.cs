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
    public class CombSave : StandardSave
    {
        ProcessSaveObject SaveProcess;


        public CombSave(Drone drone, DroneDataStore data) : base(drone, data)
        {
            SaveProcess = new(data);
        }


        // Save the Run & Model data to the dataStore
        public void Comb(RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, CombProcess? process, bool fullSave)
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

                    // Save the Pixel data 
                    if ((runConfig.ProcessConfig.SavePixels != SavePixelsEnum.None) && (process.ProcessFeatures.Count > 0))
                    {
                        Data.SelectOrAddWorksheet(PixelsTabName);
                        int row = 0;
                        foreach (var feature in process.ProcessFeatures)
                            if (runConfig.ProcessConfig.SavePixels == SavePixelsEnum.All || feature.Value.Significant)
                            {
                                var combFeature = feature.Value as CombFeature;
                                if (combFeature.Pixels != null)
                                    foreach (var pixel in combFeature.Pixels)
                                        if (runConfig.ProcessConfig.SaveObjectData == SaveObjectDataEnum.All || feature.Value.Significant)
                                            Data.SetDataListRowKeysAndValues(ref row, pixel.GetSettings());
                            }

                        Data.SetLastUpdateDateTime(PixelsTabName);
                    }

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

                    if (saveObjects)
                        SaveProcess.SavePopulation(process.ProcessObjects);
                }

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSave.Comb", ex);
            }
        }
    }
}