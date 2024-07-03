using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;
using System.Diagnostics;
using System;



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
                        SaveProcess.SaveFeatureList(process, process.ProcessFeatures, saveAllObjects);

                    // Save the Object data 
                    var saveObjects = ((runConfig.ProcessConfig.SaveObjectData != SaveObjectDataEnum.None) && (process.CombObjs.CombObjList.Count > 0));
                    if (saveObjects)
                        SaveProcess.SaveObjectList(process, process.CombObjs.CombObjList, saveAllObjects);

                    // Add the Object/Feature charts
                    SaveProcess.SaveObjectGraphs(MaxDatumId);

                    // Save the CombSpan data 
                    if ((process.CombSpans != null) && (process.CombSpans.Count > 0))
                    {
                        Data.SelectOrAddWorksheet(SpanTabName);
                        int legRow = 0;
                        foreach (var leg in process.CombSpans)
                            Data.SetDataListRowKeysAndValues(ref legRow, leg.Value.GetSettings());

                        Data.SetColumnColor(CombSpanModel.SpanIdSetting, legRow, Color.Blue);
                        Data.SetColumnColor(CombSpanModel.SpanNameSetting, legRow, Color.Blue);
                        Data.SetColumnColor(CombSpanModel.BestFixAltMSetting, legRow, Color.Blue);

                        Data.SetLastUpdateDateTime(SpanTabName);
                    }

                    if (saveObjects)
                        SaveProcess.SavePopulation(process.CombObjs.CombObjList);
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