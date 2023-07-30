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
    public class CombSave : FlowSave
    {
        CombSaveObject SaveObject;

        public CombSave(Drone drone, DroneDataStore data) : base(drone, data)
        {
            SaveObject = new(data);
        }


        // Save the Run & Model data to the dataStore
        public void Comb(RunConfig runConfig, DataPairList effort, DataPairList settings, FlightStepSummaryModel summary, CombProcessAll combProcess, bool fullSave)
        {
            try
            {
                SaveCommonSummaryAndClearDetail(runConfig, effort, settings);

                Data.SelectOrAddWorksheet(ProcessTabName);
                Data.SetTitleAndDataListColumn(ResultsTitle, ResultsTitleRow, MidColOffset, combProcess.GetSettings());
                Data.FormatSummaryPage();
                Data.SetLastUpdateDateTime(ProcessTabName);

                // Save the Block data 
                // Changing OnGroundAt or CameraDownDeg changes the Step data values like AltitudeM that are copied to block settings
                AddBlockList(combProcess.Blocks);

                // Add the Block charts
                AddBlocks2Tab(summary);

                var saveObjects = ((runConfig.Process.SaveObjectData != SaveObjectDataEnum.None) && (combProcess.CombObjs.CombObjList.Count > 0));
                var saveAllObjects = (runConfig.Process.SaveObjectData == SaveObjectDataEnum.All);

                if (fullSave)
                {
                    // Save the Pixel data 
                    if ((runConfig.Process.SavePixels != SavePixelsEnum.None) && (combProcess.CombFeatures.Count > 0))
                    {
                        Data.SelectOrAddWorksheet(PixelsTabName);
                        int row = 0;
                        foreach (var feature in combProcess.CombFeatures)
                            if (runConfig.Process.SavePixels == SavePixelsEnum.All || feature.Value.Significant)
                                if (feature.Value.Pixels != null)
                                    foreach (var pixel in feature.Value.Pixels)
                                        if (runConfig.Process.SaveObjectData == SaveObjectDataEnum.All || feature.Value.Significant)
                                            Data.SetDataListRowKeysAndValues(ref row, pixel.GetSettings());

                        Data.SetLastUpdateDateTime(PixelsTabName);
                    }


                    // Save the Feature data 
                    if ((runConfig.Process.SaveObjectData != SaveObjectDataEnum.None) && (combProcess.CombFeatures.Count > 0))
                    {
                        Data.SelectOrAddWorksheet(FeaturesTabName);
                        int featureRow = 0;
                        foreach (var feature in combProcess.CombFeatures)
                            if (runConfig.Process.SaveObjectData == SaveObjectDataEnum.All || feature.Value.Significant)
                                Data.SetDataListRowKeysAndValues(ref featureRow, feature.Value.GetSettings());

                        Data.SetColumnWidth(ProcessFeatureModel.NotesSetting, 20);

                        Data.SetNumberColumnNdp(ProcessFeatureModel.NorthingMSetting, LocationNdp);
                        Data.SetNumberColumnNdp(ProcessFeatureModel.EastingMSetting, LocationNdp);
                        Data.SetNumberColumnNdp(ProcessFeatureModel.HeightMSetting, HeightNdp);
                        Data.SetNumberColumnNdp(ProcessFeatureModel.ObjSpeedPxlsSetting, PixelVelNdp);

                        Data.SetColumnColor(ProcessFeatureModel.FeatureIdSetting, featureRow, Color.Blue);
                        Data.SetColumnColor(ProcessFeatureModel.ObjectIdSetting, featureRow, Color.Blue);
                        Data.SetColumnColor(ProcessFeatureModel.BlockIdSetting, featureRow, Color.Blue);
                        Data.SetColumnColor(ProcessFeatureModel.HeightMSetting, featureRow, Color.Blue);
                        Data.SetColumnColor(ProcessFeatureModel.LegIdSetting, featureRow, Color.Blue);

                        Data.SetLastUpdateDateTime(FeaturesTabName);
                    }


                    // Save the Object data 
                    if (saveObjects)
                        SaveObject.SaveObjectList(combProcess, saveAllObjects);

                    // Add the Object/Feature charts
                    SaveObject.SaveObjectGraphs(MaxDatumId);
                }

                // Save the CombLeg data 
                if ((combProcess.CombLegs != null) && (combProcess.CombLegs.Count > 0))
                {
                    Data.SelectOrAddWorksheet(Legs2TabName);
                    int legRow = 0;
                    foreach (var leg in combProcess.CombLegs)
                        Data.SetDataListRowKeysAndValues(ref legRow, leg.Value.GetSettings());

                    Data.SetColumnColor(CombLegModel.LegIdSetting, legRow, Color.Blue);
                    Data.SetColumnColor(CombLegModel.LegNameSetting, legRow, Color.Blue);
                    Data.SetColumnColor(CombLegModel.BestFixAltitudeMSetting, legRow, Color.Blue);

                    Data.SetLastUpdateDateTime(Legs2TabName);
                }

                if (saveObjects)
                    SaveObject.SavePopulation(combProcess);

                SaveAndClose();
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSave.Comb", ex);
            }
        }
    }
}