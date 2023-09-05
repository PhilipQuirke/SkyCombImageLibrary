using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.PersistModel
{
    // Load run and process data about a previous processing run from a datastore
    public class CombLoad : FlowLoad
    {
        public CombLoad(DroneDataStore data) : base(data)
        {
        }


        public List<string> ObjectListSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, ObjectSummaryTitle, ModelTitleRow, FarRhsColOffset);
        }


        // Load all Comb Objects from the datastore
        public void CombObjects(CombProcessAll model)
        {
            int row = 2;

            try
            {
                if (Data.SelectWorksheet(Objects1TabName))
                {
                    var cell = Data.Worksheet.Cells[row, 1];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        var objectIdString = cell.Value.ToString();
                        if (objectIdString == "")
                            break;
                        var objectId = ConfigBase.StringToNonNegInt(objectIdString);

                        // Load the non-blank cells in this row into a CombObject
                        model.CombObjs.CombObjList.AddObject(
                            ProcessFactory.NewCombObject(model, Data.GetRowSettings(row, 1)));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CombLoad.CombObjects failure: " + ex.ToString());
                model.CombObjs.CombObjList.Clear();
            }
        }


        // Load all Comb Features from the datastore
        public void CombFeatures(CombProcessAll model)
        {
            int row = 2;

            try
            {
                if (Data.SelectWorksheet(FeaturesTabName))
                {
                    var cell = Data.Worksheet.Cells[row, 1];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        var featureIdString = cell.Value.ToString();
                        if (featureIdString == "")
                            break;
                        var featureId = ConfigBase.StringToNonNegInt(featureIdString);

                        // Load the non-blank cells in this row into a CombFeature
                        var settings = Data.GetRowSettings(row, 1);
                        model.CombFeatures.AddFeature(
                            ProcessFactory.NewCombFeature(model, featureId, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CombLoad.CombFeatures failure: " + ex.ToString());
                model.CombFeatures.Clear();
            }
        }


        // Load all Comb Legs from the datastore
        public void CombLegs(CombProcessAll model, Drone drone)
        {
            int row = 2;

            try
            {
                if (Data.SelectWorksheet(Legs2TabName))
                {
                    var cell = Data.Worksheet.Cells[row, CombLeg.LegIdSetting];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        var legIdString = cell.Value.ToString();
                        if((legIdString == null) || (legIdString == ""))
                            break;
                        var legId = ConfigBase.StringToNonNegInt(legIdString);


                        // Load the non-blank cells in this row into a CombLeg
                        var settings = Data.GetRowSettings(row, 1);
                        model.CombLegs.AddLeg(
                            ProcessFactory.NewCombLeg(model, legId, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, CombLeg.LegIdSetting];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CombLoad.CombLegs failure: " + ex.ToString());
                model.CombLegs.Clear();
            }
        }

    }
}

