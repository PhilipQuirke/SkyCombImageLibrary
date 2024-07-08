using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.PersistModel
{
    // Load run and process data about a previous processing run from a datastore
    public class CombLoad : StandardLoad
    {
        public CombLoad(DroneDataStore data) : base(data)
        {
        }


        public List<string> ObjectListSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, ObjectSummaryTitle, ModelTitleRow, FarRhsColOffset);
        }


        // Load all Comb Objects from the datastore
        public void CombObjects(CombProcess model)
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
                        model.ProcessObjects.AddObject(
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
                model.ProcessObjects.Clear();
            }
        }


        // Load all Comb Features from the datastore
        public void CombFeatures(CombProcess model)
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

                        // Load the non-blank cells in this row into a CombFeature
                        var settings = Data.GetRowSettings(row, 1);
                        model.ProcessFeatures.AddFeature(
                            ProcessFactory.NewCombFeature(model, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CombLoad.CombFeatures failure: " + ex.ToString());
                model.ProcessFeatures.Clear();
            }
        }


        // Load all Comb Spans from the datastore
        public void CombSpans(CombProcess model, Drone drone)
        {
            int row = 2;

            try
            {
                if (Data.SelectWorksheet(SpanTabName))
                {
                    var cell = Data.Worksheet.Cells[row, ProcessSpan.SpanIdSetting];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        var legIdString = cell.Value.ToString();
                        if((legIdString == null) || (legIdString == ""))
                            break;
                        var legId = ConfigBase.StringToNonNegInt(legIdString);


                        // Load the non-blank cells in this row into a CombSpan
                        var settings = Data.GetRowSettings(row, 1);
                        model.ProcessSpans.AddSpan(
                            ProcessFactory.NewProcessSpan(model, legId, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, ProcessSpan.SpanIdSetting];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CombLoad.CombSpans failure: " + ex.ToString());
                model.ProcessSpans.Clear();
            }
        }

    }
}

