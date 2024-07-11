using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;


namespace SkyCombImage.PersistModel
{
    // Load run and model data about a previous processing run from a datastore
    public class StandardLoad : DataStoreAccessor
    {
        public StandardLoad(DroneDataStore data) : base(data)
        {
        }


        // Load model config data from the datastore
        public List<string> ModelConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, ProcessConfigTitle, ModelTitleRow, LhsColOffset);
        }


        // Load run config data from the datastore 
        public List<string> RunConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, RunConfigTitle, RunTitleRow, MidColOffset);
        }


        // Load draw config data from the datastore
        public List<string> DrawConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, DrawTitle, DrawTitleRow, MidColOffset);
        }


        // Load "save" config data from the datastore
        public List<string> OutputConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, OutputConfigTitle, OutputTitleRow, MidColOffset);
        }


        // Load all ProcessFeatures from the datastore
        public void ProcessFeatures(ProcessAll model)
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
                        if( model is CombProcess)
                            model.ProcessFeatures.AddFeature(
                                ProcessFactory.NewCombFeature(model as CombProcess, settings));
                        else if( model is YoloProcess)
                            model.ProcessFeatures.AddFeature(
                                ProcessFactory.NewYoloFeature(model as YoloProcess, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed StandardLoad.ProcessFeatures failure: " + ex.ToString());
                model.ProcessFeatures.Clear();
            }
        }


        // Load all Process Objects from the datastore
        public void ProcessObjects(ProcessAll model)
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

                        // Load the non-blank cells in this row into a Object
                        var settings = Data.GetRowSettings(row, 1);
                        if ( model is CombProcess)
                            model.ProcessObjects.AddObject(
                                ProcessFactory.NewCombObject(model as CombProcess, settings));
                        else 
                            model.ProcessObjects.AddObject(
                                ProcessFactory.NewYoloObject(model as YoloProcess, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed ProcessLoad.ProcessObjects failure: " + ex.ToString());
                model.ProcessObjects.Clear();
            }
        }


        public List<string> ObjectListSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, ObjectSummaryTitle, ModelTitleRow, FarRhsColOffset);
        }


        // Load all Process Spans from the datastore
        public void ProcessSpans(ProcessAll model, Drone drone)
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
                        if ((legIdString == null) || (legIdString == ""))
                            break;
                        var legId = ConfigBase.StringToNonNegInt(legIdString);


                        // Load the non-blank cells in this row into a ProcessSpan
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
                System.Diagnostics.Debug.WriteLine("Suppressed ProcessLoad.ProcessSpans failure: " + ex.ToString());
                model.ProcessSpans.Clear();
            }
        }


    }
}