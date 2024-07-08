using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.PersistModel
{
    // Load run and process data about a previous processing run from a datastore
    public class YoloLoad : StandardLoad
    {
        public YoloLoad(DroneDataStore data) : base(data)
        {
        }


        public List<string> ObjectListSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, ObjectSummaryTitle, ModelTitleRow, FarRhsColOffset);
        }


        // Load all Yolo Objects from the datastore
        public void YoloObjects(YoloProcess model)
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

                        // Load the non-blank cells in this row into a YoloObject
                        model.YoloObjects.AddObject(
                            ProcessFactory.NewYoloObject(model, Data.GetRowSettings(row, 1)));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed YoloLoad.YoloObjects failure: " + ex.ToString());
                model.YoloObjects.Clear();
            }
        }


        // Load all Yolo Features from the datastore
        public void YoloFeatures(YoloProcess model)
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

                        // Load the non-blank cells in this row into a YoloFeature
                        var settings = Data.GetRowSettings(row, 1);
                        model.ProcessFeatures.AddFeature(
                            ProcessFactory.NewYoloFeature(model, settings));

                        row++;
                        cell = Data.Worksheet.Cells[row, 1];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed YoloLoad.YoloFeatures failure: " + ex.ToString());
                model.ProcessFeatures.Clear();
            }
        }

    }
}

