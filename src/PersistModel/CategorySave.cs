using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;


namespace SkyCombImage.PersistModel
{
    // Save Comb processing model data to a datastore
    public class CategorySave : DataStoreAccessor
    {
        public CategorySave(DroneDataStore data) : base(data)
        {
        }


        // Save the annotation data to the dataStore
        public void SaveMasterCategories(MasterCategoryListJ list)
        {
            try
            {
                // Save the categories
                if ((list != null) && (list.Count > 0))
                {
                    Data.SelectOrAddWorksheet(MasterCategoryTabName);
                    int theRow = 0;
                    foreach (var annotation in list)
                        Data.SetDataListRowKeysAndValues(ref theRow, annotation.Value.GetSettings());
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.SaveMasterCategories", ex);
            }
        }


        // Save the annotation data to the dataStore
        public void SaveObjectCategories(ObjectCategoryList list)
        {
            try
            {
                // Save the user Annotations (if any)
                if ((list != null) && (list.Count > 0))
                {
                    Data.SelectOrAddWorksheet(AnimalCategoryTabName);
                    int theRow = 0;
                    foreach (var annotation in list)
                        Data.SetDataListRowKeysAndValues(ref theRow, annotation.Value.GetSettings());
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.SaveObjectCategories", ex);
            }
        }


        // Save the annotations data to the dataStore
        public void SaveAll(CategoryAll categoryAll)
        {
            try
            {
                Data.Open();

                SaveObjectCategories(categoryAll.ObjectCategories);

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.SaveAll", ex);
            }
        }



        // Save animal waypoint data to CSV and JSON files
        public static void SaveAnimalWaypoints(RunWorker runWorker)
        {
            try
            {
                var runConfig = runWorker.RunConfig;
                var processAll = runWorker.ProcessAll;
                var processObjects = processAll.ProcessObjects;
                var processObjCats = runWorker.CategoryAll.ObjectCategories;
                var processSpans = runWorker.ProcessAll.ProcessSpans;

                if (processObjects == null)
                    return;

                var animals = new AnimalModelList();
                animals.AddProcessObjects(1, runWorker.Drone, processObjects, processObjCats, processSpans);
                if (animals.Count > 0)
                {
                    var all_waypoints = AnimalSave.GetWaypoints(animals, true);

                    var filePath = DataStoreFactory.OutputFileName(
                        runConfig.InputDirectory, runConfig.InputFileName,
                        runConfig.OutputElseInputDirectory, DataStoreFactory.AllWaypointCsvSuffix);
                    UgcsWaypointExporter.ExportToCsvWithHeaders(all_waypoints, filePath);

                    filePath = DataStoreFactory.OutputFileName(
                        runConfig.InputDirectory, runConfig.InputFileName,
                        runConfig.OutputElseInputDirectory, DataStoreFactory.AllWaypointJsonSuffix);
                    UgcsWaypointExporter.ExportToJson(all_waypoints, filePath);

                    filePath = DataStoreFactory.OutputFileName(
                        runConfig.InputDirectory, runConfig.InputFileName,
                        runConfig.OutputElseInputDirectory, DataStoreFactory.AllWaypointKmlSuffix);
                    UgcsWaypointExporter.ExportToKml(all_waypoints, filePath, "SkyComb - All Hotspots");

                    var some_waypoints = AnimalSave.GetWaypoints(animals, false);

                    filePath = DataStoreFactory.OutputFileName(
                        runConfig.InputDirectory, runConfig.InputFileName,
                        runConfig.OutputElseInputDirectory, DataStoreFactory.SomeWaypointCsvSuffix);
                    UgcsWaypointExporter.ExportToCsvWithHeaders(some_waypoints, filePath);

                    filePath = DataStoreFactory.OutputFileName(
                        runConfig.InputDirectory, runConfig.InputFileName,
                        runConfig.OutputElseInputDirectory, DataStoreFactory.SomeWaypointJsonSuffix);
                    UgcsWaypointExporter.ExportToJson(some_waypoints, filePath);

                    filePath = DataStoreFactory.OutputFileName(
                        runConfig.InputDirectory, runConfig.InputFileName,
                        runConfig.OutputElseInputDirectory, DataStoreFactory.SomeWaypointKmlSuffix);
                    UgcsWaypointExporter.ExportToKml(all_waypoints, filePath, "SkyComb - Selected Hotspots");
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("StandardSave.SaveAnimalWaypoints", ex);
            }
        }
    }
}