using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;


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

                //               SaveMasterCategories(categoryAll.MasterCategories);
                SaveObjectCategories(categoryAll.ObjectCategories);

                Save();
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.SaveAll", ex);
            }
        }
    }
}