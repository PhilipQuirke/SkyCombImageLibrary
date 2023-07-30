using SkyCombImage.CategorySpace;
using SkyCombDrone.PersistModel;


namespace SkyCombImage.PersistModel
{
    // Save Comb processing model data to a datastore
    public class CategorySave : DataStoreAccessor
    {
        public CategorySave(DroneDataStore data) : base(data)
        {
        }


        // Save the annotation data to the dataStore
        public void SaveMasterCategories(MasterCategoryList list)
        {
            try
            {
                // Save the categories
                if ((list != null) && (list.Count > 0))
                {
                    Data.SelectOrAddWorksheet(CategoryTabName);
                    int theRow = 0;
                    foreach (var annotation in list)
                        Data.SetDataListRowKeysAndValues(ref theRow, annotation.Value.GetSettings());

                    Data.SetLastUpdateDateTime(ObjectCategoryTabName);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.CategoryList", ex);
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
                    Data.SelectOrAddWorksheet(ObjectCategoryTabName);
                    int theRow = 0;
                    foreach (var annotation in list)
                        Data.SetDataListRowKeysAndValues(ref theRow, annotation.Value.GetSettings());

                    Data.SetLastUpdateDateTime(CategoryTabName);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.ObjectCategoryList", ex);
            }
        }


        // Save the annotations data to the dataStore
        public void SaveAll(CategoryAll categoryAll)
        {
            try
            {
                Data.Open();

                SaveMasterCategories(categoryAll.MasterCategories);
                SaveObjectCategories(categoryAll.ObjectCategories);

                SaveAndClose();
            }
            catch (Exception ex)
            {
                throw ThrowException("CategorySave.SaveAll", ex);
            }
        }
    }
}