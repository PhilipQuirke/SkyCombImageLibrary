using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;


namespace SkyCombImage.PersistModel
{
    // Load annotation data about objects from a datastore
    public class CategoryLoad : DataStoreAccessor
    {
        public CategoryLoad(DroneDataStore data) : base(data)
        {
        }


        // Load all categories from the datastore
        public void LoadMasterCategories(MasterCategoryListJ list)
        {
            if (list == null)
                return;

            int row = 2;

            try
            {
                if (Data.SelectWorksheet(MasterCategoryTabName))
                {
                    list.Clear();

                    var cell = Data.Worksheet.Cells[row, CategoryModelJ.NameSetting];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        // Load the non-blank cells in this row into a model
                        list.Add(GroundCategoryFactory.NewMasterCategoryModel(Data.GetRowSettings(row, 1)));

                        row++;
                        cell = Data.Worksheet.Cells[row, CategoryModelJ.NameSetting];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CategoryLoad.LoadMasterCategories failure: " + ex.ToString());
                list.Clear();
            }

            list.MaybeDefault();
        }


        // Load all object-specific annotations from the datastore
        public void LoadObjectCategories(ObjectCategoryList list)
        {
            if (list == null)
                return;

            int row = 2;

            try
            {
                if (Data.SelectWorksheet(AnimalCategoryTabName))
                {
                    list.Clear();

                    var cell = Data.Worksheet.Cells[row, ObjectCategoryModel.ObjectNameSetting];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        // Load the non-blank cells in this row into a model
                        list.Add(ObjectCategoryFactory.NewObjectCategoryModel(Data.GetRowSettings(row, 1)));

                        row++;
                        cell = Data.Worksheet.Cells[row, ObjectCategoryModel.ObjectNameSetting];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CategoryLoad.LoadObjectCategories failure: " + ex.ToString());
                list.Clear();
            }
        }


        public void LoadAll(CategoryAll categoryAll)
        {
            bool initialIsOpen = Data.IsOpen;
            if (!initialIsOpen)
                Data.Open();

 //           LoadMasterCategories(categoryAll.MasterCategories);
            LoadObjectCategories(categoryAll.ObjectCategories);

            if (!initialIsOpen)
                Data.FreeResources();
        }
    }
}

