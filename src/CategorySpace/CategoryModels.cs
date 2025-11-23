// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.CategorySpace
{
    // An "ObjectCategoryModel" represents the annotations (if any) added manually to a detected object by the user.
    // The annotation is specific to one object but is deliberately NOT incorporated into ProcessObjectModel.
    // The annotations must survive the video being re-processed in part or in full, multiple times.
    // The annotations are persisted to a annotations-specific tab.
    public class ObjectCategoryModel : CategoryModelJ
    {
        // Object name
        public string ObjectName { get; set; }


        public ObjectCategoryModel(string objectName, string category = "", bool include = true, string notes = "") : base(category, include, notes)
        {
            ObjectName = objectName;

            AssertGood();
        }


        // Constructor used when loading objects from the datastore
        public ObjectCategoryModel(List<string>? settings)
        {
            ObjectName = "";

            if (settings != null)
                LoadSettings(settings);
        }


        public void AssertGood()
        {
            Assert(ObjectName != "", "ObjectCategoryModel.ObjectName bad");
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int ObjectNameSetting = 1;
        public const int ObjectCategorySetting = 2;
        public const int ObjectIncludeSetting = 3;
        public const int ObjectNotesSetting = 4;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public override DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Object Name", ObjectName },
                { "Category",  Category != "" ? Category : UnknownString  },
                { "Include", Include },
                { "Notes",  Notes != "" ? Notes : UnknownString  }
            };
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public override void LoadSettings(List<string> settings)
        {
            ObjectName = settings[0];
            LoadSettings_Internal(settings, 1);
        }
    };


    // An "ObjectCategoryList" represents all the annotations added manually by the user to objects.
    public class ObjectCategoryList : SortedList<string, ObjectCategoryModel>
    {
        public void Add(ObjectCategoryModel category)
        {
            Add(category.ObjectName, category);
        }


        // Find an existing annotation object (if any) for the named object
        public ObjectCategoryModel? GetData(string objectName)
        {
            var theObjectName = objectName.ToUpper().Trim();

            if (theObjectName != "")
                foreach (var annotation in this)
                    if (annotation.Value.ObjectName.ToUpper() == theObjectName)
                        return annotation.Value;

            return null;
        }


        // Load annotation information for a single object into the in-memory list. (Does NOT save it to the datastore.)
        // Returns true if an annotation has changed/added.
        public bool PutData(string objectName, string category, bool include, string notes)
        {
            objectName = objectName.Trim();
            if (objectName == "")
                return false;

            // If there is an existing annotation, update it, even if the values matches the default values.
            var annotation = GetData(objectName);
            if (annotation != null)
            {
                if (annotation.Category == category &&
                    annotation.Include == include &&
                    annotation.Notes == notes)
                    return false;

                if (category == "")
                    // A blank category means remove the annotation
                    Remove(objectName);
                else
                {
                    annotation.Category = category;
                    annotation.Include = include;
                    annotation.Notes = notes;
                }
                return true;
            }

            annotation = new ObjectCategoryModel(objectName, category, include, notes);

            // Don't store an annotation which matches the default values
            // as it doesnt contain any useful information.
            if (annotation.IsDefault())
                return false;

            Add(annotation);
            return true;
        }


        // Describe (summarise) the categoried objects.
        public string Describe()
        {
            var answer = "";

            if (Count > 0)
            {
                answer += Count.ToString() + " Objects categorised.\n";

                int excluded = 0;
                foreach (var annotation in Values)
                    if (!annotation.Include)
                        excluded++;
                if (excluded > 0)
                    answer += excluded.ToString() + " Objects excluded.\n";

                // For each category, count the number of objects in that category
                var categoryCounts = new SortedList<string, int>();
                foreach (var annotation in Values)
                {
                    var category = annotation.Category;
                    if (category == "")
                        continue;

                    if (!annotation.Include)
                        continue;

                    if (categoryCounts.ContainsKey(category))
                        categoryCounts[category]++;
                    else
                        categoryCounts.Add(category, 1);
                }

                // Add each non-zero included category to the answer
                foreach (var category in categoryCounts)
                    answer += category.Value.ToString() + " " + category.Key + ", ";
                answer = answer.TrimEnd(',', ' ');
            }

            return answer;
        }
    }


    // CategoryAll represents all the master and object-specific category
    // information that the user has manually entered.
    public class CategoryAll
    {
        // Master list of object categories used for annotations
        public MasterCategoryListJ MasterCategories { get; set; }

        // List of object-specific annotations added by user to detected objects
        public ObjectCategoryList ObjectCategories { get; set; }


        public CategoryAll()
        {
            MasterCategories = new();
            ObjectCategories = new();
        }
    }
}
