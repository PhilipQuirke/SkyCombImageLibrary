using SkyCombGround.CommonSpace;
using static Microsoft.WindowsAPICodePack.Shell.PropertySystem.SystemProperties.System;



// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.CategorySpace
{
    // An "CategoryModel" represents the information that can be added to an object to categorise it.
    public class CategoryModel : ConfigBase
    {
        // What category does the object belong to?
        public string Category { get; set; }

        // Should the object be included in analysis?
        public bool Include { get; set; }

        // Notes about the object
        public string Notes { get; set; }

        public CategoryModel(string category = "", bool include = true, string notes = "")
        {
            Category = category;
            Include = include;
            Notes = notes;
        }


        // Constructor used when loading objects from the datastore
        public CategoryModel(List<string>? settings)
        {
            Category = "";
            Include = true;
            Notes = "";

            if (settings != null)
                LoadSettings(settings);
        }


        // Is the annotation set to the default values?
        public bool IsDefault()
        {
            return Include && Category == "" && Notes == "";
        }


        public bool Equals(string category, bool include, string notes)
        {
            return
                Category == category &&
                Include == include &&
                Notes == notes;
        }


        public bool Equals(CategoryModel other)
        {
            return
                this.Category == other.Category &&
                this.Include == other.Include &&
                this.Notes == other.Notes;
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int NameSetting = 1;
        public const int IncludeSetting = 2;
        public const int NotesSetting = 3;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public virtual DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Category",  Category != "" ? Category : UnknownString  },
                { "Include", Include },
                { "Notes",  Notes != "" ? Notes : UnknownString  }
            };
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public void LoadSettings_Internal(List<string> settings, int offset)
        {
            int i = offset;
            Category = settings[i++];
            Include = StringToBool(settings[i++]);
            Notes = settings[i++];

            if (Category == UnknownString)
                Category = "";
            if (Notes == UnknownString)
                Notes = "";
        }
        public virtual void LoadSettings(List<string> settings)
        {
            LoadSettings_Internal(settings, 0);
        }
    };


    // An "MasterCategoryModel" represents a category that can be assigned to an object to categorise it.
    public class MasterCategoryModel : CategoryModel
    {
        // Is the animal restricted to the ground? That is, unable to climb or fly.
        public bool Terrestrial { get; set; }

        // Minimum valid size of this category of object in cm2
        public float MinSizeCM2 { get; set; }
        // Maximum valid size of this category of object in cm2
        public float MaxSizeCM2 { get; set; }

        // Minimum valid temperature of this category of object
        public float MinTemp { get; set; }
        // Maximum valid temperature of this category of object
        public float MaxTemp { get; set; }


        public MasterCategoryModel(
            string category = "",
            bool include = true,
            string notes = "",
            bool terrestrial = true,
            float minSizeCM2 = 0.0f,
            float maxSizeCM2 = 0.0f,
            float minTemp = 0.0f,
            float maxTemp = 0.0f) : base(category, include, notes)
        {
            Terrestrial = terrestrial;
            MinSizeCM2 = minSizeCM2;
            MaxSizeCM2 = maxSizeCM2;
            MinTemp = minTemp;
            MaxTemp = maxTemp;
        }


        // Constructor used when loading objects from the datastore
        public MasterCategoryModel(List<string> settings)
        {
            if (settings != null)
                LoadSettings(settings);
        }


        public bool Equals(MasterCategoryModel other)
        {
            return
                base.Equals(other) &&
                this.Terrestrial == other.Terrestrial &&
                this.MinSizeCM2 == other.MinSizeCM2 &&
                this.MaxSizeCM2 == other.MaxSizeCM2 &&
                this.MinTemp == other.MinTemp &&
                this.MaxTemp == other.MaxTemp;
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int TerrestrialSetting = 4;
        public const int MinSizeCM2Setting = 5;
        public const int MaxSizeCM2Setting = 6;
        public const int MinTempSetting = 7;
        public const int MaxTempSetting = 8;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public override DataPairList GetSettings()
        {
            var answer = base.GetSettings();

            answer.Add("Terrestrial", Terrestrial);
            answer.Add("MinSizeCM2", MinSizeCM2, AreaCM2Ndp);
            answer.Add("MaxSizeCM2", MaxSizeCM2, AreaCM2Ndp);
            answer.Add("MinTemp", MinTemp, TemperatureNdp);
            answer.Add("MaxTemp", MaxTemp, TemperatureNdp);

            return answer;
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public override void LoadSettings(List<string> settings)
        {
            base.LoadSettings_Internal(settings, 0);

            Terrestrial = StringToBool(settings[TerrestrialSetting - 1]);
            MinSizeCM2 = StringToNonNegFloat(settings[MinSizeCM2Setting - 1]);
            MaxSizeCM2 = StringToNonNegFloat(settings[MaxSizeCM2Setting - 1]);
            MinTemp = StringToNonNegFloat(settings[MinTempSetting - 1]);
            MaxTemp = StringToNonNegFloat(settings[MaxTempSetting - 1]);
        }
    };


    // An "MasterCategoryList" represents all the MasterCategoryModel supported.
    // This is the maximal list of categories that the user picks from when categorising objects.
    public class MasterCategoryList : SortedList<string, MasterCategoryModel>
    {
        public void Add(MasterCategoryModel category)
        {
            Add(category.Category.ToUpper().Trim(), category);
        }


        // Default NZ values
        public void Default()
        {
            Clear();
            Add(new("Bird", true, "", false));
            Add(new("Cat", true, "", false, 120, 600)); // PQR TODO TBC
            Add(new("Dog", true, "", true, 120, 800)); // PQR TODO TBC
            Add(new("Inanimate", false, "", true));
            Add(new("Mammal", true, "4 legged animal")); // PQR TODO TBC
            Add(new("Person", false, "", false, 250, 1200)); // PQR TODO TBC
            Add(new("Pig", true, "", true, 120, 800)); // PQR TODO TBC
            Add(new("Possum", true, "", false, 120, 700));
            Add(new("Rabbit", true, "Rabbit or wallaby", true, 50, 180)); // PQR TODO TBC
            Add(new("Rat", true, "Rat, hedgehog, ferret or stoat", true, 10, 60)); // PQR TODO TBC
            Add(new("Stone", false, "Stone or concrete", true));
            Add(new("Ungulate", true, "Cow, horse, deer or lama", true, 250, 3000)); // PQR TODO TBC
            Add(new("Water", false, "", true));
        }


        public void MaybeDefault()
        {
            if (Count == 0)
                Default();
        }


        // Describe (summarise) the categories.
        public string Describe()
        {
            var answer = "";

            if (Count > 0)
            {
                answer += Count.ToString() + " Categories: ";

                int samples = 0;
                foreach (var category in Values)
                {
                    answer += category.Category;

                    samples++;
                    if (samples >= 5)
                        break;

                    answer += ", ";
                }

                if (Count > samples)
                    answer += " ...";
            }

            return answer;
        }
    };


    // An "ObjectCategoryModel" represents the annotations (if any) added manually to a detected object by the user.
    // The annotation is specific to one object but is deliberately NOT incorporated into ProcessObjectModel.
    // The annotations must survive the video being re-processed in part or in full, multiple times.
    // The annotations are persisted to a annotations-specific tab.
    public class ObjectCategoryModel : CategoryModel
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
        public MasterCategoryList MasterCategories { get; set; } 

        // List of object-specific annotations added by user to detected objects
        public ObjectCategoryList ObjectCategories { get; set; } 


        public CategoryAll()
        {
            MasterCategories = new();
            ObjectCategories = new();
        }
    }
}
