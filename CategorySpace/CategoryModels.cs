// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;


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


        public MasterCategoryModel(
            string category = "",
            bool include = true,
            string notes = "",
            bool terrestrial = true,
            float minSizeCM2 = 0.0f,
            float maxSizeCM2 = 0.0f) : base(category, include, notes)
        {
            Terrestrial = terrestrial;
            MinSizeCM2 = minSizeCM2;
            MaxSizeCM2 = maxSizeCM2;
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
                this.MaxSizeCM2 == other.MaxSizeCM2;

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
        }
    };


    public class MasterAreaModel
    {
        public string Name { get; }

        // Top-down area range in cm²
        public int MinAreaCM2 { get; }
        public int MaxAreaCM2 { get; }

        // Animals in this category
        public string Animals { get; }

        public MasterAreaModel(string name, int minAreaCM2, int maxAreaCM2, string animals)
        {
            Name = name;
            MinAreaCM2 = minAreaCM2;
            MaxAreaCM2 = maxAreaCM2;
            Animals = animals;
        }   
    }


    public class MasterSizeClassList
    {
        static public int NumAreas = 8;

        // Cached master list of area models
        static private readonly List<MasterAreaModel> _sizeClasses;
        // Dictionary for quick name-to-index lookup
        static private readonly Dictionary<string, int> _sizeClassIndices;
        // List of tuples for area range lookup
        static private readonly List<(int MinAreaCM2, int MaxAreaCM2, string Name, int Index)> _areaRanges;

        // Static constructor to initialize data once
        static MasterSizeClassList()
        {
            _sizeClasses = new List<MasterAreaModel>
            {
                new("XXS", 0, 100, "Mouse, Rats, Birds"),
                new("XS", 100, 500, "Rats, Rabbits, Possums, Birds"),
                new("S", 500, 1000, "Cats, Possums, Rabbits, Dogs, Person, Birds"),
                new("M", 1000, 2500, "Wallabies, Rabbits, Dogs, Goats, Person, Birds"),
                new("L", 2500, 5000, "Dogs, Goats, Sheep, Pigs, Deer"),
                new("XL", 5000, 10000, "Sheep, Pigs, Deer, Cows"),
                new("XXL", 10000, 20000, "Cows, Deer"),
                new("XXL", 20000, 99999, "Water")
            };

            _sizeClassIndices = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            _areaRanges = new List<(int, int, string, int)>();

            for (int i = 0; i < _sizeClasses.Count; i++)
            {
                var areaModel = _sizeClasses[i];
                // Map name to index
                if (!_sizeClassIndices.ContainsKey(areaModel.Name))
                    _sizeClassIndices[areaModel.Name] = i;

                // Prepare area ranges for lookup
                _areaRanges.Add((areaModel.MinAreaCM2, areaModel.MaxAreaCM2, areaModel.Name, i));
            }
        }

        static public List<MasterAreaModel> Get() => _sizeClasses;

        static public int GetSizeClassIndex(string sizeClass)
        {
            return _sizeClassIndices.TryGetValue(sizeClass, out int index) ? index : -1;
        }

        // Return the MasterAreaModel Name that best matches the size
        static public (string, int) CM2ToClass(int areaCM2)
        {
            foreach (var (minArea, maxArea, name, index) in _areaRanges)
            {
                if (areaCM2 >= minArea && areaCM2 <= maxArea)
                    return (name, index);
            }
            return ("Unknown", -1);
        }

        // Return the count of objects in each size category
        static public List<int> GetObjectCountBySizeClass(ProcessObjList objects)
        {
            var answer = new int[NumAreas];
            foreach (var obj in objects)
            {
                var (_, index) = CM2ToClass((int)obj.Value.SizeCM2);
                if (index >= 0 && index < NumAreas)
                    answer[index]++;
            }
            return new List<int>(answer);
        }
    }


    // An "MasterCategoryList" represents all the MasterCategoryModel supported.
    // This is the maximal list of categories that the user picks from when categorising objects.
    public class MasterCategoryList : SortedList<string, MasterCategoryModel>
    {
        public void Add(MasterCategoryModel category)
        {
            Add(category.Category.ToUpper().Trim(), category);
        }


        // Default NZ values. Areas assume animal seen from above has a ellipsoid shape.
        public void Default()
        {
            Clear();

            Add(new("Mouse", true, "", false, 19, 24));
            // Smallest(standing): House mouse (~8 cm × ~3 cm) = 19 cm²
            // Largest(lying down): Slightly larger house mouse (~10 cm × ~3 cm) = 24 cm²

            Add(new("Rat", true, "", false, 63, 137));
            // Smallest(standing): Ship rat (~16 cm × ~5 cm) = 63 cm²
            // Largest(lying down): Norway rat (~25 cm × ~7 cm) = 137 cm²

            Add(new("Rabbit", true, "", true, 157, 1885));
            // Smallest(standing): Netherland Dwarf (~20 cm × ~10 cm) = 157 cm²
            // Largest(lying down): Flemish Giant (~80 cm × ~30 cm) = 1,885 cm²

            Add(new("Cat", true, "", false, 541, 942));
            // Smallest(standing): Small domestic cat (~46 cm × ~15 cm) = 541 cm²
            // Largest(lying down): Large domestic cat (~60 cm × ~20 cm) = 942 cm²

            Add(new("Possum", true, "", false, 377, 911));
            // Smallest(standing): Common brushtail possum (~32 cm × ~15 cm) = 377 cm²
            // Largest(lying down): Larger brushtail possum (~58 cm × ~20 cm) = 911 cm²

            Add(new("Bird", true, "", false, 39, 3162));
            // Smallest(standing): Fantail (~10 cm × ~5 cm) = 39 cm²
            // Largest(lying down): Royal albatross (~115 cm × ~35 cm) = 3,162 cm²

            Add(new("Wallaby", true, "", true, 785, 1767)); 
            // Smallest(standing): Dama wallaby (~50 cm × ~20 cm) = 785 cm²
            // Largest(lying down): Bennett's wallaby (~90 cm × ~25 cm) = 1,767 cm²

            Add(new("Dog", true, "", true, 353, 2827));
            // Smallest(standing): Jack Russell Terrier(~30 cm × ~15 cm) = 353 cm²
            // Largest(lying down): German Shepherd(~100 cm × ~36 cm) = 2,827 cm²

            Add(new("Goat", true, "", true, 1767, 4712));
            // Smallest(standing): Arapawa goat(~75 cm × ~30 cm) = 1,767 cm²
            // Largest(lying down): Boer goat(~120 cm × ~50 cm) = 4,712 cm²

            Add(new("Sheep", true, "", true, 3142, 6123));
            // Smallest(standing): Merino sheep(~100 cm × ~40 cm) = 3,142 cm²
            // Largest(lying down): Romney sheep(~130 cm × ~60 cm) = 6,123 cm²

            Add(new("Pig", true, "", true, 3927, 9889));
            // Smallest(standing): Kunekune pig (~100 cm × ~50 cm) = 3,927 cm²
            // Largest(lying down): Large White pig(~180 cm × ~70 cm) = 9,889 cm²

            Add(new("Deer", true, "", true, 3024, 10996));
            // Smallest(standing): Fallow deer (~110 cm × ~35 cm) = 3,024 cm²
            // Largest(lying down): Red deer (~200 cm × ~70 cm) = 10,996 cm²

            Add(new("Cow", true, "", true, 9889, 18850));
            // Smallest(standing): Jersey cow (~180 cm × ~70 cm) = 9,889 cm²
            // Largest(lying down): Holstein Friesian cow (~240 cm × ~100 cm) = 18,850 cm²

            Add(new("Person", false, "", true, 943, 1571));
            // Smallest(standing): Adult of smaller stature (~30 cm × ~40 cm) = 943 cm²
            // Largest(lying down): Taller adult (~40 cm × ~50 cm) = 1,571 cm²

            Add(new("Inanimate", false, "", true));

            Add(new("Stone", false, "", true));

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
