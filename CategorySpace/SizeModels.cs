// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessLogic;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.CategorySpace
{
    public class MasterSizeModel
    {
        public string Name { get; }

        // Top-down area range in cm²
        public int MinAreaCM2 { get; }
        public int MaxAreaCM2 { get; }

        // Animals in this category
        public string Animals { get; }


        public MasterSizeModel(string name, int minAreaCM2, int maxAreaCM2, string animals)
        {
            Name = name;
            MinAreaCM2 = minAreaCM2;
            MaxAreaCM2 = maxAreaCM2;
            Animals = animals;
        }   
    }


    public class MasterSizeModelList
    {
        static public int NumAreas = 7;

        // Cached master list of area models
        static private readonly List<MasterSizeModel> _sizeClasses;
        // Dictionary for quick name-to-index lookup
        static private readonly Dictionary<string, int> _sizeClassIndices;
        // List of tuples for area range lookup
        static private readonly List<(int MinAreaCM2, int MaxAreaCM2, string Name, int Index)> _areaRanges;

        // Static constructor to initialize data once
        static MasterSizeModelList()
        {
            _sizeClasses = new List<MasterSizeModel>
            {
                new("XXS", 0, 100, "Mouse, Rats, Birds"),
                new("XS", 100, 500, "Rats, Rabbits, Possums, Birds"),
                new("S", 500, 1000, "Cats, Possums, Rabbits, Dogs, Person, Birds"),
                new("M", 1000, 2500, "Wallabies, Rabbits, Dogs, Goats, Person, Birds"),
                new("L", 2500, 5000, "Dogs, Goats, Sheep, Pigs, Deer"),
                new("XL", 5000, 10000, "Sheep, Pigs, Deer, Cows"),
                new("XXL", 10000, 20000, "Cows, Deer"),
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

        static public List<MasterSizeModel> Get() => _sizeClasses;

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

            if (areaCM2 < 0)
                return ("XS", 0);
            else
                return ("XXL", NumAreas - 1);
        }


        // Return the count of objects in each size category
        static public List<int> GetObjectCountBySizeClass(ProcessObjList objects, bool significantObjectsOnly = true)
        {
            var answer = new int[NumAreas];
            if (objects != null)
                foreach (var obj in objects)
                    if( obj.Value.Significant || !significantObjectsOnly )
                    {
                        var (_, index) = CM2ToClass((int)obj.Value.SizeCM2);
                        if (index >= 0 && index < NumAreas)
                            answer[index]++;
                    }
            return new List<int>(answer);
        }
    }
}
