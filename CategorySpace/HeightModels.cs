// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessLogic;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.CategorySpace
{
    public class MasterHeightModel
    {
        public string Name { get; set; }
        public double MinHeight { get; set; }
        public double MaxHeight { get; set; }
        public string Description { get; set; }

        public MasterHeightModel(string name, double minHeight, double maxHeight, string description)
        {
            Name = name;
            MinHeight = minHeight;
            MaxHeight = maxHeight;
            Description = description;
        }
    }

    public class MasterHeightModelList
    {
        static public int NumHeights = 8; // Number of height categories

        // Cached master list of height models
        static private readonly List<MasterHeightModel> _heightClasses;
        // Dictionary for quick name-to-index lookup
        static private readonly Dictionary<string, int> _heightClassIndices;
        // List of tuples for height range lookup
        static private readonly List<(double MinHeight, double MaxHeight, string Name, int Index)> _heightRanges;

        // Static constructor to initialize data once
        static MasterHeightModelList()
        {
            _heightClasses = new List<MasterHeightModel>
            {
                new("??", -9999, -2, "Unknown height"),
                new("G", -1.9, 2, "Ground level"),
                new("1", 2.01, 5, "First floor"),
                new("2", 5.01, 8, "Second floor"),
                new("3", 8.01, 11, "Third floor"),
                new("4", 11.01, 14, "Fourth floor"),
                new("5", 14.01, 17, "Fifth floor"),
                new("6+", 17.01, 99999, "Sixth floor and above")
            };

            _heightClassIndices = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            _heightRanges = new List<(double, double, string, int)>();

            for (int i = 0; i < _heightClasses.Count; i++)
            {
                var heightModel = _heightClasses[i];
                // Map name to index
                if (!_heightClassIndices.ContainsKey(heightModel.Name))
                    _heightClassIndices[heightModel.Name] = i;

                // Prepare height ranges for lookup
                _heightRanges.Add((heightModel.MinHeight, heightModel.MaxHeight, heightModel.Name, i));
            }
        }

        static public List<MasterHeightModel> Get() => _heightClasses;


        static public int GetHeightClassIndex(string heightClass)
        {
            return _heightClassIndices.TryGetValue(heightClass, out int index) ? index : -1;
        }


        // Return the MasterHeightModel Name that best matches the height
        static public (string, int) HeightMToClass(double height)
        {
            foreach (var (minHeight, maxHeight, name, index) in _heightRanges)
            {
                if (height >= minHeight && height <= maxHeight)
                    return (name, index);
            }

            if (height < 0)
                return ("??", 0);
            else
                return ("6+", NumHeights - 1);
        }


        // Return the count of objects in each height category
        static public List<int> GetObjectCountByHeightClass(ProcessObjList objects, bool significantObjectsOnly = true)
        {
            var answer = new int[NumHeights];
            if(objects != null)
                foreach (var obj in objects)
                    if (obj.Value.Significant || !significantObjectsOnly)
                    {
                        var (_, index) = HeightMToClass(obj.Value.HeightM);
                        if (index >= 0 && index < NumHeights)
                            answer[index]++;
                    }
                return new List<int>(answer);
        }
    }

}
