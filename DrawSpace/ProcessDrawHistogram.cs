// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombImage.CategorySpace;


namespace SkyCombImage.DrawSpace
{
    // Code to draw histogram of object sizes as XXS to XXL.
    public class ProcessDrawSizeHistogram : DrawHistogram
    {
        public ProcessDrawSizeHistogram(ProcessDrawScope drawScope, ObjectDrawScope? objectDrawScope, List<int> values) : base(drawScope, values, 0, MasterSizeModelList.NumAreas - 1)
        {

            HorizLeftLabel = "XXS";
            HorizRightLabel = "XXL";

            if (objectDrawScope != null)
            {
                FilterMin = objectDrawScope.MinSizeIndex;
                FilterMax = objectDrawScope.MaxSizeIndex;
            }
        }
    }


    // Code to draw histogram of object heights as ?, G, 1f to 6f+.
    public class ProcessDrawHeightHistogram : DrawHistogram
    {
        public ProcessDrawHeightHistogram(ProcessDrawScope drawScope, ObjectDrawScope? objectDrawScope, List<int> values) : base(drawScope, values, 0, MasterHeightModelList.NumHeights - 1)
        {
            HorizLeftLabel = "?      G       1f";
            HorizRightLabel = "6f+";

            if (objectDrawScope != null)
            {
                FilterMin = objectDrawScope.MinHeightIndex;
                FilterMax = objectDrawScope.MaxHeightIndex;
            }
        }
    }
}