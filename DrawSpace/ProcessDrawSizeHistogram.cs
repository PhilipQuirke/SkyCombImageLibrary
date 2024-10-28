// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombImage.ProcessLogic;


namespace SkyCombImage.DrawSpace
{
    // Code to draw histogram of object sizes as XXS to XXL.
    public class ProcessDrawSizeHistogram : DrawHistogram
    {
        ProcessAll ProcessAll { get; }


        public ProcessDrawSizeHistogram(ProcessAll processAll, ProcessDrawScope drawScope, List<int> values) : base(drawScope, values, 0, values.Count - 1)
        {
            ProcessAll = processAll;

            HorizLeftLabel = "XXS";
            HorizRightLabel = "XXL";
        }
    }


    // Code to draw histogram of object heights as ??, G to 6+.
    public class ProcessDrawHeightHistogram : DrawHistogram
    {
        ProcessAll ProcessAll { get; }


        public ProcessDrawHeightHistogram(ProcessAll processAll, ProcessDrawScope drawScope, List<int> values) : base(drawScope, values, 0, values.Count - 1)
        {
            ProcessAll = processAll;

            HorizLeftLabel = "??     G     1";
            HorizRightLabel = "6+";
        }
    }
}