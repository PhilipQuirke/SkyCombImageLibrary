// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombImage.ProcessLogic;


namespace SkyCombImage.DrawSpace
{
    // Code to draw histogram of object sizes as XXS to XXL.
    public class ProcessDrawHistogram : DrawHistogram
    {
        ProcessAll ProcessAll { get; }


        public ProcessDrawHistogram(ProcessAll processAll, ProcessDrawScope drawScope, List<int> values) : base(drawScope, values, 0, values.Count - 1)
        {
            ProcessAll = processAll;

            HorizLeftLabel = "XXS";
            HorizRightLabel = "XXL";
        }
    }
}