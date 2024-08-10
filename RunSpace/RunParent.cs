// Copyright SkyComb Limited 2024. All rights reserved.


namespace SkyCombImage.RunSpace
{
    abstract public class RunParent
    {
        protected RunParent() { }


        public abstract void RefreshAll();

        public abstract void DrawUI(RunVideo runVideo);

        public abstract void DrawObjectGrid(RunVideo runVideo, bool showObjectGrid);

        public abstract void ShowStepProgress(RunVideo runVideo, int intervalCount, int stepCount);

        public abstract void BadDuration(RunVideo runVideo);

        public abstract void ShowRunSummary(string summary);
    }

}
