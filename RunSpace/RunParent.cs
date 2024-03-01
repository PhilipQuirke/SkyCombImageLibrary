// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV.Structure;
using Emgu.CV;


namespace SkyCombImage.RunSpace
{
    abstract public class RunParent
    {
        protected RunParent() { }


        public abstract void RefreshAll();

        public abstract void DrawUI(
            RunVideo runVideo,
            Image<Bgr, byte> inputFrame,
            Image<Bgr, byte> displayFrame);

        public abstract void DrawObjectGrid(RunVideo runVideo, bool showObjectGrid);

        public abstract void ShowStepProgress(RunVideo runVideo);

        public abstract void BadDuration(RunVideo runVideo);

        public abstract void ShowRunSummary(string summary);
    }

}
