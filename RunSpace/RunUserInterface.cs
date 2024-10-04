// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using System.Windows.Forms;


namespace SkyCombImage.RunSpace
{
    public abstract class RunUserInterface
    {
        protected RunUserInterface() { }


        public abstract void RefreshAll();

        public abstract void DrawUI(RunVideo runVideo);

        public abstract void DrawObjectGrid(RunVideo runVideo, bool showObjectGrid);

        public abstract void ShowStepProgress(RunVideo runVideo, int intervalCount, int stepCount);

        public abstract void BadDuration(RunVideo runVideo);

        public abstract void ShowRunSummary(string summary);

        public virtual bool UnitTestRunning() { return false; }


        public abstract RunConfig? RunConfig();
        public abstract void LegsForm_CopyMainFormButtonState(object legsForm);
        public virtual Button? MainForm_ButtonRun() { return null; }
        public virtual void MainForm_ButtonRun_Click(object sender, EventArgs args) { }
        public virtual void MainForm_LegNButton_Click(object sender, EventArgs args) { }
        public virtual void MainForm_LegsAllButton_Click(object sender, EventArgs args) { }
        public virtual void MainForm_SetLegRange(int from, int to) { }

        public virtual void StorePictures(Image<Bgr, byte> inputFrame, Image<Bgr, byte> displayFrame) { }
        public virtual void DrawObjectGrid() { }
        public virtual void ShowObjectCategories() { }
    }

}
