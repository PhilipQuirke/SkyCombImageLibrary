// Copyright SkyComb Limited 2024. All rights reserved.
using System.Windows.Forms;


namespace SkyCombImage.RunSpace
{
    abstract public class RunUserInterface
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
        public virtual void LegsForm_CopyMainFormButtonState(object legsForm) { }
        public virtual Button? MainForm_ButtonRun() { return null; }
        public virtual void MainForm_ButtonRun_Click(object sender, EventArgs args) { }
        public virtual void MainForm_LegNButton_Click(object sender, EventArgs args) { }
        public virtual void MainForm_LegsAllButton_Click(object sender, EventArgs args) { }
    }

}
