// Copyright SkyComb Limited 2024. All rights reserved. 
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold a Yolo object - layer over a sequence of Yolo features.
    public class YoloObject : ProcessObject
    {
        public string ClassName { get; set; }
        public Color ClassColor { get; set; }
        public double ClassConfidence { get; set; }


        public YoloObject(YoloProcess yoloProcess, ProcessScope scope, int legId, YoloFeature firstFeature, string className, Color classColor, double classConfidence) : base(yoloProcess, scope)
        {
            ResetCalcedMemberData();
            FlightLegId = legId;
            ClassName = className;
            ClassColor = classColor;
            ClassConfidence = classConfidence;
            RunFromVideoS = (float)(firstFeature.Block.InputFrameMs / 1000.0);

            ClaimFeature(firstFeature);
        }


        // Constructor used when loaded objects from the datastore
        public YoloObject(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, null)
        {
            ResetCalcedMemberData();
            ClassName = "";
            ClassColor = Color.Black;
            ClassConfidence = 0.66f;
            Significant = true;

            LoadSettings(settings);
        }
    };
}
