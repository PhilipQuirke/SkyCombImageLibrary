// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.ProcessLogic
{
    // A class to hold a Yolo object - layer over a sequence of Yolo features.
    public class YoloObject : ProcessObject
    {
        public string ClassName { get; set; }
        public Color ClassColor { get; set; }
        public float ClassConfidence { get; set; }


        public YoloObject(YoloProcess yoloProcess, ProcessScope scope, int legId, YoloFeature firstFeature, string className, Color classColor, float classConfidence) : base(yoloProcess, scope)
        {
            Assert(firstFeature.Type == FeatureTypeEnum.Real, "Initial feature must be Real");

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


        // This object claims this feature
        public override bool ClaimFeature(ProcessFeature theFeature)
        {
            Assert(theFeature.ObjectId <= 0, "YoloObject.ClaimFeature: Feature is already owned");

            theFeature.ObjectId = this.ObjectId;

            ProcessFeatures.AddFeature(theFeature);

            LastRealFeatureId = theFeature.FeatureId;
            RunToVideoS = (float)(theFeature.Block.InputFrameMs / 1000.0);
            NumSigBlocks = theFeature.BlockId - FirstFeature.BlockId + 1;
            MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, theFeature.PixelBox.Width);
            MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, theFeature.PixelBox.Height);
            MaxRealHotPixels = Math.Max(MaxRealHotPixels, theFeature.NumHotPixels);
            Significant = true;

            // Calculate the simple member data (int, float, VelocityF, etc) of this real object.
            // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
            Calculate_RealObject_SimpleMemberData();

            return true;
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        public bool MaybeClaimFeature(YoloFeature feature, Rectangle objectExpectedPixelBox)
        {
            if (feature.ObjectId == 0) // Not claimed yet
                if (feature.SignificantPixelBoxIntersection(objectExpectedPixelBox))
                {
                    // Object will claim feature if the object remains viable after claiming feature
                    ClaimFeature(feature);
                    return true;
                }

            return false;
        }
    };
}
