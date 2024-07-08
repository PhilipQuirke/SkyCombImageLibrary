// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;
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


        public YoloObject(YoloProcess yoloProcess, ProcessScope scope, YoloFeature firstFeature, string className, Color classColor, float classConfidence) : base(yoloProcess, scope)
        {
            ResetMemberData();
            ClassName = className;
            ClassColor = classColor;
            ClassConfidence = classConfidence;
            Significant = true;
            RunFromVideoS = (float)(firstFeature.Block.InputFrameMs / 1000.0);

            Assert(firstFeature.Type == FeatureTypeEnum.Real, "Initial feature must be Real");
            ClaimFeature(firstFeature);
        }


        // Constructor used when loaded objects from the datastore
        public YoloObject(YoloProcess yoloProcess, List<string> settings) : base(yoloProcess, null)
        {
            ResetMemberData();

            LoadSettings(settings);
        }


        // Reset member data to mirror a newly created object.
        // Used in experimentation to allow repeated calculation run against this object.
        public override void ResetMemberData()
        {
            base.ResetMemberData();
            ClassName = "";
            ClassColor = Color.Black;
            ClassConfidence = 0.66f;
        }


        // This object claims this feature
        public void ClaimFeature(YoloFeature theFeature)
        {
            if(theFeature.ObjectId > 0)
                Assert(theFeature.ObjectId <= 0, "YoloObject.ClaimFeature: Feature is already owned");

            theFeature.ObjectId = this.ObjectId;
            ProcessFeatures.AddFeature(theFeature);

            LastRealFeatureId = theFeature.FeatureId;
            RunToVideoS = (float)(theFeature.Block.InputFrameMs / 1000.0);
            NumSigBlocks = theFeature.BlockId - FirstFeature.BlockId + 1;
            MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, theFeature.PixelBox.Width);
            MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, theFeature.PixelBox.Height);
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


    public class YoloObjList : ProcessObjList
    {
        private YoloProcess YoloProcess;


        // We do not process objects below this index in the object array.
        public int LegFirstIndex;


        public YoloObjList(YoloProcess yoloProcess)
        {
            YoloProcess = yoloProcess;
        }


        public YoloObject AddObject(YoloObject theObject)
        {
            BaseConstants.Assert(theObject.ObjectId > 0, "YoloObjectList.AddObject: No Id");
            Add(theObject.ObjectId, theObject);
            return theObject;
        }
        public YoloObject AddObject(ProcessScope scope, YoloFeature firstFeature)
        {
            BaseConstants.Assert(firstFeature != null, "YoloObjectList.AddObject: No firstFeature");

            string className = firstFeature.BoundingBox != null ? firstFeature.BoundingBox.Class.Name : "??";
            float classConfidence = firstFeature.BoundingBox != null ? firstFeature.BoundingBox.Confidence : 0.6f;
            /*
            newObject.ClassId = box.Class.Id;
            newObject.ClassType = box.Class.Type;
            newObject.ClassDescription = box.Class.Description;
            */

            return AddObject(new YoloObject(YoloProcess, scope, firstFeature, className, Color.Red, classConfidence));
        }


        // No existing objects should be tracked at the start of a new leg
        public override void StopTracking()
        {
            base.StopTracking();
            LegFirstIndex = Count;
        }
    };
}
