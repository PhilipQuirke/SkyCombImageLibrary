// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using System.Drawing;


namespace SkyCombImage.ProcessModel
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

            Assert(firstFeature.Type == FeatureTypeEnum.Real, "Initial feature must be Real");
            ClaimFeature(firstFeature);
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


        // Number of real features owned by this object.
        public int NumRealFeatures()
        {
            int answer = 0;

            // Rarely, the object may have a sequence of real, then unreal, then real features.
            foreach (var feature in ProcessFeatures)
                if (feature.Value.Type == FeatureTypeEnum.Real)
                    answer++;

            return answer;
        }


        // Given this object's last known position, and the object's
        // average velocity, where do we expect the object to be this block?
        public Rectangle ExpectedLocationThisBlock()
        {
            Rectangle answer;

            var firstFeat = FirstFeature;
            var lastFeat = LastFeature;

            var firstBox = firstFeat.PixelBox;
            var lastBox = lastFeat.PixelBox;

            int lastWidth = lastBox.Width;
            int lastHeight = lastBox.Height;

/*
            var numBlockSteps = lastFeat.BlockId - firstFeat.BlockId + 1;
            if (numBlockSteps >= 2)
            {
                // In DJI_0118 leg 3, Object 1 starts large but fades
                // so that LastFeature().PixelBox is very small.
                // The expected location should use the maximum object size.
                int addWidth = Math.Max(0, COM.MaxRealPixelWidth - lastWidth);
                int addHeight = Math.Max(0, COM.MaxRealPixelHeight - lastHeight);

                // We have multiple features. Use their difference in location.
                var distanceX = 1.0F * lastBox.X + lastBox.Width / 2.0F - firstBox.X - lastBox.Width / 2.0F;
                var distanceY = 1.0F * lastBox.Y + lastBox.Height / 2.0F - firstBox.Y - lastBox.Height / 2.0F;
                var numMoves = numBlockSteps - 1;

                // Advance one average stride from the previous location.
                answer = new Rectangle(
                    (int)(
                    lastBox.X + distanceX / numMoves - addWidth / 2.0f),
                    (int)(lastBox.Y + distanceY / numMoves - addHeight / 2.0f),
                    lastWidth + addWidth,
                    lastHeight + addHeight);
            }
            else
*/
                // With one feature we dont know the object's velocity across the image.
                // Rely on image overlap
                answer = new Rectangle(
                    lastBox.X,
                    lastBox.Y,
                    lastWidth,
                    lastHeight);

            if (lastFeat.Type == FeatureTypeEnum.Real)
                // We don't want a drone wobble to break the object feature sequence
                // So we inflate the expected location by 5 pixels in each direction.
                answer.Inflate(5, 5);

            return answer;
        }


        // This object claims this feature
        public void ClaimFeature(YoloFeature theFeature)
        {
            Assert(theFeature.ObjectId <= 0, "YoloObject.ClaimFeature: Feature is already owned");

            theFeature.ObjectId = this.ObjectId;
            ProcessFeatures.AddFeature(theFeature);

            NumSigBlocks = LastFeature.BlockId - FirstFeature.BlockId + 1;
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        // But only if the object reamins viable after claiming feature (e.g. doesn't get too big or density too low).
        public bool MaybeClaimFeature(YoloFeature feature, Rectangle objectExpectedPixelBox)
        {
            if (feature.ObjectId == 0) // Not claimed yet
                if (feature.Significant || this.Significant)
                    if (feature.SignificantPixelBoxIntersection(objectExpectedPixelBox))
                    {
                        // Object will claim feature if the object remains viable after claiming feature
                        ClaimFeature(feature);
                        return true;
                    }

            return false;
        }


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        override public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "Object", ObjectId },
                { "FromS", RunFromVideoS, SecondsNdp },
                { "ToS", RunToVideoS, SecondsNdp },
                { "Attributes", Attributes },
                { "Significant", Significant },
                { "#SigBlocks", NumSigBlocks },
                { "FirstFeat", (FirstFeature == null) ? -1 : FirstFeature.FeatureId },
                { "LastFeat", (LastFeature == null) ? -1 : LastFeature.FeatureId },
            };
        }
    };


    public class YoloObjectList : SortedList<int, YoloObject>
    {
        private YoloProcess YoloProcess;


        // We do not process objects below this index in the object array.
        public int LegFirstIndex;


        public YoloObjectList(YoloProcess yoloProcess)
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
        public void ProcessLegStart()
        {
            LegFirstIndex = Count;
        }
    };
}
