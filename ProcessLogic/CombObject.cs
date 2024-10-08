// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombImage.ProcessModel;


namespace SkyCombImage.ProcessLogic
{
    // A significant Comb object - a logical object derived from overlapping features over successive frames. 
    public class CombObject : ProcessObject
    {
        // Parent process model
        private CombProcess CombProcess { get; }


        // Constructor used processing video 
        public CombObject(ProcessScope scope, CombProcess combProcess, CombFeature firstFeature) : base(combProcess, scope)
        {
            CombProcess = combProcess;
            ResetCalcedMemberData();

            if (firstFeature != null)
            {
                Assert(firstFeature.Type == FeatureTypeEnum.Real, "Initial feature must be Real");
                ClaimFeature(firstFeature);
            }
        }


        // Constructor used when loaded objects from the datastore
        public CombObject(CombProcess combProcess, List<string> settings) : base(combProcess, null)
        {
            CombProcess = combProcess;
            ResetCalcedMemberData();

            LoadSettings(settings);
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        // But only if the object remains viable after claiming feature (e.g. doesn't get too big or density too low).
        public override bool ClaimFeature(ProcessFeature theFeature)
        {
            var lastFeature = LastFeature;
            if ((theFeature.Type == FeatureTypeEnum.Real) && (lastFeature != null))
            {
                // To get here, theFeature overlaps this object significantly.
                // But claiming theFeature can make this object exceed FeatureMaxSize
                // or reduce the density below FeatureMinDensityPerc, potentially making the object insignificant.

                if (theFeature.FeatureOverSized)
                    return false;

                // ToDo: This approach allows one bad block before it stops growth. Bad. May make object insignificant.
                if (lastFeature.FeatureOverSized)
                    return false;
            }

            return base.ClaimFeature(theFeature);
        }
    };

}
