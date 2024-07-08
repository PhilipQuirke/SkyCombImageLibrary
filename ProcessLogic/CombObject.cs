// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessModel;
using System.Drawing;


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
            ResetMemberData();

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
            ResetMemberData();

            LoadSettings(settings);
        }


        // Key logic of the Comb model to distinguish significant objects from insignificant, based on data collected.
        //
        // Label this object as "Significant" based on these characteristics:
        // 1) Count: the object has >= 5 hot pixels per block
        //      (avoids very small objects. Includes unreal blocks between real blocks, so unreal blocks work against success)
        // 2) Density: The object's rectangle is at least 20% filled with hot pixels
        //      (deselects large sparce rectangles)
        // 3) Time: Duration we have seen the object for. 
        // 4) Elevation: The object and ground elevations differ significantly - parallex implies object is above ground.
        //
        // Key use cases:
        // 1) Animal in tree, partially visible through foliage, moving faster than ground.
        // 2) Animal on open ground, fully visible, very dense, stationary.
        // 3) Animal on open ground, fully visible, very dense, moving very slowly.
        public void Calculate_Significant()
        {
            try
            {
                // Debugging - Set breakpoint on assignment. Assignment value is overridden later in this proc.
                if (ObjectId == ProcessConfig.FocusObjectId)
                    if (CombProcess.Blocks.Count >= 17)
                        Significant = false;

                // COUNT
                // Maximum pixel count per real feature
                var maxCount = MaxRealHotPixels;
                var countOk = (maxCount > ProcessConfig.ObjectMinPixelsPerBlock); // Say 5 pixels / Block
                var countGood = (maxCount > 2 * ProcessConfig.ObjectMinPixelsPerBlock); // Say 10 pixels / Block
                var countGreat = (maxCount > 4 * ProcessConfig.ObjectMinPixelsPerBlock); // Say 20 pixels / Block

                // DENSITY
                // Average pixel density based on encompassing pixel block of each real feature.
                var minDensity = ProcessConfig.ObjectMinDensityPerc / 100.0F;
                var density = RealDensityPx();
                var densityOk = (density > minDensity); // Say 33%
                var densityGood = (density > 1.5 * minDensity); // Say 50%
                var densityGreat = (density > 2 * minDensity); // Say 66%

                // TIME
                // Aka duration. Proxy for numRealFeatures.
                var seenForMinDurations = SeenForMinDurations();
                var timeOk = (seenForMinDurations >= 1); // Say 500ms
                var timeGood = (seenForMinDurations >= 2); // Say 1000ms
                var timeGreat = (seenForMinDurations >= 4); // Say 2000ms

                // ELEVATION
                // Object with a significant height above ground are more interesting 
                var elevationOK = (HeightM >= 0);
                var elevationGood = (HeightM > 2);
                var elevationGreat = (HeightM > 4);

                // Key calculation of Comb algorithm for identifying significant objects
                Significant =
                    countOk &&
                    densityOk &&
                    timeOk &&
                    (
                        elevationGreat ||
                        countGreat ||
                        densityGreat ||
                        (densityGood && countGood)
                    );

                if (Significant)
                    NumSigBlocks++;

                // Summarise why object is significant or not. Gets displayed in UI and saved to xls.
                Attributes = String.Format("{0}: {1} {2} {3} {4}",
                    Significant ? "Yes" : "No",
                    countGreat ? "C3" : (countGood ? "C2" : (countOk ? "C1" : "c")),
                    densityGreat ? "D3" : (densityGood ? "D2" : (densityOk ? "D1" : "d")),
                    timeGreat ? "T3" : (timeGood ? "T2" : (timeOk ? "T1" : "t")),
                    elevationGreat ? "E3" : (elevationGood ? "E2" : (elevationOK ? "E1" : "e")));
            }
            catch (Exception ex)
            {
                throw ThrowException("CombObject.Calculate_Significant", ex);
            }
        }


        // A CombObject can't be significant until Config.ObjectMinDurationMs has passed.
        // A CombFeature can be significant immediately.
        // This "VaguelySignificant" object code mirrors the feature "Significant" code
        public bool VaguelySignificant()
        {
            // COUNT
            // Maximum pixel count per real feature
            var maxCount = MaxRealHotPixels;
            var maxCountOk = (maxCount > ProcessConfig.ObjectMinPixelsPerBlock); // Say 5 pixels / Block

            // Density
            var minDensity = ProcessConfig.ObjectMinDensityPerc / 100.0F;
            var density = RealDensityPx();
            var densityOk = (density > minDensity); // Say 20%

            return maxCountOk && densityOk;
        }


        // Calculate the simple (int, float, VelocityF, etc) member-data of this real object.
        // Calculates LocationM, LocationErrM, HeightM, HeightErrM, etc.
        public override void Calculate_RealObject_SimpleMemberData_Core()
        {
            try
            {
                base.Calculate_RealObject_SimpleMemberData_Core();
 
                // Is this OBJECT significant?
                Calculate_Significant();
            }
            catch (Exception ex)
            {
                throw ThrowException("CombObject.Calculate_RealObject_SimpleMemberData_Core", ex);
            }
        }


        // Calculate the simple (int, float, VelocityF, etc) member-data of this real object.
        // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
        public void Calculate_RealObject_SimpleMemberData()
        {
            // Calculate the drone SumLinealM distance corresponding to the centroid of the object
            Calculate_AvgSumLinealM();

            Calculate_RealObject_SimpleMemberData_Core();
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        // But only if the object reamins viable after claiming feature (e.g. doesn't get too big or density too low).
        public bool ClaimFeature(CombFeature theFeature)
        {
            try
            {
                var lastFeature = LastFeature as CombFeature;
                if ((theFeature.Type == FeatureTypeEnum.Real) && (lastFeature != null))
                {
                    // To get here, theFeature overlaps this object significantly.
                    // But claiming theFeature can make this object exceed FeatureMaxSize
                    // or reduce the density below FeatureMinDensityPerc, potentially making the object insignificant.

                    if (theFeature.FeatureOverSized)
                        return false;
                    if (!theFeature.PixelDensityGood)
                        return false;

                    // ToDo: This approach allows one bad block before it stops growth. Bad. May make object insignificant.
                    if (lastFeature.FeatureOverSized)
                        return false;
                    if ((lastFeature.Type == FeatureTypeEnum.Real) && // Unreal features have no density
                        !lastFeature.PixelDensityGood)
                        return false;
                }


                // Associate the feature with this object.
                Assert(theFeature.ObjectId <= 0, "CombObject.ClaimFeature: Feature is already owned.");
                theFeature.ObjectId = this.ObjectId;

                // Debugging - Set breakpoint on assignment. (Value is unchanged by assignment)
                if (ObjectId == ProcessConfig.FocusObjectId)
                    theFeature.ObjectId = this.ObjectId;

                bool wasSignificant = Significant;

                // Is object a real feature?
                if (theFeature.Type == FeatureTypeEnum.Real)
                {
                    theFeature.IsTracked = true;
                    MaxRealHotPixels = Math.Max(MaxRealHotPixels, theFeature.NumHotPixels);

                    var theBlock = theFeature.Block;

                    if ((LastRealFeature == null) || (LastRealFeature.Block.BlockId < theBlock.BlockId))
                    {
                        // First real feature claimed by this object for THIS block
                        ProcessFeatures.AddFeature(theFeature);
                        LastRealFeatureId = theFeature.FeatureId;
                        RunToVideoS = (float)(theFeature.Block.InputFrameMs / 1000.0);
                        MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, theFeature.PixelBox.Width);
                        MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, theFeature.PixelBox.Height);
                    }
                    else
                    {
                        // This object is claiming a second or third feature for this block.
                        // Use case is a large rectangle in previous block, getting replaced by 2 or 3 smaller rectangles in this block.
                        // For better visualisation we want to combine all features in this block into one.

                        // The first real feature for the last block consumes theFeature, leaving theFeature empty.
                        LastRealFeature.Consume(theFeature);
                        theFeature.ObjectId = UnknownValue;

                        MaxRealPixelWidth = Math.Max(MaxRealPixelWidth, LastRealFeature.PixelBox.Width);
                        MaxRealPixelHeight = Math.Max(MaxRealPixelHeight, LastRealFeature.PixelBox.Height);
                    }

                    // Calculate the simple member data (int, float, VelocityF, etc) of this real object.
                    // Calculates DemM, LocationM, LocationErrM, HeightM, HeightErrM, AvgSumLinealM, etc.
                    Calculate_RealObject_SimpleMemberData();
                }
                else if (theFeature.Type == FeatureTypeEnum.Unreal)
                {
                    // theFeature is unreal - it is a persistance object
                    ProcessFeatures.AddFeature(theFeature);

                    LastFeature.HeightM = HeightM;
                    LastFeature.HeightAlgorithm = CombFeature.UnrealCopyHeightAlgorithm;
                }


                // Copy these details to the feature to be saved in the DataStore.
                // Useful for understanding the feature by feature progression of values that are refined over time.
                LastFeature.Significant = Significant & (LastFeature.Type == FeatureTypeEnum.Real);
                LastFeature.Attributes = Attributes;
                if (Significant && !wasSignificant)
                    // This object has just become significant. Two use cases:
                    // - After 5 real features, enough time has based for object to become significant on 6th real feature.
                    // - Object had 20 real features, then 5 unreal features, then another 1 real features.
                    // Mark all (real and unreal) features associated with this object as significant.
                    foreach (var feature in ProcessFeatures)
                        feature.Value.Significant = true;

                // Debugging - Set breakpoint on assignment. 
                if (ObjectId == ProcessConfig.FocusObjectId)
                    if ((theFeature.FeatureId == 41) || (theFeature.FeatureId == 59))
                        LastFeature.Attributes = this.Attributes + ".";

                return true;
            }
            catch (Exception ex)
            {
                throw ThrowException("CombObject.ClaimFeature", ex);
            }
        }


        // Object will claim ownership of this feature extending the objects lifetime and improving its "Significant" score.
        // In rare cases, object can claim multiple features from a single block (e.g. a tree branch bisects a heat spot into two features) 
        // But only if the object reamins viable after claiming feature (e.g. doesn't get too big or density too low).
        public bool MaybeClaimFeature(CombFeature feature, Rectangle objectExpectedPixelBox)
        {
            if (feature.ObjectId == 0) // Not claimed yet
                if (feature.Significant || this.Significant)
                    if (feature.SignificantPixelBoxIntersection(objectExpectedPixelBox))
                        // Object will claim feature if the object remains viable after claiming feature
                        return ClaimFeature(feature);

            return false;
        }

    };



    // A list of Comb objects
    public class CombObjs
    {
        private readonly CombProcess Model;

        public ProcessObjList CombObjList;


        public CombObjs(CombProcess model)
        {
            Model = model;
            CombObjList = new();
        }


        public void Add(ProcessScope scope, CombFeature firstFeature)
        {
            var theObject = ProcessFactory.NewCombObject(scope, Model, firstFeature);
            CombObjList.AddObject(theObject);
        }


        // Number of objects that have ever been significant. 
        // Not same as num objs significant in the current Block, as objects can become insignificant. 
        public int NumEverSignificantObjects
        {
            get
            {
                return CombObjList.NumEverSignificantObjects;
            }
        }


        public void StopTracking()
        {
            CombObjList.StopTracking();
        }


        public string DescribeSignificantObjects()
        {
            var num = NumEverSignificantObjects;
            return (num == 0 ? "" : string.Format("{0} Objects", num));
        }
    }
}
