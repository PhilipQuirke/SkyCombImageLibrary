// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;
using Emgu.CV.Features2D;



namespace SkyCombImage.ProcessLogic
{
    // A process feature combines the stored data and some logic
    public class ProcessFeature : ProcessFeatureModel
    {
        // Parent process model
        protected ProcessAll ProcessAll { get; }

        // A feature is associated 1-1 with a Block
        public ProcessBlock Block { get; set; }


        public ProcessFeature(ProcessAll processAll, int blockId, FeatureTypeEnum type) : base(blockId, type)
        {
            ResetMemberData();

            ProcessAll = processAll;
            Block = processAll.Blocks[blockId];
        }


        // Constructor used when loaded objects from the datastore
        public ProcessFeature(ProcessAll processAll, List<string> settings) : base(settings)
        {
            ProcessAll = processAll;
            Block = ProcessAll.Blocks[BlockId];
        }
    }


    // A list of Comb features bound to a specific Block
    abstract public class ProcessFeatureList : SortedList<int, ProcessFeature>
    {
        private static ProcessConfigModel ProcessConfig;

        public ProcessFeature? FirstFeature { get { return (Count == 0 ? null : Values[0]); } }
        // Last (Real or UnReal) feature claimed by this object. May be null.
        public ProcessFeature? LastFeature { get { return (Count == 0 ? null : Values[^1]); } }


        public ProcessFeatureList(ProcessConfigModel config)
        {
            ProcessFeatureList.ProcessConfig = config;
        }


        public void AddFeature(ProcessFeature feature)
        {
            BaseConstants.Assert(feature != null, "AddFeature: Feature not specified.");
            BaseConstants.Assert(feature.FeatureId > 0, "AddFeature: No Id");

            this.Add(feature.FeatureId, feature);
        }


        // Add the feature list for a new block into this CombFeatureListList 
        public void AddFeatureList(ProcessFeatureList featuresToAdd)
        {
            if (featuresToAdd != null)
                foreach (var feature in featuresToAdd)
                    AddFeature(feature.Value);
        }


        protected abstract ProcessFeatureList Create(ProcessConfigModel config);


        public ProcessFeatureList Clone()
        {
            var answer = Create(ProcessConfig);

            foreach (var feature in this)
                answer.AddFeature(feature.Value);

            return answer;
        }


        // Return the number of significant features in this list
        public int NumSig
        {
            get
            {
                int numSig = 0;
                foreach (var theFeature in this)
                    if (theFeature.Value.Significant)
                        numSig++;
                return numSig;
            }
        }

    };
}
