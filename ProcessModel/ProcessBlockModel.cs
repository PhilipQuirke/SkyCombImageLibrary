// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    public class ProcessBlockModel : TardisModel
    {
        // Unique identifier of the processing block. One-based.
        public int BlockId { get { return TardisId; } }


        // The calculated FlightStep info derived from FlightSection & Ground data. 
        // Refer https://github.com/PhilipQuirke/SkyCombAnalystHelp/Drone.md
        // and section "Drone Overall Accuracy".
        // Due to sampling, a few successive blocks can share the same FlightStep.
        public int FlightStepId { get; set; }


        // Corresponding flight leg. One-based.
        public int FlightLegId { get; set; }
        public string FlightLegName { get { return IdToLetter(FlightLegId); } }


        // ------ Input Video Position Data -----
        // Position in the input video in Frames, corresponding to this block. One-based. No offsets applied - straight from input video.
        public int InputFrameId { get; set; }
        // Position in the input video in milliseconds, corresponding to this block. No offsets applied - straight from input video.
        public int InputFrameMs { get; set; }


        // ------ Display Video Position Data (if any) -----
        // Position in the display video in Frames, corresponding to this block. One-based. No offsets applied - straight from display video.
        public int DisplayFrameId { get; set; }
        // Position in the display  video in milliseconds, corresponding to this block. No offsets applied - straight from display video.
        public int DisplayFrameMs { get; set; }



        // ------ Min / Max Features associated with this block -----
        public int MinFeatureId { get; set; }
        public int MaxFeatureId { get; set; }


        // Number of significant objects in the block. 
        public int NumSig { get; set; }


        public ProcessBlockModel(ProcessScopeModel scope) : base(scope.CurrBlockId)
        {
            FlightStepId = UnknownValue;
            FlightLegId = scope.CurrRunLegId;
            InputFrameId = scope.CurrInputFrameId;
            InputFrameMs = scope.CurrInputFrameMs;
            DisplayFrameId = scope.CurrDisplayFrameId;
            DisplayFrameMs = scope.CurrDisplayFrameMs;
            MinFeatureId = UnknownValue;
            MaxFeatureId = UnknownValue;
            NumSig = 0;
        }


        // Constructor used when loading objects from the datastore
        public ProcessBlockModel(int blockId, List<string> settings) : base(blockId)
        {
            if (settings != null)
                LoadSettings(settings);
            NumSig = 0;
        }


        public void AddFeature(ProcessFeatureModel featureToAdd)
        {
            if (MinFeatureId == UnknownValue || featureToAdd.FeatureId < MinFeatureId)
                MinFeatureId = featureToAdd.FeatureId;
            if (MaxFeatureId == UnknownValue || featureToAdd.FeatureId > MaxFeatureId)
                MaxFeatureId = featureToAdd.FeatureId;
        }
        public void AddFeatureList(ProcessFeatureList featuresToAdd)
        {
            if (featuresToAdd != null)
            {
                var count = featuresToAdd.Count;
                if (count > 0)
                {
                    MinFeatureId = featuresToAdd.Keys[0];
                    MaxFeatureId = featuresToAdd.Keys[count - 1];
                }
            }
        }


        public void AssertGood()
        {
            Assert(TimeMs == UnknownValue || TimeMs > 0, "AssertGood: Logic 1");
            Assert(SumTimeMs == UnknownValue || SumTimeMs >= 0, "AssertGood: Logic 2");
            Assert(LinealM == UnknownValue || LinealM >= 0, "AssertGood: Logic 3");

            // Some drone operators halt their drone to look at intersting objects => SumLinealM == 0
            // if (BlockId > 1)
            //    Assert(SumLinealM == UnknownValue || SumLinealM > 0, "AssertGood: Logic 4");
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int FlightStepIdSetting = FirstFreeSetting;
        public const int LegIdSetting = FirstFreeSetting + 1;
        public const int VelYSetting = FirstFreeSetting + 2;
        public const int InputFrameIdSetting = FirstFreeSetting + 3;
        public const int InputFrameMsSetting = FirstFreeSetting + 4;
        public const int DisplayFrameIdSetting = FirstFreeSetting + 5;
        public const int DisplayFrameMsSetting = FirstFreeSetting + 6;
        public const int MinFeatureIdSetting = FirstFreeSetting + 7;
        public const int MaxFeatureIdSetting = FirstFreeSetting + 8;
        public const int DsmMSetting = FirstFreeSetting + 9;
        public const int DemMSetting = FirstFreeSetting + 10;
        public const int HasLegSetting = FirstFreeSetting + 11;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public override DataPairList GetSettings()
        {
            var answer = base.GetSettings();
            answer[0].Key = "Block";

            answer.Add("Flight Step", FlightStepId);
            answer.Add("Leg Id", (FlightLegId == UnknownValue ? 0 : FlightLegId));
            answer.Add("Input Frame Id", InputFrameId);
            answer.Add("Input Frame Ms", InputFrameMs, MillisecondsNdp);
            answer.Add("Display Frame Id", DisplayFrameId);
            answer.Add("Display Frame Ms", DisplayFrameMs, MillisecondsNdp);
            answer.Add("Min Feat Id", MinFeatureId);
            answer.Add("Max Feat Id", MaxFeatureId);

            return answer;
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public override void LoadSettings(List<string> settings)
        {
            base.LoadSettings(settings);

            int i = FirstFreeSetting - 1;
            FlightStepId = StringToInt(settings[i++]);
            FlightLegId = StringToNonNegInt(settings[i++]);
            InputFrameId = StringToNonNegInt(settings[i++]);
            InputFrameMs = StringToNonNegInt(settings[i++]);
            DisplayFrameId = StringToNonNegInt(settings[i++]);
            DisplayFrameMs = StringToNonNegInt(settings[i++]);
            MinFeatureId = StringToNonNegInt(settings[i++]);
            MaxFeatureId = StringToNonNegInt(settings[i++]);

            if (FlightLegId == 0)
                FlightLegId = UnknownValue;
        }
    };
}
