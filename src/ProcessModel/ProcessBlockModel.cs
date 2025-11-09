// Copyright SkyComb Limited 2025. All rights reserved. 
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

            // Some drone operators halt their drone to look at interesting objects => SumLinealM == 0
            // if (BlockId > 1)
            //    Assert(SumLinealM == UnknownValue || SumLinealM > 0, "AssertGood: Logic 4");
        }


        // One-based settings index values. Must align with GetSettings procedure below     
        public const int FlightStepIdSetting = FirstFreeSetting;
        public const int LegIdSetting = FirstFreeSetting + 1;
        public const int InputFrameIdSetting = FirstFreeSetting + 2;
        public const int InputFrameMsSetting = FirstFreeSetting + 3;
        public const int MinFeatureIdSetting = FirstFreeSetting + 4;
        public const int MaxFeatureIdSetting = FirstFreeSetting + 5;
        public const int MinRawHeatSetting = FirstFreeSetting + 6;
        public const int MaxRawHeatSetting = FirstFreeSetting + 7;
        public const int DsmMSetting = FirstFreeSetting + 8;
        public const int DemMSetting = FirstFreeSetting + 9;
        public const int HasLegSetting = FirstFreeSetting + 10;


        // Get the class's settings as datapairs (e.g. for saving to the datastore). Must align with above index values.
        public override DataPairList GetSettings()
        {
            var answer = base.GetSettings();
            answer[0].Key = "Block";

            answer.Add("Flight Step", FlightStepId);
            answer.AddInt_UnknownIsBlank("Leg Id", FlightLegId);
            answer.Add("Input Frame Id", InputFrameId);
            answer.Add("Input Frame Ms", InputFrameMs);
            answer.AddInt_UnknownIsBlank("Min Feat Id", MinFeatureId);
            answer.AddInt_UnknownIsBlank("Max Feat Id", MaxFeatureId);

            return answer;
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public override void LoadSettings(List<string> settings)
        {
            base.LoadSettings(settings);

            int i = FirstFreeSetting - 1;
            FlightStepId = StringToInt(settings[i++]);
            FlightLegId = StringToInt_BlankIsUnknown(settings[i++]);
            InputFrameId = StringToNonNegInt(settings[i++]);
            InputFrameMs = StringToNonNegInt(settings[i++]);
            MinFeatureId = StringToInt_BlankIsUnknown(settings[i++]);
            MaxFeatureId = StringToInt_BlankIsUnknown(settings[i++]);

            if (FlightLegId == 0)
                FlightLegId = UnknownValue;
        }


        // Unit test to ensure that GetSettings and LoadSettings form a consistent pair.
        public static void TestSettingsPair()
        {
            var rand = new Random();
            var obj = new ProcessBlockModel( new ProcessScopeModel() )
            {
                FlightStepId = rand.Next(1, 10000),
                FlightLegId = rand.Next(1, 100),
                InputFrameId = rand.Next(0, 10000),
                InputFrameMs = rand.Next(0, 1000000),
                MinFeatureId = rand.Next(0, 10000),
                MaxFeatureId = rand.Next(0, 10000),
                NumSig = rand.Next(0, 10000),
            };

            // Save settings to list
            var settings = obj.GetSettings().Select(dp => dp.Value.ToString()).ToList();
            // Create a new object and load settings
            var obj2 = new ProcessBlockModel(1234, settings);
            // Compare all relevant properties
            Assert(obj.FlightStepId == obj2.FlightStepId, "FlightStepId mismatch");
            Assert(obj.FlightLegId == obj2.FlightLegId, "FlightLegId mismatch");
            Assert(obj.InputFrameMs == obj2.InputFrameMs, "InputFrameMs mismatch");
            Assert(obj.InputFrameId == obj2.InputFrameId, "InputFrameId mismatch");
            Assert(obj.MinFeatureId == obj2.MinFeatureId, "MinFeatureId mismatch");
            Assert(obj.MaxFeatureId == obj2.MaxFeatureId, "MaxFeatureId mismatch");
        }
    };
}
