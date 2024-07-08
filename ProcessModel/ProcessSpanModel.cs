// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // ProcessSpan relates to either the steps in a FlightLeg OR a sequence of FlightSteps (not related to a FlightLeg).
    // Either way, ProcessSpan analyses ProcessObjects to refine/correct the flight altitude data using FlightStep.FixAltM.
    public class ProcessSpanModel : TardisSummaryModel
    {
        // Unique identifier of this ProcessSpan. If UseLeg there is a 1-1 correspondence to FlightLeg. Otherwise they are unrelated.
        public int ProcessSpanId { get; set; } = UnknownValue;
        public string Name { get { return IdToLetter(ProcessSpanId); } }

        // Id of first FlightStep in this span
        public int MinStepId { get; set; } = UnknownValue;
        // Id of last FlightStep in this span
        public int MaxStepId { get; set; } = UnknownValue;


        // DATA USED TO CALCULATE BestFixAltM
        // The original (after) error value with best altitude fix 
        public float BestFixAltM { get; set; } = 0;
        protected float BestSumLocnErrM { get; set; } = UnknownValue;
        protected float BestSumHeightErrM { get; set; } = UnknownValue;
        // The original (before) error values (with FlightLeg.FixAltM set to 0)
        protected float OrgSumLocnErrM { get; set; } = UnknownValue;
        protected float OrgSumHeightErrM { get; set; } = UnknownValue;



        // IMAGE PROCESSING DATA
        // Number of significant objects in the leg
        public int NumSignificantObjects { get; set; } = UnknownValue;
        // First block of process summarised
        public int MinBlockId { get { return MinTardisId; } }
        // Last block of process summarised
        public int MaxBlockId { get { return MaxTardisId; } }


        // Constructor used when loaded objects from the datastore
        public ProcessSpanModel(List<string>? settings = null) : base("Block")
        {
            if (settings != null)
                LoadSettings(settings);
        }


        protected void ResetBest()
        {
            BestFixAltM = 0;
            BestSumLocnErrM = 9999;
            BestSumHeightErrM = 9999;
            OrgSumLocnErrM = 9999;
            OrgSumHeightErrM = 9999;
        }


        protected void SetBest(float fixAltM, ProcessObjList objs)
        {
            BestFixAltM = fixAltM;
            BestSumLocnErrM = objs.SumLocationErrM;
            BestSumHeightErrM = objs.SumHeightErrM;
        }


        // One-based settings index values. Must align with GetSettings procedure below
        public const int SpanIdSetting = 1;
        public const int SpanNameSetting = 2;
        public const int NumSigObjsSetting = 3;
        public const int BestFixAltMSetting = 4;
        public const int BestSumLocnErrMSetting = 5;
        public const int BestObjLocnErrMSetting = 6;
        public const int BestSumHeightErrMSetting = 7;
        public const int BestObjHeightErrMSetting = 8;
        public const int OrgSumLocnErrMSetting = 9;
        public const int OrgObjLocnErrMSetting = 10;
        public const int OrgSumHeightErrMSetting = 11;
        public const int OrgObjHeightErrMSetting = 12;
        public const int MinStepIdSetting = 13;
        public const int MaxStepIdSetting = 14;
        public const int NumBlocksSetting = 15;


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        public override DataPairList GetSettings()
        {
            var answer = new DataPairList
            {
                { "Process Leg Id", ProcessSpanId },
                { "Name", Name },
                { "Num Sig Objs", NumSignificantObjects },
                { "Bst Fix Alt M", BestFixAltM, HeightNdp},
                { "Bst Sum Locn Err M", BestSumLocnErrM, LocationNdp },
                { "Bst Avg Locn Err M", (NumSignificantObjects > 0 ? BestSumLocnErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Bst Sum Ht Err M", BestSumHeightErrM, LocationNdp },
                { "Bst Avg Ht Err M", (NumSignificantObjects > 0 ? BestSumHeightErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Org Sum Locn Err M", OrgSumLocnErrM, LocationNdp },
                { "Org Avg Locn Err M", (NumSignificantObjects > 0 ? OrgSumLocnErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Org Sum Ht Err M", OrgSumHeightErrM, LocationNdp},
                { "Org Avg Ht Err M", (NumSignificantObjects > 0 ? OrgSumHeightErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Min Step Id", MinStepId },
                { "Max Step Id", MaxStepId },
                { "# Blocks", MaxBlockId - MinBlockId + 1 },
            };

            answer.AddRange(base.GetSettings());

            return answer;
        }


        // Load this object's settings from strings (loaded from a datastore)
        // This function must align to the above GetSettings function.
        public override void LoadSettings(List<string> settings)
        {
            int i = 0;
            ProcessSpanId = StringToInt(settings[i++]);
            i++; // Skip LegName  
            NumSignificantObjects = StringToInt(settings[i++]);
            BestFixAltM = StringToFloat(settings[i++]);
            BestSumLocnErrM = StringToFloat(settings[i++]);
            i++; // BestObjLocnErrM  
            BestSumHeightErrM = StringToFloat(settings[i++]);
            i++; // BestObjHeightErrM       
            OrgSumLocnErrM = StringToFloat(settings[i++]);
            i++; // OrgObjLocnErrM  
            OrgSumHeightErrM = StringToFloat(settings[i++]);
            i++; // OrgObjHeightErrM  
            MinStepId = StringToInt(settings[i++]);
            MaxStepId = StringToInt(settings[i++]);
            i++; // #Blocks

            LoadSettingsOffset(settings, i);
        }
    }
}
