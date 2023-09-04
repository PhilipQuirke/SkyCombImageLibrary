// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;


// Models are used in-memory and to persist/load data to/from the datastore
namespace SkyCombImage.ProcessModel
{
    // FlightLeg contains the best estimate of a drone flight leg from drone data.
    // CombLeg analyses CombObjects in that FlightLeg to refine/correct
    // the flight altitude data using FlightStep.FixAltM.
    public class CombLegModel : TardisSummaryModel
    {
        // DRONE FLIGHT DATA
        // Details of the associated FlightLeg
        public int LegId { get; set; } = UnknownValue;
        public string LegName { get { return LegIdToName(LegId); } }

        // Id of FlightStep of first block of process summarised
        public int MinStepId { get; set; } = UnknownValue;
        // Id of FlightStep of last block of process summarised
        public int MaxStepId { get; set; } = UnknownValue;


        // DATA USED TO CALCULATE BestFixAltM
        // The original (before) error values (with FlightLeg.FixAltM set to 0)
        protected float OrgSumLocnErrM { get; set; } = UnknownValue;
        protected float OrgSumHeightErrM { get; set; } = UnknownValue;
        // The original (after) error value with best altitude fix 
        protected float BestFixAltM { get; set; } = 0;
        protected float BestSumLocnErrM { get; set; } = UnknownValue;
        protected float BestSumHeightErrM { get; set; } = UnknownValue;


        // IMAGE PROCESSING DATA
        // Number of significant objects in the leg
        public int NumSignificantObjects { get; set; } = UnknownValue;
        // First block of process summarised
        public int MinBlockId { get { return MinTardisId; } }
        // Last block of process summarised
        public int MaxBlockId { get { return MaxTardisId; } }


        // Constructor used when loaded objects from the datastore
        public CombLegModel(List<string>? settings = null) : base("Block")
        {
            if (settings != null)
                LoadSettings(settings);
        }


        // This class does not own any FlightSteps or FlightSections
        public override int GetTardisMaxKey() { return 0; }
        public override TardisModel? GetTardisModel(int index) { return null; }


        // One-based settings index values. Must align with GetSettings procedure below
        public const int LegIdSetting = 1;
        public const int LegNameSetting = 2;
        public const int NumSigObjsSetting = 3;
        public const int OrgSumLocnErrMSetting = 4;
        public const int OrgObjLocnErrMSetting = 5;
        public const int OrgSumHeightErrMSetting = 6;
        public const int OrgObjHeightErrMSetting = 7;
        public const int BestFixAltMSetting = 8;
        public const int BestSumLocnErrMSetting = 9;
        public const int BestObjLocnErrMSetting = 10;
        public const int BestSumHeightErrMSetting = 11;
        public const int BestObjHeightErrMSetting = 12;
        public const int MinStepIdSetting = 13;
        public const int MaxStepIdSetting = 14;
        public const int NumBlocksSetting = 15;


        // Get the class's settings as datapairs (e.g. for saving to the datastore)
        public override DataPairList GetSettings()
        {
            var answer = new DataPairList
            {
                { "Leg Id", LegId },
                { "Name", LegName },
                { "Num Sig Objs", NumSignificantObjects },
                { "Org Sum Locn Err M", OrgSumLocnErrM, LocationNdp },
                { "Org Avg Locn Err M", (NumSignificantObjects > 0 ? OrgSumLocnErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Org Sum Ht Err M", OrgSumHeightErrM, LocationNdp},
                { "Org Avg Ht Err M", (NumSignificantObjects > 0 ? OrgSumHeightErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Bst Fix Alt M", BestFixAltM, HeightNdp},
                { "Bst Sum Locn Err M", BestSumLocnErrM, LocationNdp },
                { "Bst Avg Locn Err M", (NumSignificantObjects > 0 ? BestSumLocnErrM / NumSignificantObjects : UnknownValue), LocationNdp },
                { "Bst Sum Ht Err M", BestSumHeightErrM, LocationNdp },
                { "Bst Avg Ht Err M", (NumSignificantObjects > 0 ? BestSumHeightErrM / NumSignificantObjects : UnknownValue), LocationNdp },
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
            LegId = StringToInt(settings[i++]);
            i++; // Skip LegName  
            NumSignificantObjects = StringToInt(settings[i++]);
            OrgSumLocnErrM = StringToFloat(settings[i++]);
            i++; // OrgObjLocnErrM  
            OrgSumHeightErrM = StringToFloat(settings[i++]);
            i++; // OrgObjHeightErrM  
            BestFixAltM = StringToFloat(settings[i++]);
            BestSumLocnErrM = StringToFloat(settings[i++]);
            i++; // BestObjLocnErrM  
            BestSumHeightErrM = StringToFloat(settings[i++]);
            i++; // BestObjHeightErrM       
            MinStepId = StringToInt(settings[i++]);
            MaxStepId = StringToInt(settings[i++]);
            i++; // #Blocks

            LoadSettingsOffset(settings, i);
        }
    }
}
