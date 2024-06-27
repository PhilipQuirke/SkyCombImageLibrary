// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombImage.ProcessLogic;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DrawSpace;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.DrawSpace
{
    // Code to draw images related to drone & process data in charts, graphs,etc.
    public class ProcessDrawScope : DroneDrawScope  
    {
        // The Video, Drone & model scope of a processing run
        public ProcessScope ProcessScope;
        // The actual processing object (if any)
        public CombProcess Process;


        // First millisecond of flight data drawn. Used on graphs with a time axis
        public override int FirstDrawMs { get { return ProcessScope.MinSumTimeMs; } }
        // Last millisecond of flight data drawn. Used on graphs with a time axis
        public override int LastDrawMs { get { return ProcessScope.MaxSumTimeMs; } }


        // First step of flight data to draw. A StepId axis approximates a time axis.
        public override int FirstDrawStepId { get { return ProcessScope.MinStepId; } }
        // Last step of flight data to draw. A StepId axis approximates a time axis.
        public override int LastDrawStepId { get { return ProcessScope.MaxStepId; } }


        // First step of flight data to draw. A StepId axis approximates a time axis.
        public override int FirstRunStepId { get { return ProcessScope.FirstRunStepId; } }
        // Last step of flight data to draw. A StepId axis approximates a time axis.
        public override int LastRunStepId { get { return ProcessScope.LastRunStepId; } }


        public override FlightStep? CurrRunFlightStep { get { return ProcessScope.CurrRunFlightStep; } }
        public override int CurrRunStepId { get { return ProcessScope.PSM.CurrRunStepId; } }


        public override float FloorMinSumLinealM { get { return ProcessScope.FloorMinSumLinealM; } }
        public override float CeilingMaxSumLinealM { get { return ProcessScope.CeilingMaxSumLinealM; } }
        public override DataPairList GetSettings_FlightPath { get { return ProcessScope.GetSettings_FlightPath(); } }


        public override float FloorMinPitchDeg { get { return ProcessScope.FloorMinPitchDeg; } }
        public override float CeilingMaxPitchDeg { get { return ProcessScope.CeilingMaxPitchDeg; } }
        public override string DescribePitch { get { return ProcessScope.DescribePitch(Drone.DroneConfig); } }
        public override DataPairList GetSettings_Pitch { get { return ProcessScope.GetSettings_Pitch(); } }


        public override float FloorMinDeltaYawDeg { get { return ProcessScope.FloorMinDeltaYawDeg; } }
        public override float CeilingMaxDeltaYawDeg { get { return ProcessScope.CeilingMaxDeltaYawDeg; } }
        public override string DescribeDeltaYaw { get { return ProcessScope.DescribeDeltaYaw(Drone.DroneConfig); } }
        public override DataPairList GetSettings_DeltaYaw { get { return ProcessScope.GetSettings_DeltaYaw(); } }


        public override float FloorMinRollDeg { get { return ProcessScope.FloorMinRollDeg; } }
        public override float CeilingMaxRollDeg { get { return ProcessScope.CeilingMaxRollDeg; } }
        public override string DescribeRoll { get { return ProcessScope.DescribeRoll(Drone.DroneConfig); } }
        public override DataPairList GetSettings_Roll { get { return ProcessScope.GetSettings_Roll(); } }


        public override (float, float) MinMaxVerticalAxisM { get { return ProcessScope.MinMaxVerticalAxisM; } }
        public override string DescribeElevation { get { return ProcessScope.DescribeElevation; } }
        public override DataPairList GetSettings_Altitude { get { return ProcessScope.GetSettings_Altitude(); } }


        public override string DescribeSpeed { get { return ProcessScope.DescribeSpeed; } }
        public override DataPairList GetSettings_Speed { get { return ProcessScope.GetSettings_Speed(); } }
        public override float MaxSpeedMps { get { return ProcessScope.MaxSpeedMps; } }


        public ProcessDrawScope(CombProcess process, ProcessScope processScope, Drone drone) : base (drone)
        {
            Process = process;
            ProcessScope = processScope;

            Reset(processScope, drone);
        }


        public virtual void Reset(ProcessScope processScope, Drone drone)
        {
            ProcessScope = processScope;
            Drone = drone;

            Assert(ProcessScope != null, "Reset: Missing scope");

            // We need to draw all the steps in the flight
            ProcessScope.TardisMaxKey = ProcessScope.LastRunStepId;
            if(Drone != null)
                ProcessScope.TardisMaxKey = Drone.FlightSteps.TardisMaxKey;


            // Constrain drawing to the process scope.
            TardisSummary = ProcessScope;
        }


        // Sometimes the CurrRunFlightStep can go one step outside of range
        // and the "out of range" step may have "out of range" attribute values,
        // triggering the Asserts in DroneDatumToHeight.
        public override bool CurrRunFlightStepValid()
        {
            return
                (ProcessScope.CurrRunFlightStep != null) &&
                (ProcessScope.CurrRunFlightStep.StepId >= ProcessScope.FirstRunStepId) &&
                (ProcessScope.CurrRunFlightStep.StepId <= ProcessScope.LastRunStepId);
        }


        // Always draw the first/last frames, first/last frames processed
        public override bool DrawStepId(int thisStepId)
        {
            return
                thisStepId == FirstDrawStepId ||
                thisStepId == LastDrawStepId ||
                thisStepId == ProcessScope.FirstRunStepId ||
                thisStepId == ProcessScope.LastRunStepId;
        }
    }


    // Code to draw images related to process object data in charts, graphs,etc.
    public class ObjectDrawScope : ProcessDrawScope // : DroneDrawScope : TardisDrawScope
    {
        // First millisecond object visible
        public int FirstObjectMs;
        // Last millisecond object visible
        public int LastObjectMs;

        // Optional filters on the objects to draw
        public int MinHeightM;
        public int MaxHeightM;
        public int MinSizeCM2;
        public int MaxSizeCM2;
        public int MinHeat;
        public int MaxHeat;
        public int MinRangeM;
        public int MaxRangeM;

        public int NumObjects;
        public int NumFilteredObjects;


        public override int FirstDrawMs { get { return FirstObjectMs; } }
        public override int LastDrawMs { get { return LastObjectMs; } }


        public ObjectDrawScope(CombProcess process, ProcessScope scope, Drone drone) : base(process, scope, drone)
        {
            Reset(scope, drone);
        }


        public override void Reset(ProcessScope scope, Drone drone)
        {
            base.Reset(scope, drone);
            ResetMemberData();
        }


        private void ResetMemberData()
        {
            FirstObjectMs = UnknownValue;
            LastObjectMs = UnknownValue;
            MinHeightM = ProcessObjectModel.UnknownHeight;
            MaxHeightM = ProcessObjectModel.UnknownHeight;
            MinSizeCM2 = UnknownValue;
            MaxSizeCM2 = UnknownValue;
            MinHeat = UnknownValue;
            MaxHeat = UnknownValue;
            MinRangeM = UnknownValue;
            MaxRangeM = UnknownValue;

            NumObjects = 0;
            NumFilteredObjects = NumObjects;
        }


        public void SetObjectRange(CombObjList objList)
        {
            if (objList == null)
                ResetMemberData();
            else
            {
                MinHeightM = (int)Math.Floor(objList.MinHeightM); // If we have negative heights show them
                MaxHeightM = (int)Math.Ceiling(objList.MaxHeightM);
                MinSizeCM2 = (int)Math.Floor(objList.MinSizeCM2);
                MaxSizeCM2 = (int)Math.Ceiling(objList.MaxSizeCM2);
                MinHeat = objList.MinHeat;
                MaxHeat = objList.MaxHeat;
                MinRangeM = objList.MinRangeM;
                MaxRangeM = objList.MaxRangeM;

                NumObjects = objList.Count;
                NumFilteredObjects = NumObjects;
            }
        }
    }
}

