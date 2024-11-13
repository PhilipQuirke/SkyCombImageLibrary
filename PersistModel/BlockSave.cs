using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.PersistModel
{
    // Save Block data to a datastore
    public class BlockSave : TardisSaveGraph
    {
        Drone Drone { get; }

        FlightStepSummaryModel? Summary = null;


        public BlockSave(Drone drone, DroneDataStore data)
            : base(data, BlockDataTabName, "")
        {
            Drone = drone;
        }


        // Save the list of Block data (includes some drone data copied from FlightStep)
        public void AddBlockList(ProcessBlockList blocks)
        {
            if (blocks.Count > 0)
            {
                MinDatumId = 1000000;
                MaxDatumId = 1;

                Data.SelectOrAddWorksheet(BlockDataTabName);
                int blockRow = 0;
                foreach (var block in blocks)
                {
                    Data.SetDataListRowKeysAndValues(ref blockRow, block.Value.GetSettings());
                    MinDatumId = Math.Min(MinDatumId, block.Value.BlockId);
                    MaxDatumId = Math.Max(MaxDatumId, block.Value.BlockId);
                }

                Data.SetNumberColumnNdp(TardisModel.YawDegSetting, DegreesNdp);
                Data.SetNumberColumnNdp(TardisModel.DeltaYawDegSetting, DegreesNdp);
                Data.SetNumberColumnNdp(TardisModel.PitchDegSetting, DegreesNdp);
                Data.SetNumberColumnNdp(TardisModel.RollDegSetting, DegreesNdp);

                Data.SetColumnColor(TardisModel.TardisIdSetting, blockRow, Color.Blue);
                Data.SetColumnColor(TardisModel.AltitudeMSetting, blockRow, Color.Blue);
                Data.SetColumnColor(ProcessBlockModel.DsmMSetting, blockRow, Color.Blue);
                Data.SetColumnColor(ProcessBlockModel.DemMSetting, blockRow, Color.Blue);

                // Highlight in red any blocks where the DeltaYawDeg exceeds FlightConfig.MaxLegStepDeltaYawDeg. This implies not part of a leg.
                Data.AddConditionalRuleBad(TardisModel.DeltaYawDegSetting, blockRow, Drone.DroneConfig.MaxLegStepDeltaYawDeg);
                // Highlight in red any blocks where the PitchDeg exceeds FlightConfig.MaxLegStepPitchDeg. This implies not part of a leg.
                Data.AddConditionalRuleBad(TardisModel.PitchDegSetting, blockRow, Drone.DroneConfig.MaxLegStepPitchDeg);
            }
        }
    }
}