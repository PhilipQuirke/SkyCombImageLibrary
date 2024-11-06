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
            : base(data, Blocks1TabName, Blocks2TabName)
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

                Data.SelectOrAddWorksheet(Blocks1TabName);
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

                Data.SetLastUpdateDateTime(Blocks1TabName);
            }
        }


        // Add a graph of the drone & ground elevations 
        public void AddElevationsGraph()
        {
            (var _, var lastRow) = Data.PrepareChartArea(GraphTabName, "BlocksElevations", TardisTabName);
            if ((lastRow > 0) && (MaxDatumId > 0) && (Summary != null) && (Summary.TardisMaxKey > 1))
            {
                var FirstGraphRow = 0;

                // Generate a bitmap of the DSM land overlaid with the drone path 
                var drawScope = new DroneDrawScope(Drone);
                var drawAltitudes = new DrawElevations(drawScope);
                drawAltitudes.Initialise(new Size(1600, 300));
                var graphBitmap = drawAltitudes.CurrBitmap();

                Data.SaveBitmap(graphBitmap, "BlocksElevations", FirstGraphRow, 0);

                Data.SetTitleAndDataListColumn("Metrics", FirstGraphRow + 1, ChartWidth + 1, Summary.GetSettings_Altitude(), true, 1);
            }
        }


        // Add a drone travel distance graph  
        public void AddTravelDistGraph()
        {
            AddTravelDistGraph(
                1,
                "BlocksTravel",
                "Travel distance (in lineal cms) vs Block",
                Summary.GetSettings_Lineal());
        }


        // Add a graph of the drone speed as per smoothed Steps data 
        public void AddSpeedGraph()
        {
            AddSpeedGraph(
                2,
                "BlocksSpeed",
                "Smoothed drone flight speed (in Mps) vs Block",
                Summary.GetSettings_Speed());
        }


        // Add a delta yaw graph 
        public void AddDeltaYawGraph()
        {
            AddDeltaYawGraph(
                3,
                "BlocksDeltaYaw",
                "Drone Delta Yaw (in degrees / block) vs Block",
                Summary.GetSettings_DeltaYaw());
        }


        // Add a pitch graph  
        public void AddPitchGraph()
        {
            AddPitchGraph(
                4,
                "BlocksPitch",
                "Drone Pitch (in degrees) vs Block",
                Summary.GetSettings_Pitch());
        }


        // Add a roll graph  
        public void AddRollGraph()
        {
            AddRollGraph(
                5,
                "BlocksRoll",
                "Drone Roll (in degrees) vs Block",
                Summary.GetSettings_Roll());
        }


        // Add a graph of whether the drone step is part of a leg or not
        public void AddLegGraph()
        {
            AddLegGraph(
                6,
                "BlocksLegs",
                "Block is part of a flight Leg",
                ProcessBlockModel.HasLegSetting);
        }


        // Save the list of Block charts
        public void AddBlocks2Tab(FlightStepSummaryModel summary)
        {
            Summary = summary;

            // Show the min/max BlockId (rounded down/up to nearest 50) value on all the graphs x axis
            MinDatumId = (int)(Math.Floor(MinDatumId / 50.0) * 50.0);
            MaxDatumId = (int)(Math.Ceiling(MaxDatumId / 50.0) * 50.0);

            // Add the charts
            Data.SelectOrAddWorksheet(Blocks2TabName);
            Data.Worksheet.Drawings.Clear();
            if (MaxDatumId > 0)
            {
                AddElevationsGraph();
                AddTravelDistGraph();
                AddSpeedGraph();
                AddDeltaYawGraph();
                AddPitchGraph();
                AddRollGraph();
                AddLegGraph();

                Data.SetLastUpdateDateTime(Blocks2TabName);
            }
        }
    }
}