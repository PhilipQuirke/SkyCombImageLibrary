using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.PersistModel
{
    // Load run and model data about a previous processing run from a datastore
    public class BlockLoad : DataStoreAccessor
    {
        public BlockLoad(DroneDataStore data) : base(data)
        {
        }


        // Load all Block Objects from the datastore
        public void BlockObjects(CombProcessAll model, Drone drone)
        {
            int row = 2;
            try
            {
                if (Data.SelectWorksheet(Blocks1TabName))
                {
                    var cell = Data.Worksheet.Cells[row, TardisModel.TardisIdSetting];
                    while (cell != null && cell.Value != null && cell.Value.ToString() != "")
                    {
                        var blockIdString = cell.Value.ToString();
                        if (blockIdString == "")
                            break;
                        var blockId = ConfigBase.StringToNonNegInt(blockIdString);

                        // Load the non-blank cells in this row into a CombObject
                        model.Blocks.AddBlock(ProcessFactory.NewBlock(blockId, Data.GetRowSettings(row, 1), drone), null, drone);

                        row++;
                        cell = Data.Worksheet.Cells[row, TardisModel.TardisIdSetting];
                    }
                }
            }
            catch (Exception ex)
            {
                // Suppress the error and any objects loaded
                System.Diagnostics.Debug.WriteLine("Suppressed CombLoad.CombBlocks failure: " + ex.ToString());
                model.Blocks.Clear();
            }
        }
    }
}