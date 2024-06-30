using SkyCombDrone.PersistModel;


namespace SkyCombImage.PersistModel
{
    // Load run and model data about a previous processing run from a datastore
    public class StandardLoad : DataStoreAccessor
    {
        public StandardLoad(DroneDataStore data) : base(data)
        {
        }


        // Load model config data from the datastore
        public List<string> ModelConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, ProcessConfigTitle, ModelTitleRow, LhsColOffset);
        }


        // Load run config data from the datastore 
        public List<string> RunConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, RunConfigTitle, RunTitleRow, MidColOffset);
        }


        // Load draw config data from the datastore
        public List<string> DrawConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, DrawTitle, DrawTitleRow, MidColOffset);
        }


        // Load "save" config data from the datastore
        public List<string> OutputConfigSettings()
        {
            return Data.GetColumnSettingsIfAvailable(ProcessTabName, OutputConfigTitle, OutputTitleRow, MidColOffset);
        }
    }
}