// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;


// Continuation of RunVideo.cs, contains both Skycomb-specific runners
namespace SkyCombImage.RunSpace
{
    // Given the Config, and the corresponding file(s) found on disk,
    // create the appropriate RunVideo object.
    public class VideoRunnerFactory
    {
        public static RunVideo Create(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone)
        {
            try
            {
                var directory = config.OutputElseInputDirectory();

                CombLoad dataReader = new(dataStore);
                var runSettings = dataReader.RunConfigSettings();

                // Are we processing a video/flight log for the first time?
                bool firstTime = (runSettings == null);
                if (firstTime)
                {
                    // If input is an optical video then default image processing model to none.
                    if (drone.HasInputVideo && !drone.InputVideo.Thermal)
                        config.RunProcess = RunProcessEnum.None;
                }
                else
                    config.LoadSettings(runSettings);

                var theRunModel = config.RunProcess;


                if (firstTime)
                    // If we are not processing the video then (by default) dont create a video output .
                    if (theRunModel == RunProcessEnum.None)
                        config.ProcessConfig.SaveAnnotatedVideo = false;

                // Create the appropriate VideoRunner object
                RunVideo answer = null; 
                switch (theRunModel)
                {
                    case RunProcessEnum.Flow:
                        answer = new RunVideoFlow(parent, config, dataStore, drone);
                        break;
                    case RunProcessEnum.Comb:
                        answer = new RunVideoCombDrone(parent, config, dataStore, drone);
                        answer.ProcessDrawScope.Process = answer.ProcessAll as CombProcessAll;
                        break;
                    default:
                        answer = new RunVideoStandard(parent, config, dataStore, drone);
                        break;
                };

                answer.LoadDataStoreSettings();

                // Reload the Category and Object Category data (if any) from the Datastore.
                // This data is independent of the process run range. We retain all this data
                // even if user has reduced the processing leg range this run - as next run
                // they may increase the leg range again - and we want any past object
                // annotations to be available to them.
                answer.LoadCategoryAll();

                return answer;
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("VideoRunnerFactory.Create", ex);
            }
        }
    }

}