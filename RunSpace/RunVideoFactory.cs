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
        public static RunVideo Create(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone, ObservationHandler<ProcessAll> processHook)
        {
            try
            {
                var directory = config.OutputElseInputDirectory();

                StandardLoad dataReader = new(dataStore);
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
                RunVideo? answer = null; 
                switch (theRunModel)
                {
                    case RunProcessEnum.Yolo:
                        var yoloRunner = new RunVideoYolo(parent, config, dataStore, drone);
                        yoloRunner.ProcessDrawScope.Process = yoloRunner.ProcessAll;
                        yoloRunner.YoloProcess.Observation += processHook;
                        answer = yoloRunner;
                        break;
                    case RunProcessEnum.Comb:
                        var combRunner = new RunVideoCombDrone(parent, config, dataStore, drone);
                        combRunner.ProcessDrawScope.Process = combRunner.ProcessAll;
                        combRunner.CombProcess.Observation += processHook;
                        answer = combRunner;
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

        private static void CombProcess_Observation(ProcessAll sender, ProcessEventEnum processEvent, EventArgs e)
        {
            throw new NotImplementedException();
        }
    }

}