// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombGround.CommonSpace;
using SkyCombImageLibrary.RunSpace;


// Continuation of RunVideo.cs, contains both Skycomb-specific runners
namespace SkyCombImage.RunSpace
{
    // Given the Config, and the corresponding file(s) found on disk,
    // create the appropriate RunVideo object.
    public class VideoRunnerFactory
    {

        // Create the appropriate VideoRunner object
        public static RunVideo CreateRunVideo(RunUserInterface parent, RunConfig runConfig, DroneDataStore dataStore, Drone drone, DroneIntervalList intervals, ObservationHandler<ProcessAll> processHook)
        {
            RunVideo? answer = null;

            switch (runConfig.RunProcess)
            {
                case RunProcessEnum.Yolo:
                    var yoloRunner = new RunVideoYoloDrone(parent, runConfig, dataStore, drone);
                    yoloRunner.ProcessDrawScope.Process = yoloRunner.ProcessAll;
                    yoloRunner.YoloProcess.Observation += processHook;
                    answer = yoloRunner;
                    break;
                case RunProcessEnum.Comb:
                    var combRunner = new RunVideoCombDrone(parent, runConfig, dataStore, drone);
                    combRunner.ProcessDrawScope.Process = combRunner.ProcessAll;
                    combRunner.CombProcess.Observation += processHook;
                    answer = combRunner;
                    break;
                default:
                    answer = new RunVideoStandard(parent, runConfig, dataStore, drone);
                    break;
            }
            answer.RunIntervals = intervals; 

            answer.LoadDataStoreConfigSettings();

            // Reload the Category and Object Category data (if any) from the Datastore.
            // This data is independent of the process run range. We retain all this data
            // even if user has reduced the processing leg range this run - as next run
            // they may increase the leg range again - and we want any past object
            // annotations to be available to them.
            answer.LoadCategoryAll();

            answer.ConfigureModelScope();

            return answer;
        }


        public static RunVideo Create(RunUserInterface parent, RunConfig runConfig, DroneDataStore dataStore, Drone drone, DroneIntervalList intervals, ObservationHandler<ProcessAll> processHook)
        {
            try
            {
                var directory = runConfig.OutputElseInputDirectory;

                StandardLoad dataReader = new(dataStore);
                dataReader.LoadRunConfigSettings(runConfig);
                var runSettings = dataReader.RunConfigSettings();

                // Are we processing a video/flight log for the first time?
                bool firstTime = (runSettings == null);
                if (firstTime)
                {
                    // If input is an optical video then default image processing model to none.
                    if (drone.HasInputVideo && !drone.InputVideo.Thermal)
                        runConfig.RunProcess = RunProcessEnum.None;
                }
                else
                    runConfig.LoadSettings(runSettings);

                var theRunModel = runConfig.RunProcess;


                if (firstTime)
                    // If we are not processing the video then (by default) dont create a video output .
                    if (theRunModel == RunProcessEnum.None)
                        runConfig.ProcessConfig.SaveAnnotatedVideo = false;

                return CreateRunVideo(parent, runConfig, dataStore, drone, intervals, processHook);
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("VideoRunnerFactory.Create", ex);
            }
        }
    }

}