// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DroneLogic;
using SkyCombDrone.PersistModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;


// Continuation of RunWorker.cs, contains both Skycomb-specific runners
namespace SkyCombImage.RunSpace
{
    // Given the Config, and the corresponding file(s) found on disk,
    // create the appropriate RunWorker object.
    public class RunWorkerFactory
    {

        // Create the appropriate RunWorker object
        public static RunWorker CreateRunWorker(RunUserInterface parent, RunConfig runConfig, DroneDataStore dataStore, Drone drone, DroneIntervalList? intervals, ObservationHandler<ProcessAll>? processHook)
        {
            RunWorker? answer = null;

            switch (runConfig.RunProcess)
            {
                case RunProcessEnum.Yolo:
                    var yoloRunner = new RunWorkerYoloDrone(parent, runConfig, dataStore, drone);
                    yoloRunner.ProcessDrawScope.Process = yoloRunner.ProcessAll;
                    if (processHook != null)
                        yoloRunner.YoloProcess.Observation += processHook;
                    answer = yoloRunner;
                    break;
                case RunProcessEnum.Comb:
                    var combRunner = new RunWorkerCombDrone(parent, runConfig, dataStore, drone);
                    combRunner.ProcessDrawScope.Process = combRunner.ProcessAll;
                    if (processHook != null)
                        combRunner.CombProcess.Observation += processHook;
                    answer = combRunner;
                    break;
                default:
                    answer = new RunWorkerStandard(parent, runConfig, dataStore, drone);
                    break;
            }
            answer.SizeImages = parent.GetSizeImages();
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


        public static RunWorker Create(RunUserInterface parent, RunConfig runConfig, DroneDataStore dataStore, Drone drone, DroneIntervalList intervals, ObservationHandler<ProcessAll>? processHook)
        {
            try
            {
                var directory = runConfig.OutputElseInputDirectory;

                StandardLoad dataReader = new(dataStore);
                dataReader.LoadRunConfigSettings(runConfig);
                var runSettings = dataReader.RunConfigSettings();

                // Are we processing a video/flight log for the first time?
                bool firstTime = (runSettings == null);
                if (!firstTime)
                    runConfig.LoadSettings(runSettings);

                var theRunModel = runConfig.RunProcess;

                if (firstTime)
                    // If we are not processing the video then (by default)
                    // or we dont have a video then dont create a video output
                    if ((theRunModel == RunProcessEnum.None) || (runConfig.InputIsImages))
                        runConfig.ProcessConfig.SaveAnnotatedVideo = false;

                return CreateRunWorker(parent, runConfig, dataStore, drone, intervals, processHook);
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("VideoRunnerFactory.Create", ex);
            }
        }
    }

}