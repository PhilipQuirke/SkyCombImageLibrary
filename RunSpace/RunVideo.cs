// Copyright SkyComb Limited 2024. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DrawSpace;
using SkyCombDrone.PersistModel;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombImage.CategorySpace;
using SkyCombImage.DrawSpace;
using SkyCombImage.PersistModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Diagnostics;


// Namespace for processing of a video made up of multiple images (frames).
// Some classes contain code that persists information between frames
namespace SkyCombImage.RunSpace
{
    // Base video class that does not persist information betweem frames
    abstract public class RunVideo : ProcessScope
    {
        public RunParent RunParent { get; }

        // The configuration data used in processing the video
        public RunConfig RunConfig { get; set; }

        // All drone input data: video (definitely), flight (maybe) and ground (maybe) data 
        // public Drone Drone { get; set; }

        // The data store (spreadsheet) that meta-data is saved to / read from. 
        public DroneDataStore DataStore { get; set; }

        // The processing model apply to to the video. 
        public ProcessAll ProcessAll { get; set; }

        public CategoryAll CategoryAll { get; set; }

        // Basic attributes of the input video
        public VideoModel? VideoBase = null;

        // Current input video frame image
        public Image<Bgr, byte>? CurrInputVideoFrame { get; set; } = null;


        // Has user requested we stop processing?
        public bool StopRunning { get; set; } = false;

        // The process calculation effort (duration) in milliseconds
        int ProcessDurationMs = 0;


        // How to draw various graphs and charts
        public ProcessDrawScope ProcessDrawScope;
        public CombDrawPath CombDrawPath;
        //public DrawCombAltitudeByTime DrawCombAltitudeByTime = null;
        public CombDrawAltitudeByLinealM DrawCombAltitudeByLinealM;
        public DrawSpeed DrawSpeed;
        public DrawPitch DrawPitch;
        public DrawDeltaYaw DrawDeltaYaw;
        public DrawRoll DrawRoll;
        public DrawLeg DrawLeg;


        public RunVideo(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone, ProcessAll processAll) : base(drone)
        {
            RunParent = parent;
            RunConfig = config;
            DataStore = dataStore;
            Drone = drone;
            ProcessAll = processAll;
            VideoBase = null;
            CategoryAll = new();

            CombProcess combProcess = CombProcessIfAny();
            ProcessDrawScope = new(combProcess, this, Drone);

            // What is the maximum scope of objects we draw?
            ProcessObjList? combObjList = null;
            ObjectDrawScope? drawObjectScope = null;
            if (combProcess != null)
            {
                // Only consider objects in the current ProcessScope.
                // This is effectively the intersects of what legs were processed
                // and what legs are currently selected
                combObjList = combProcess.CombObjs.CombObjList.FilterByProcessScope(this);
                if ((combObjList != null) && (combObjList.Count > 0))
                {
                    // Default the filters based on these objects.
                    // This default shows the value range of these objects. 
                    drawObjectScope = new(combProcess, this, Drone);
                    drawObjectScope.SetObjectRange(combObjList);
                }
            }
            CombDrawPath = new(ProcessDrawScope, combObjList, drawObjectScope);

            //DrawCombAltitudeByTime = new(null, DrawScope);
            DrawCombAltitudeByLinealM = new(null, ProcessDrawScope);
            DrawSpeed = new(ProcessDrawScope);
            DrawPitch = new(ProcessDrawScope);
            DrawDeltaYaw = new(ProcessDrawScope);
            DrawRoll = new(ProcessDrawScope);
            DrawLeg = new(ProcessDrawScope);

            if (Drone != null)
                VideoBase = Drone.InputVideo;
        }


        // Do we have access to DroneSpace flight path data?
        public bool HasFlightSteps { get { return Drone != null && Drone.HasFlightSteps; } }
        // Do we have access to DroneSpace speed data?
        public bool HasDroneSpeed { get { return Drone != null && Drone.HasDroneSpeed; } }
        // Do we have access to DroneSpace altitude, ground elevation &/or surface elevation data?
        public bool HasDroneAltitude { get { return Drone != null && Drone.HasDroneAltitude; } }
        public bool HasDronePitch { get { return Drone != null && Drone.HasDronePitch; } }
        public bool HasDroneYaw { get { return Drone != null && Drone.HasDroneYaw; } }
        public bool HasDroneRoll { get { return Drone != null && Drone.HasDroneRoll; } }
        // Do we have DroneSpace flight leg data and user wants to use it
        public bool UseFlightLegs { get { return Drone != null && Drone.UseFlightLegs; } }


        // See if there is an "XXXXXXX_SkyComb.xls" created in a previous run that we can use as a settings cache.
        // This allows the values of RunVideoFromS, RunVideoToS, RunModel, ThresholdValue, etc to be preserved between runs.
        // Can only evaluate this after we know InputVideoFileName (aka the ThermalVideo and/or OpticalVideo names).
        public void LoadDataStoreSettings()
        {
            CombLoad dataReader = new(DataStore);

            var modelSettings = dataReader.ModelConfigSettings();
            if (modelSettings != null)
                RunConfig.ProcessConfig.LoadModelSettings(modelSettings);

            var drawSettings = dataReader.DrawConfigSettings();
            if (drawSettings != null)
                RunConfig.ImageConfig.LoadSettings(drawSettings);

            var outputSettings = dataReader.OutputConfigSettings();
            if (outputSettings != null)
                RunConfig.ProcessConfig.LoadOutputSettings(outputSettings);
        }


        public void LoadCategoryAll()
        {
            CategoryLoad datareader = new(DataStore);
            datareader.LoadAll(CategoryAll);
        }


        // Configure the model scope for the current run. Set videos start frames etc
        public void ConfigureModelScope()
        {
            ConfigureScope_SetFramePos(
                RunConfig.DroneConfig.RunVideoFromS,
                RunConfig.DroneConfig.RunVideoToS);

            StopRunning = false;
        }


        // The input video file name to process.
        public virtual string InputVideoFileName()
        {
            return RunConfig.InputFileName;
        }


        // Load model data from the previous run (if any).
        public virtual void LoadDataStore()
        {
        }


        // Describe the objects found
        public virtual string DescribeSignificantObjects()
        {
            return "";
        }


        // Process start &/or end of drone flight legs.
        public abstract void ProcessFlightLegChange(int prevLegId, int currLegId);


        // Process/analyse a single input video frame 
        public abstract ProcessBlock AddBlockAndProcessInputVideoFrame();


        // Return ProcessModel as CombProcessModel if that is possible
        public CombProcess CombProcessIfAny()
        {
            return (ProcessAll is CombProcess ? ProcessAll as CombProcess : null);
        }


        // Process a single input and (maybe) display video frame for the specified block, returning the modified input&display frames to show 
        public virtual (Image<Bgr, byte>, Image<Bgr, byte>) DrawVideoFrames(ProcessBlockModel block, Image<Bgr, byte> inputFrame, Image<Bgr, byte> displayFrame)
        {
            (var modifiedInputFrame, var modifiedDisplayFrame) = 
                CombDrawVideoFrames.Draw(
                    RunConfig.RunProcess, RunConfig.ProcessConfig, RunConfig.ImageConfig, Drone,
                    block, CombProcessIfAny(), RunConfig.ProcessConfig.FocusObjectId,
                    inputFrame, displayFrame);

            if(modifiedInputFrame != null)
                DrawYawPitchZoom.Draw(ref modifiedInputFrame, Drone, CurrRunFlightStep);

            return (modifiedInputFrame, modifiedDisplayFrame);
        }


         // Return the data to show in the ObjectGrid in the Main Form
        public virtual List<object[]> GetObjectGridData(bool mainForm)
        {
            return null;
        }


        // Do any final activity at the end processing (aka running) of video / flight data
        public virtual void EndRunning()
        {
        }


        // Save just the process settings to the DataStore
        public abstract void SaveProcessSettings();


        // Reset any internal state of the run or model, so they can be re-used in another run ResetRun().
        // Do not change input or drone data. Do not delete config references.
        public virtual void RunStart()
        {
            CurrInputVideoFrame = null;
            StopRunning = false;
            ProcessDurationMs = 0;

            ConfigureModelScope();
            ProcessDrawScope.Reset(this, Drone);
            CombDrawPath.Reset(ProcessDrawScope);

            ProcessAll?.ProcessStartWrapper();
        }


        // Update (draw) the graph(s), labels etc.
        public void DrawUI(
            Image<Bgr, byte> inputFrame,
            Image<Bgr, byte> displayFrame)
        {
            if (PSM.CurrBlockId == 1)
            {
                ProcessDrawScope.Reset(this, Drone);
                CombDrawPath.Reset(ProcessDrawScope);
            }

            RunParent.DrawUI(this, inputFrame, displayFrame);
        }


        // Convert double duration (in seconds or ms) to string with 0 dp.
        public static string DurationToString(double secs)
        {
            var secsStr = VideoModel.DurationSecToString(secs, 0);

            return (secsStr == "0" ? "<1" : secsStr);
        }


        // Show initial summary of step processing effort
        public void ShowStepProgress()
        {
            RunParent.ShowStepProgress(this);
        }


        // End running the model process & save changes to the datastore.
        public void SafeEndRunning()
        {
            try
            {
                DataStore.Open();
                EndRunning();
                DataStore.Close();
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSpace.RunVideo.SafeEndRunning", ex);
            }
        }


        public void RefreshAll()
        {
            RunParent.RefreshAll();
        }


        // Process the input video frame by frame and display/save the output video
        public void Run()
        {
            try
            {
                var inputVideo = Drone.InputVideo;

                RunParent.DrawObjectGrid(this, false);

                ProcessDurationMs = 0;
                var elapsedWatch = Stopwatch.StartNew();

                // Create a video file writer to output the processed input video to.
                (var videoWriter, var outputVideoFilename) =
                    CombSave.CreateVideoWriter(RunConfig, InputVideoFileName(), VideoBase.Fps, VideoBase.ImageSize);

                RunStart();
                PSM.CurrBlockId = 0;
                RefreshAll();

                if (PSM.InputVideoDurationMs < 0)
                {
                    RunParent.BadDuration(this);
                    return;
                }

                ShowStepProgress();
                RefreshAll();

                // Ensure we trigger a ProcessFlightLegChange start event on the first block (if needed)
                PSM.CurrRunLegId = UnknownValue;

                Image<Bgr, byte>? modifiedInputImage = null;
                Image<Bgr, byte>? modifiedDisplayImage = null;
                while (true)
                {
                    int prevLegId = PSM.CurrRunLegId;

                    // Move to the next video frame, processing block, and maybe flight section
                    PSM.CurrBlockId++;

                    // If process at max speed then (mostly) suppress processing of DisplayFrame and updating of UI
                    bool suppressUiUpdates =
                        RunConfig.MaxRunSpeed &&
                        (PSM.CurrBlockId != 1) &&                   // Always show the first block
                        (PSM.CurrBlockId < PSM.LastBlockId - 1) &&  // Always show the last block
                        (PSM.CurrBlockId % 100 != 0);               // Always show every 100th block

                    var calcWatch = Stopwatch.StartNew();

                    if (!Drone.HaveFrames())
                        break;

                    // Convert the already loaded Mat(s) into Image(s)
                    (var currInputImage, var currDisplayImage) = ConvertImages();

                    if (prevLegId != PSM.CurrRunLegId)
                    {
                        if (PSM.CurrRunLegId > 0)
                            // Surprisingly, a long series of sequential "seek next frame" GetVideoFrames 
                            // calls can give a CurrVideoFrameMs value that is, after 100 seconds, 400ms different
                            // from the CurrVideoFrameMs value if we seek direct to CurrVideoFrameID!
                            // So at the start of each new Leg we do a direct (slow) seek.
                            Drone.SetAndGetCurrFrames(PSM.CurrInputFrameId);

                        if (!Drone.HaveFrames())
                            break;

                        (currInputImage, currDisplayImage) = ConvertImages();

                        Assert(inputVideo.CurrFrameId == PSM.CurrInputFrameId, "RunVideo.Run: Bad FrameId 1");

                        // Process start &/or end of drone flight legs.
                        ProcessAll.OnObservation( ProcessEventEnum.LegEnd_Before, EventArgs.Empty);
                        ProcessFlightLegChange(prevLegId, PSM.CurrRunLegId);

                        // If we have just ended a leg change, then may have just calculated FixAltM
                        // so display the UI so the object-feature-lines are redrawn using the refined locations.
                        if (PSM.CurrRunLegId <= 0)
                            suppressUiUpdates = false;
                    }

                    // Frames occur say every 1/30 second.
                    // User can specify any end point in ms.
                    // Ensure we don't exceed the user specified end point.
                    if (PSM.CurrInputFrameMs > PSM.LastVideoFrameMs)
                        break;


                    // On rare occassions, the CurrRunVideoFrameMs to CurrRunStepId translation wobble can 
                    // position us one frame outside the approved range. This "unapproved" step may have values
                    // outside the TardisSummary values, causing the graphing code to fail its Asserts.
                    if ((Drone.HasFlightSteps) && (PSM.CurrRunStepId > MaxTardisId))
                        break;

                    CurrInputVideoFrame = currInputImage.Clone();

                    // Apply process model to this new frame
                    var thisBlock = AddBlockAndProcessInputVideoFrame();

                    Assert(inputVideo.CurrFrameId == thisBlock.InputFrameId, "RunVideo.Run: Bad FrameId 2");

                    // If we need it, draw the output for this new frame
                    modifiedInputImage = null;
                    modifiedDisplayImage = null;
                    if ((videoWriter != null) || (!suppressUiUpdates))
                        (modifiedInputImage, modifiedDisplayImage) =
                            DrawVideoFrames(thisBlock, CurrInputVideoFrame, currDisplayImage);

                    // Save the output frame to disk. 
                    if ((videoWriter != null) && (modifiedInputImage != null))
                        videoWriter.Write(modifiedInputImage.Mat);

                    // Calc effort excludes UI updates
                    ProcessDurationMs += (int)calcWatch.Elapsed.TotalMilliseconds;

                    // Always show summary of processing effort
                    ShowStepProgress();
                    if (!suppressUiUpdates)
                    {
                        // Show input/display images & update the graphs
                        DrawUI(modifiedInputImage, modifiedDisplayImage);
                        RefreshAll();
                    }

                    // Has user clicked the Stop button?
                    if (StopRunning)
                        break;

                    // Should we stop processing?
                    if (Drone.HasFlightSteps)
                    {
                        if (PSM.CurrRunStepId == MaxTardisId)
                            break;
                    }
                    else
                    {
                        if (PSM.CurrInputFrameMs >= PSM.LastVideoFrameMs)
                            break;
                    }

                    // Pause after processing of each frame - allowing user click the Stop button
                    switch (RunConfig.RunSpeed)
                    {
                        case RunSpeedEnum.Max: break; // No pause
                        default:
                        case RunSpeedEnum.Fast: CvInvoke.WaitKey(5); break;
                        case RunSpeedEnum.Medium: CvInvoke.WaitKey(50); break;
                        case RunSpeedEnum.Slow: CvInvoke.WaitKey(500); break;
                    }

                    // Move to the next frame(s)
                    if (!Drone.GetNextFrames())
                        break;
                }

                // End the last leg (if any)
                if (PSM.CurrRunLegId > 0)
                {
                    ProcessAll.OnObservation(ProcessEventEnum.LegEnd_Before, EventArgs.Empty);
                    ProcessFlightLegChange(PSM.CurrRunLegId, UnknownValue);
                }
                ProcessAll.ProcessEndWrapper();

                var saveWatch = Stopwatch.StartNew();

                // Finalise video content on disk. Quick.
                videoWriter?.Dispose(); // This finalises the video 

                RunParent.ShowRunSummary(
                    string.Format("{0}\nFrames: {1} of {2}\nUpdating datastore",
                        DescribeSignificantObjects(), PSM.CurrBlockId, PSM.LastBlockId));

                // End running (update) the model process
                // & save changes to the datastore.
                SafeEndRunning();

                DrawUI(modifiedInputImage, modifiedDisplayImage);
                RunParent.DrawObjectGrid(this, true);
                RunParent.ShowRunSummary(
                    string.Format("{0}\nFrames: {1} of {2}\nSaving video",
                        DescribeSignificantObjects(), PSM.CurrBlockId, PSM.LastBlockId));
                RefreshAll();

                // Show finalised process results
                RunParent.ShowRunSummary(
                    string.Format("{0}\nFrames: {1} of {2}\nSaved all",
                        DescribeSignificantObjects(), PSM.CurrBlockId, PSM.LastBlockId));

                videoWriter = null;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSpace.RunVideo.Run", ex);
            }
        }


        public DataPairList GetEffort()
        {
            var answer = new DataPairList()
            {
                { "LoadTimeMs", Drone.EffortDurations.LoadEffortMs },
                { "ProcessDurationMs", ProcessDurationMs },
            };

            if (StopRunning)
                answer.Add("Finally", "Stopped");
            else
                answer.Add("Finally", "Finished");

            return answer;
        }
    }


    // Video class that uses the ImageStandard techniques. No persistance of info between frames
    internal class RunVideoStandard : RunVideo
    {
        public RunVideoStandard(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone) 
            : base(parent, config, dataStore, drone, ProcessFactory.NewProcessModel(config.ProcessConfig, drone))
        {
        }


        public override void ProcessFlightLegChange(int prevLegId, int currLegId) { }


        // Process/analyse a single frame at a time.
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = ProcessFactory.NewBlock(this);
                ProcessAll.Blocks.AddBlock(thisBlock, this, Drone);

                var inputImage = CurrInputVideoFrame.Clone();
                RunConfig.Run(RunConfig, ref inputImage);

                return thisBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSpace.StandardVideoProcessor.ProcessFrame", ex);
            }
        }


        // Do any final activity at the end processing of video
        public override void EndRunning()
        {
            StandardSave dataWriter = new(Drone, DataStore);
            dataWriter.StandardProcess(RunConfig, GetEffort(), GetSettings(), this, ProcessAll);
            base.EndRunning();
        }


        // Save just the process settings to the DataStore
        public override void SaveProcessSettings()
        {
            DataStore.Open();
            StandardSave datawriter = new(Drone, DataStore);
            datawriter.StandardProcess(RunConfig, GetEffort(), GetSettings(), this, ProcessAll);
            DataStore.Close();
        }
    };


    // Class to implement processing of a video, with information persisted from frame to frame in member data.
    abstract public class RunVideoPersist : RunVideo
    {
        // Persisted information across many frames
        protected Image<Gray, byte>? PrevGray = null;


        public RunVideoPersist(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone, ProcessAll processAll) 
            : base(parent, config, dataStore, drone, processAll)
        {
        }


        // Reset any internal state of the run or model, so they can be re-used in another run. Do no change input or drone data.
        public override void RunStart()
        {
            PrevGray = null;
            base.RunStart();
        }
    }
}