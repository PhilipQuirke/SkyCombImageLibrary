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
using SkyCombImageLibrary.RunSpace;


// Namespace for processing of a video made up of multiple images (frames).
// Some classes contain code that persists information between frames
namespace SkyCombImage.RunSpace
{
    public class DroneIntervalList : List<DroneIntervalModel>
    {
        public void Add(float startS, float endS)
        {
            Add(new DroneIntervalModel(startS, endS));
        }
    }


    // Base video class that does not persist information betweem frames
    abstract public class RunVideo : ProcessScope
    {
        public RunParent RunParent { get; }

        // The configuration data used in processing the video
        public RunConfig RunConfig { get; set; }
        // We may be given a series of intervals to process
        public DroneIntervalList? RunIntervals { get; set; } = null;
 
        // All drone input data: video (definitely), flight (maybe) and ground (maybe) data 
        // public Drone Drone { get; set; }

        // The data store (spreadsheet) that meta-data is saved to / read from. 
        public DroneDataStore DataStore { get; set; }

        // The processing model apply to to the video. 
        public ProcessAll ProcessAll { get; set; }

        public CategoryAll CategoryAll { get; set; }

        // Basic attributes of the input video
        public VideoModel? VideoBase = null;


        // Has user requested we stop processing?
        public bool StopRunning { get; set; } = false;

        // The process calculation effort (duration) in milliseconds
        int ProcessDurationMs = 0;


        public Image<Bgr, byte>? ModifiedInputImage = null; 
        public Image<Bgr, byte>? ModifiedDisplayImage = null;



        // How to draw various graphs and charts
        public ProcessDrawScope ProcessDrawScope;
        public CombDrawPath CombDrawPath;
        //public DrawCombAltitudeByTime DrawCombAltitudeByTime;
        public ProcessDrawAltitudeByLinealM DrawCombAltitudeByLinealM;
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

            ProcessDrawScope = new(ProcessAll, this, Drone);

            // What is the maximum scope of objects we draw?
            ProcessObjList? objList = null;
            ObjectDrawScope? drawObjectScope = null;
            if (ProcessAll != null)
            {
                // Only consider objects in the current ProcessScope.
                // This is effectively the intersects of what legs were processed
                // and what legs are currently selected
                objList = ProcessAll.ProcessObjects.FilterByProcessScope(this);
                if ((objList != null) && (objList.Count > 0))
                {
                    // Default the filters based on these objects.
                    // This default shows the value range of these objects. 
                    drawObjectScope = new(ProcessAll, this, Drone);
                    drawObjectScope.SetObjectRange(objList);
                }
            }
            CombDrawPath = new(ProcessDrawScope, objList, drawObjectScope);

            //DrawCombAltitudeByTime = new(processAll, DrawScope);
            DrawCombAltitudeByLinealM = new(processAll, ProcessDrawScope);
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
        public void LoadDataStoreConfigSettings()
        {
            StandardLoad dataReader = new(DataStore);
            dataReader.LoadConfigSettings(RunConfig);
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


        public void ResetModifiedImages()
        {
            ModifiedInputImage?.Dispose();
            ModifiedInputImage = null;
            ModifiedDisplayImage?.Dispose();
            ModifiedDisplayImage = null;
        }


        // The input video file name to process.
        public virtual string InputVideoFileName()
        {
            return RunConfig.InputFileName;
        }


        // Load model data from the previous run (if any).
        public void LoadDataStore()
        {
            try
            {
                BlockLoad datareader1 = new(DataStore);
                datareader1.ProcessBlocks(ProcessAll, Drone);

                if ((ProcessAll is CombProcess) || (ProcessAll is YoloProcess))
                {
                    StandardLoad datareader2 = new(DataStore);
                    datareader2.ProcessFeatures(ProcessAll);

                    datareader2.ProcessObjects(ProcessAll);
                    var objectListSettings = datareader2.ObjectListSettings();
                    if (objectListSettings != null)
                        ProcessAll.ProcessObjects.LoadSettings(objectListSettings);

                    datareader2.ProcessSpans(ProcessAll, Drone);

                    // Reset the FlightStep.FixAltM values from the ProcessSpan data
                    ProcessAll.ProcessSpans.SetFixAltMAfterLoad(Drone.InputVideo, Drone);

                    // Link each object to its features
                    foreach (var feature in ProcessAll.ProcessFeatures)
                        if (feature.Value.ObjectId >= 0)
                            ProcessAll.ProcessObjects.SetLinksAfterLoad(feature.Value);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("RunVideoYolo.LoadDataStore", ex);
            }
        }


        // Describe the objects found
        public virtual string DescribeSignificantObjects()
        {
            return "";
        }


        // Process start &/or end of drone flight legs.
        public abstract void ProcessFlightLegChange(ProcessScope scope, int prevLegId, int currLegId);


        // Process/analyse a single input video frame 
        public abstract ProcessBlock AddBlockAndProcessInputVideoFrame();


        // Process a single input and (maybe) display video frame for the specified block, returning the modified input&display frames to show 
        public void DrawVideoFrames(ProcessBlockModel block)
        {
            ResetModifiedImages();

            if (CurrInputImage == null)
                return;

            (ModifiedInputImage, ModifiedDisplayImage) =
                DrawSpace.DrawVideoFrames.Draw(
                    RunConfig.RunProcess, RunConfig.ProcessConfig, RunConfig.ImageConfig, Drone,
                    block, ProcessAll, UnknownValue,
                    CurrInputImage, CurrDisplayImage);

            if(ModifiedInputImage != null)
                DrawYawPitchZoom.Draw(ref ModifiedInputImage, Drone, CurrRunFlightStep);
        }


         // Return the data to show in the ObjectGrid in the Main Form
        public virtual List<object[]> GetObjectGridData(bool mainForm)
        {
            return null;
        }


        // Do any final activity at the end processing (aka running) of video / flight data
        public virtual void RunEnd()
        {
        }


        // Save just the process settings to the DataStore
        public abstract void SaveProcessSettings();


        // Reset any internal state of the run or model, so they can be re-used in another run ResetRun().
        // Do not change input or drone data. Do not delete config references.
        public void RunStart_Process()
        {
            StopRunning = false;
            ProcessDurationMs = 0;

            ProcessAll?.RunStart();

            ProcessAll?.OnObservation(ProcessEventEnum.RunStart);
        }
        public void RunStart_Interval()
        {
            ResetCurrImages();
            ResetModifiedImages();
            ConfigureModelScope();
            ProcessDrawScope.Reset(this, Drone);
            CombDrawPath.Reset(ProcessDrawScope);
            
            ProcessAll?.OnObservation(ProcessEventEnum.IntervalStart);
        }


        // Update (draw) the graph(s), labels etc.
        public void DrawUI()
        {
            if (PSM.CurrBlockId == 1)
            {
                ProcessDrawScope.Reset(this, Drone);
                CombDrawPath.Reset(ProcessDrawScope);
            }

            RunParent.DrawUI(this);
        }


        // Convert double duration (in seconds or ms) to string with 0 dp.
        public static string DurationToString(double secs)
        {
            var secsStr = VideoModel.DurationSecToString(secs, 0);

            return (secsStr == "0" ? "<1" : secsStr);
        }


        // Show initial summary of step processing effort
        public void ShowStepProgress(int intervalCount, int stepCount)
        {
            RunParent.ShowStepProgress(this, intervalCount, stepCount);
        }


        public void ShowRunSummary(string suffix)
        {
            RunParent.ShowRunSummary(
                string.Format("{0}\nFrames: {1} of {2}\n{3}",
                    DescribeSignificantObjects(), PSM.CurrBlockId, PSM.LastBlockId, suffix));
        }


        // End running the model process & save changes to the datastore.
        public void SafeRunEnd()
        {
            try
            {
                DataStore.Open();
                RunEnd();
                DataStore.Close();

                ProcessAll.OnObservation(ProcessEventEnum.RunEnd);
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSpace.RunVideo.SafeRunEnd", ex);
            }
        }


        // Pause after processing of each frame - allowing user click the Stop button
        public void WaitKey()
        { 
            switch (RunConfig.RunSpeed)
            {
                case RunSpeedEnum.Max: break; // No pause
                default:
                case RunSpeedEnum.Fast: CvInvoke.WaitKey(5); break;
                case RunSpeedEnum.Medium: CvInvoke.WaitKey(50); break;
                case RunSpeedEnum.Slow: CvInvoke.WaitKey(500); break;
            }
        }


        // Should we stop processing?
        public bool StopProcessing_Part1()
        {
            // Frames occur say every 1/30 second.
            // User can specify any end point in ms.
            // Ensure we don't exceed the user specified end point.
            if (PSM.CurrInputFrameMs > PSM.LastVideoFrameMs)
                return true;

            // On rare occassions, the CurrRunVideoFrameMs to CurrRunStepId translation wobble can 
            // position us one frame outside the approved range. This "unapproved" step may have values
            // outside the TardisSummary values, causing the graphing code to fail its Asserts.
            if ((Drone.HasFlightSteps) && (PSM.CurrRunStepId > MaxTardisId))
                return true;

            return false;
        }


        // Should we stop processing?
        public bool StopProcessing_Part2()
        {
            if (StopRunning)
                // User has clicked the Stop button
                return true;

            if (Drone.HasFlightSteps)
            {
                if (PSM.CurrRunStepId == MaxTardisId)
                    // We have reached the end of the flight step scope
                    return true;
            }
            else
            {
                // Rare case for video only processing
                if (PSM.CurrInputFrameMs >= PSM.LastVideoFrameMs)
                    // We have reached the end of the video time scope
                    return true;
            }

            return false;
        }


        public void RefreshAll()
        {
            RunParent.RefreshAll();
        }


        private DroneIntervalList SafeDroneIntervals()
        {
            if (RunIntervals != null)
                return RunIntervals;

            // Normal case there is exactly one interval
            DroneIntervalList answer = new();
            answer.Add(
                RunConfig.DroneConfig.RunVideoFromS,
                RunConfig.DroneConfig.RunVideoToS);
            return answer;
        }



        // Process the input video frame by frame and display/save the output video
        public void Run()
        {
            try
            {
                var inputVideo = Drone.InputVideo;

                RunParent.DrawObjectGrid(this, false);

                RunStart_Process();

                // Create an output video file writer (if user wants MP4 output)
                (var videoWriter, var _) =
                    StandardSave.CreateVideoWriter(RunConfig, InputVideoFileName(), VideoBase.Fps, VideoBase.ImageSize);

                int intervalCount = 0;
                var safeDroneIntervals = SafeDroneIntervals();
                foreach (var interval in safeDroneIntervals)
                {
                    intervalCount++;
                    RunConfig.DroneConfig.RunVideoFromS = interval.RunVideoFromS;
                    RunConfig.DroneConfig.RunVideoToS = interval.RunVideoToS;

                    RunStart_Interval();
                    PSM.CurrBlockId = 0;
                    RefreshAll();

                    if (PSM.InputVideoDurationMs < 0)
                    {
                        RunParent.BadDuration(this);
                        return;
                    }

                    int stepCount = 0;
                    ShowStepProgress(intervalCount, stepCount);
                    RefreshAll();

                    // Ensure we trigger a ProcessFlightLegChange start event on the first block (if needed)
                    PSM.CurrRunLegId = UnknownValue;

                    while (true)
                    {
                        int prevLegId = PSM.CurrRunLegId;

                        // Move to the next video frame, processing block, and maybe flight section
                        PSM.CurrBlockId++;
                        stepCount++;

                        // If process at max speed then (mostly) suppress processing of DisplayFrame and updating of UI
                        bool suppressUiUpdate =
                            RunConfig.MaxRunSpeed &&
                            (PSM.CurrBlockId != 1) &&                   // Always show the first block
                            (PSM.CurrBlockId < PSM.LastBlockId - 1) &&  // Always show the last block
                            (PSM.CurrBlockId % 100 != 0);               // Always show every 100th block

                        var calcWatch = Stopwatch.StartNew();

                        if (!Drone.HaveFrames())
                            break;

                        // Convert the already loaded Mat(s) into Image(s)
                        ConvertCurrImages();

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

                            ConvertCurrImages();

                            Assert(inputVideo.CurrFrameId == PSM.CurrInputFrameId, "RunVideo.Run: Bad FrameId 1");

                            // Process start &/or end of drone flight legs.
                            ProcessFlightLegChange(this, prevLegId, PSM.CurrRunLegId);

                            // If we have just ended a leg change, then may have just calculated FixAltM
                            // so display the UI so the object-feature-lines are redrawn using the refined locations.
                            if (PSM.CurrRunLegId <= 0)
                                suppressUiUpdate = false;
                        }

                        // Should we stop processing?
                        if (StopProcessing_Part1())
                            break;

                        // Apply process model to this new frame
                        var thisBlock = AddBlockAndProcessInputVideoFrame();

                        Assert(inputVideo.CurrFrameId == thisBlock.InputFrameId, "RunVideo.Run: Bad FrameId 2");

                        // If we need it, draw the output for this new frame
                        if ((videoWriter != null) || (!suppressUiUpdate))
                            DrawVideoFrames(thisBlock);

                        // Save the output frame to disk. 
                        if ((videoWriter != null) && (ModifiedInputImage != null))
                            videoWriter.Write(ModifiedInputImage.Mat);

                        // Calc effort excludes UI updates
                        ProcessDurationMs += (int)calcWatch.Elapsed.TotalMilliseconds;

                        // Show summary of processing effort
                        ShowStepProgress(intervalCount, stepCount);
                        if (!suppressUiUpdate)
                        {
                            // Show input/display images & update the graphs
                            DrawUI();
                            RefreshAll();
                        }

                        // Pause after processing of each frame - allowing user click the Stop button
                        WaitKey();

                        // Should we stop processing?
                        if (StopProcessing_Part2())
                            break;

                        // Move to the next frame(s)
                        if (!Drone.GetNextFrames())
                            break;
                    }

                    // End the last leg (if any)
                    if (PSM.CurrRunLegId > 0)
                        ProcessFlightLegChange(this, PSM.CurrRunLegId, UnknownValue);

                    ProcessAll.EndInterval();
                }

                ShowRunSummary("Finalise video");
                // Finalise video content on disk. Quick.
                videoWriter?.Dispose(); // This finalises the video 

                ShowRunSummary("Update datastore");
                // End running (update) the model process
                // & save changes to the datastore.
                SafeRunEnd();

                DrawUI();
                ResetModifiedImages();
                ResetCurrImages();
                RunParent.DrawObjectGrid(this, true);
                RefreshAll();

                // Show finalised process results
                ShowRunSummary("Saved all");

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


        public override void ProcessFlightLegChange(ProcessScope scope,int prevLegId, int currLegId) { }


        // Process/analyse a single frame at a time.
        public override ProcessBlock AddBlockAndProcessInputVideoFrame()
        {
            try
            {
                var thisBlock = ProcessFactory.NewBlock(this);
                ProcessAll.Blocks.AddBlock(thisBlock, this, Drone);

                var inputImage = CurrInputImage.Clone();

                // Process (analyse) a single image (using any one ProcessName) and returns an image.
                DrawImage.Draw(RunConfig.RunProcess, RunConfig.ProcessConfig, RunConfig.ImageConfig, ref inputImage);

                return thisBlock;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunSpace.RunVideoStandard.AddBlockAndProcessInputVideoFrame", ex);
            }
        }


        // Do any final activity at the end processing of video
        public override void RunEnd()
        {
            StandardSave dataWriter = new(Drone, DataStore);
            dataWriter.StandardProcess(RunConfig, GetEffort(), GetSettings(), this, ProcessAll);
            base.RunEnd();
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
        public RunVideoPersist(RunParent parent, RunConfig config, DroneDataStore dataStore, Drone drone, ProcessAll processAll) 
            : base(parent, config, dataStore, drone, processAll)
        {
        }
    }
}