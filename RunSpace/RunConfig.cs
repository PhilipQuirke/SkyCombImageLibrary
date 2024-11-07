// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.RunSpace
{
    public enum RunProcessEnum { Comb, Yolo, Threshold, None };

    public enum RunSpeedEnum { Max, Fast, Medium, Slow };


    // Configuration settings related to running processing models.
    //
    // When processing a video for the first time:
    //      - the below member data default values are used, with a few override default values loaded from App.Config
    //      - the member data values are saved to the DataStore (spreadsheet) specific to this video.
    // When processing that same video for a second or subsequent time:
    //      - ALL member data values are loaded from the DataStore specific to this video 
    //      - So if you want to trial different values for the same video, alter the setting in the DATASTORE.
    //      - WARNING: Changing the values below will have no effect.
    public class RunConfig : SettingsBase
    {
        public DroneConfigModel DroneConfig;
        public ProcessConfigModel ProcessConfig;
        public DrawImageConfig ImageConfig;

        // InputDirectory with an optional file name. Trailing "\" (if any) is trimmed
        public string InputFileName { get; set; } = "";

        // Name of main process to run: yolo, comb, threshold, none.
        public RunProcessEnum RunProcess { get; set; } = RunProcessEnum.Yolo;

        // Between processing blocks, do we pause (slows down processing). That values max, fast, medium, slow. 
        public RunSpeedEnum RunSpeed { get; set; } = RunSpeedEnum.Max;

        // Close the application after processing load file?
        public bool RunAutoClose { get; set; } = false;


        public RunConfig()
        {
            DroneConfig = new DroneConfigModel();
            ProcessConfig = new ProcessConfigModel();
            ImageConfig = new DrawImageConfig();
        }


        public bool RunFileExists()
        {
            return File.Exists(InputFileName);
        }


        public bool RunFileIsVideo()
        {
            if (InputFileName.Length < 5)
                return false;

            string suffix = InputFileName.Substring(InputFileName.Length - 4, 4).ToLower();
            return suffix == ".mp4" || suffix == ".avi";
        }


        // We output files to the OutputDirectory (if specified) else the InputDirectory.
        // File directory name stripped from the input file name
        public string OutputElseInputDirectory { get { return OutputDirectory != "" ? OutputDirectory : InputDirectory; } }

        public bool MaxRunSpeed { get { return RunSpeed == RunSpeedEnum.Max; } }



        // Describe (summarise) the process settings.
        public string DescribeProcess()
        {
            string answer = "Process: " + RunProcess + "\r\n";

            if (RunProcess != RunProcessEnum.None)
            {
                answer += "Speed: " + RunSpeed + "\r\n";

                if (ProcessConfig != null)
                {
                    answer += "Heat Threshold: " + ProcessConfig.HeatThresholdValue + "\r\n";
                    if (RunProcess == RunProcessEnum.Yolo)
                        answer += "Confidence: " +
                            ProcessConfig.YoloDetectConfidence.ToString() + ", IoU: " +
                            ProcessConfig.YoloIoU.ToString() + "\r\n";

                    if (ProcessConfig.SaveAnnotatedVideo)
                        answer += "Save Annotated Video\r\n";

                    if (ProcessConfig.SaveObjectData == SaveObjectDataEnum.Significant)
                        answer += "Save Significant Objects\r\n";
                    else if (ProcessConfig.SaveObjectData == SaveObjectDataEnum.All)
                        answer += "Save All Objects\r\n";
                }
            }

            return answer;
        }


        // Get the class's settings as datapairs (e.g. for saving to a spreadsheet)
        public DataPairList GetSettings()
        {
            return new DataPairList
            {
                { "RunModel", RunProcess.ToString() },
                { "RunSpeed", RunSpeed.ToString() },
            };
        }


        // Load this object's settings from strings (loaded from a spreadsheet)
        // This function must align to the above GetSettings function.
        public void LoadSettings(List<string> settings)
        {
            if (settings != null)
            {
                RunProcess = (RunProcessEnum)Enum.Parse(typeof(RunProcessEnum), settings[0]);
                RunSpeed = (RunSpeedEnum)Enum.Parse(typeof(RunSpeedEnum), settings[1]);
            }
        }


        // Load this object's settings from json
        public void LoadJsonSettings()
        {
            try
            {
                var settings = JsonSettings.LoadSettings();
                InputDirectory = settings.InputDirectory.TrimEnd('\\');
                GroundDirectory = settings.GroundDirectory.TrimEnd('\\');
                YoloDirectory = settings.YoloDirectory.TrimEnd('\\');
                OutputDirectory = settings.OutputDirectory.TrimEnd('\\');
                RecentFiles = settings.RecentFiles;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunConfig.LoadJsonSettings", ex);
            }
        }
    }
}