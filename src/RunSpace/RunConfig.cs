// Copyright SkyComb Limited 2025. All rights reserved. 
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneLogic;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;
using SkyCombImage.CategorySpace;
using SkyCombImage.ProcessModel;


namespace SkyCombImage.RunSpace
{
    public enum RunProcessEnum { Comb, Yolo, Threshold, None };

    public enum RunSpeedEnum { Max, Fast, Medium, Slow, VerySlow };


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
        public RunProcessEnum RunProcess { get; set; } = RunProcessEnum.Threshold;

        // Between processing blocks, do we pause (slows down processing). That values max, fast, medium, slow. 
        public RunSpeedEnum RunSpeed { get; set; } = RunSpeedEnum.Fast;

        // Close the application after processing load file?
        public bool RunAutoClose { get; set; } = false;

        public bool[] HeightButtons { get; set; } = null;
        public bool[] SizeButtons { get; set; } = null;


        public RunConfig()
        {
            DroneConfig = new DroneConfigModel();
            ProcessConfig = new ProcessConfigModel();
            ImageConfig = new DrawImageConfig();
            HeightButtons = [true, true, true, true, true, true, true, true]; // 8 categories
            SizeButtons = [true, true, true, true, true, true, true]; // 7 categories
        }


        public bool RunFileExists()
        {
            return File.Exists(InputFileName);
        }


        public bool InputIsVideo
        {
            get
            {
                if (string.IsNullOrEmpty(InputFileName) || InputFileName.Length < 4)
                    return false;

                string ext = System.IO.Path.GetExtension(InputFileName).ToLowerInvariant();
                return ext == ".mp4" || ext == ".avi" || ext == ".ts" || ext == ".srt";
            }
        }

        public bool InputIsImages
        {
            get
            {
                return (InputDirectory != "") && (InputFileName == "");
            }
        }


        // We output files to the OutputDirectory (if specified) else the InputDirectory.
        // File directory name stripped from the input file name
        public string OutputElseInputDirectory { get { return OutputDirectory != "" ? OutputDirectory : InputDirectory; } }

        public bool MaxRunSpeed { get { return RunSpeed == RunSpeedEnum.Max; } }

        // Return just the last section of either the input file name or the folder name. 
        public string ShortInputSource { get { return InputFileName == "" ? VideoData.ShortFolderFileName(InputDirectory.Trim('\\')) : VideoData.ShortFolderFileName(InputFileName); } }



        // Describe (summarise) the process settings.
        public string DescribeProcess()
        {
            string answer = "Process: " + RunProcess + "\r\n";

            if (RunProcess != RunProcessEnum.None)
            {
                answer += "Speed: " + RunSpeed + "\r\n";

                if (ProcessConfig != null)
                {
                    answer += "Heat threshold: " + ProcessConfig.HeatThresholdValue + "\r\n";
                    if (RunProcess == RunProcessEnum.Yolo)
                        answer += "Confidence: " +
                            ProcessConfig.YoloDetectConfidence.ToString() + ", IoU: " +
                            ProcessConfig.YoloIoU.ToString() + "\r\n";

                    if (ProcessConfig.SaveAnnotatedVideo)
                        answer += "Save annotated video.\r\n";

                    if (ProcessConfig.SaveObjectData == SaveObjectDataEnum.Significant)
                        answer += "Save significant objects.\r\n";
                    else if (ProcessConfig.SaveObjectData == SaveObjectDataEnum.All)
                        answer += "Save all objects.\r\n";
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
                CategoryList = settings.CategoryList;
            }
            catch (Exception ex)
            {
                throw ThrowException("RunConfig.LoadJsonSettings", ex);
            }
        }


        public bool InRange(ProcessObjectModel processObject)
        {
            (var _, var heightIndex) = MasterHeightModelList.HeightMToClass(processObject);
            (var _, var sizeIndex) = MasterSizeModelList.CM2ToClass(processObject);
            return ((HeightButtons[heightIndex] == true) && (SizeButtons[sizeIndex] == true));
        }
    }
}