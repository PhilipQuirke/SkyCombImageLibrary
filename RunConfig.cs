// Copyright SkyComb Limited 2023. All rights reserved. 
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneModel;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.ProcessModel
{
    public enum RunProcessEnum { Comb, Yolo, Threshold, GFTT, None };

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
    public class RunConfig : ConfigBase
    {
        public DroneConfigModel DroneConfig;
        public ProcessConfigModel ProcessConfig;
        public DrawImageConfig ImageConfig;


        // Name of image or video file to load at app start up. Can be just a directory path. Can be blank.
        public string InputFileName { get; set; } = "";

        // Ground path containing static ground contour data. Can be blank. Trailing "\" (if any) is trimmed
        public string GroundDirectory { get; set; } = "";

        // Directory path to store created (video and spreadsheet) files into. Can be blank. Trailing "\" (if any) is trimmed
        public string OutputDirectory { get; set; } = "";

        // Directory path/file to load YOLOv8 model from. If is a bare directory path, code appends "\yolo_v8_s_e100.onnx"
        // yolo_v8_s_e100.onnx was generated in and exported from Supervisely.
        public string YoloDirectory { get; set; } = "";

        // Name of main process to run: contour, distance, yolo, gftt, comb, threshold, none. Lowercase.
        public RunProcessEnum RunProcess { get; set; } = RunProcessEnum.Comb;

        // Between processing blocks, do we pause (slows down processing). That values max, fast, medium, slow. 
        public RunSpeedEnum RunSpeed { get; set; } = RunSpeedEnum.Fast;

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


        // File directory name stripped from the input file name
        public string InputDirectory()
        {
            return Path.GetDirectoryName(InputFileName).Trim('\\');
        }


        // We output files to the OutputDirectory (if specified) else the InputDirectory.
        // File directory name stripped from the input file name
        public string OutputElseInputDirectory()
        {
            if (OutputDirectory != "")
                return OutputDirectory;

            return Path.GetDirectoryName(InputFileName).Trim('\\');
        }


        public bool MaxRunSpeed { get { return RunSpeed == RunSpeedEnum.Max; } }



        // Describe (summarise) the process settings.
        public string DescribeProcess()
        {
            string answer =
                "Process: " + RunProcess + "\r\n" +
                "Speed: " + RunSpeed + "\r\n";

            if(ProcessConfig != null)
            {
                if( RunProcess == RunProcessEnum.Comb)
                    answer += "Heat Threshold: " + ProcessConfig.HeatThresholdValue + "\r\n";
                else
                    answer += "Detect/IoU/Merge: " +
                        ProcessConfig.YoloDetectConfidence.ToString() + "/" + 
                        ProcessConfig.YoloIoU.ToString() + "/" + 
                        ProcessConfig.YoloMergeConfidence.ToString() + "\r\n";

                answer += "Save Annotated Video: " + (ProcessConfig.SaveAnnotatedVideo ? "Yes" : "No") + "\r\n";

                if (ProcessConfig.SaveObjectData != SaveObjectDataEnum.None)
                    answer += "Save Objects: " + ProcessConfig.SaveObjectData + "\r\n";
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
            RunProcess = (RunProcessEnum)Enum.Parse(typeof(RunProcessEnum), settings[0]);
            RunSpeed = (RunSpeedEnum)Enum.Parse(typeof(RunSpeedEnum), settings[1]);
        }


        // Load this object's settings from strings (loaded from App.Config)
        // This function must align to the above GetSettings function.
        public void LoadSettings(DataPairList settings)
        {
            try
            {
                ProcessConfig.LoadSettings(settings);

                foreach (var setting in settings)
                    switch (setting.Key)
                    {
                        case "inputfilename":
                            InputFileName = setting.Value; break;
                        case "grounddirectory":
                            GroundDirectory = setting.Value.TrimEnd('\\'); break;
                        case "outputdirectory":
                            OutputDirectory = setting.Value.TrimEnd('\\'); break;
                        case "yolodirectory":
                            YoloDirectory = setting.Value.TrimEnd('\\'); break;
                    }
            }
            catch (Exception ex)
            {
                throw ThrowException("RunConfig.LoadSettings", ex);
            }
        }


        // Process (analyse) a single image (using any one ProcessName) and returns an image.
        // The output image shows hot pixel in green, with red rectangles bounding significant features.
        public static void Run(RunConfig config, ref Image<Bgr, byte> imgInput)
        {
            DrawSpace.DrawImage.Draw(config.RunProcess, config.ProcessConfig, config.ImageConfig, ref imgInput);
        }
    }
}