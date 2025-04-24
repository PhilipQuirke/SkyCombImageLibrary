using SkyCombImage.RunSpace;
using System.Text.RegularExpressions;


namespace SkyCombImage.ProcessLogic
{
    public class ProcessFolder
    {
        // Lists files in the input directory
        public List<string> SrtFiles;
        public List<string> GpxFiles;
        public List<string> JpgFiles;
        public List<string> ImageFolders;

        public ProcessFolder()
        {
            Reset();
        }


        private void Reset()
        {
            SrtFiles = new();
            GpxFiles = new();
            JpgFiles = new();
            ImageFolders = new();
        }


        public List<string> InputNames(bool inputIsVideo)
        {
            if (inputIsVideo)
                return SrtFiles;
            else
                return ImageFolders;
        }


        // Function to recursively get file names in subfolders
        private void ListVideoFilesInSubfolders(string folderPath, string filter = "_T")
        {
            string filenamefilter = "*" + filter.Trim().ToLower() + "*";
            string regexPattern = "^" + Regex.Escape(filenamefilter).Replace("\\*", ".*") + "$";

            // List files in the current folder
            string[] files = Directory.GetFiles(folderPath);
            foreach (string file in files)
            {
                string the_file = file.ToLower();

                if (the_file.Length < 5)
                    continue;
                if (!Regex.IsMatch(the_file, regexPattern, RegexOptions.IgnoreCase))
                    continue;

                string suffix = the_file.Substring(the_file.Length - 4, 4);
                switch (suffix)
                {
                    case ".srt": SrtFiles.Add(file); break;
                    case ".gpx": GpxFiles.Add(file); break;
                    case ".jpg":
                    case ".jpeg": JpgFiles.Add(file); break;
                }
            }

            // Recursively list files in subfolders
            string[] subfolders = Directory.GetDirectories(folderPath);
            foreach (string subfolder in subfolders)
            {
                ListVideoFilesInSubfolders(subfolder, filter);
            }
        }


        // Create a list of the video files in the input directory and subfolders
        public void ListVideoFilesInSubfolders(RunConfig runConfig)
        {
            Reset();
            ListVideoFilesInSubfolders(runConfig.InputDirectory);
        }


        // Function to recursively get names of folders that contain multiple jpg files
        private void ListImagesInSubfolders(string folderPath, string filter = "_T")
        {
            string filenamefilter = "*" + filter.Trim().ToLower() + "*";
            string regexPattern = "^" + Regex.Escape(filenamefilter).Replace("\\*", ".*") + "$";
            // List files in the current folder
            string[] files = Directory.GetFiles(folderPath);

            int num_files_found = 0;
            foreach (string file in files)
            {
                string the_file = file.ToLower();
                if (the_file.Length < 5)
                    continue;
                if (!Regex.IsMatch(the_file, regexPattern, RegexOptions.IgnoreCase))
                    continue;
                string suffix = the_file.Substring(the_file.Length - 4, 4);
                if (suffix == ".jpg" || suffix == ".jpeg")
                    num_files_found++;

                // Folder must have 2 or more images to be added to the list
                if (num_files_found>=2)
                {
                    ImageFolders.Add(folderPath);
                    break;
                }
            }

            // Recursively list files in subfolders
            string[] subfolders = Directory.GetDirectories(folderPath);
            foreach (string subfolder in subfolders)
            {
                ListImagesInSubfolders(subfolder, filter);
            }
        }


        // Create a list of the folders and subfolders that contain multiple jpg files
        public void ListImageFoldersAndSubfolders(RunConfig runConfig)
        {
            Reset();
            ListImagesInSubfolders(runConfig.InputDirectory);
        }

    }
}
