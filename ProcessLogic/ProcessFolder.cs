using SkyCombImage.RunSpace;
using System.Text.RegularExpressions;


namespace SkyCombImage.ProcessLogic
{
    public class ProcessFolder
    {
        // List of flight logs (associated with videos) in the input directory 
        public List<string> SrtFiles;

        // List of sub-folders and images in the input directory 
        public List<string> ImageFolders;
        public List<string> JpgFiles;

        // List of KML (Keyhole Markup Language) files are text files that store geographic data like points, lines, polygons, and images.
        public List<string> KmlFiles;


        public ProcessFolder()
        {
            Reset();
        }


        public void Reset()
        {
            SrtFiles = new();
            KmlFiles = new();
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
        private void ListInputFilesInSubfolders(string folderPath)
        {
            string filenamefilter = "*_t*";
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
                    case ".jpg":
                    case ".jpeg": JpgFiles.Add(file); break;
                }
            }

            // Recursively list files in subfolders
            string[] subfolders = Directory.GetDirectories(folderPath);
            foreach (string subfolder in subfolders)
                ListInputFilesInSubfolders(subfolder);
        }


        // Create a list of the video files in the input directory and subfolders
        public void ListInputFilesInSubfolders(RunConfig runConfig)
        {
            ListInputFilesInSubfolders(runConfig.InputDirectory);
        }


        // Function to recursively get names of folders that contain multiple jpg files
        private void ListImagesInSubfolders(string folderPath)
        {
            string filenamefilter = "*_t*";
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
                if (num_files_found == 2)
                    ImageFolders.Add(folderPath);
            }

            // Recursively list files in subfolders
            string[] subfolders = Directory.GetDirectories(folderPath);
            foreach (string subfolder in subfolders)
                ListImagesInSubfolders(subfolder);
        }


        // Create a list of the folders and subfolders that contain multiple jpg files
        public void ListImageFoldersAndSubfolders(RunConfig runConfig)
        {
            ListImagesInSubfolders(runConfig.InputDirectory);
        }


        // Function to recursively get names of folders that contain multiple jpg files
        private void ListKmlsInSubfolders(string folderPath)
        {
            string[] files = Directory.GetFiles(folderPath);
            foreach (string file in files)
            {
                if (file.Length < 5)
                    continue;
                string suffix = file.Substring(file.Length - 4, 4);
                if (suffix.ToLower() == ".kml")
                    KmlFiles.Add(file);
            }

            // Recursively list files in subfolders
            string[] subfolders = Directory.GetDirectories(folderPath);
            foreach (string subfolder in subfolders)
                ListKmlsInSubfolders(subfolder);
        }


        // Create a list of the folders and subfolders that contain multiple jpg files
        public void ListKmlFoldersAndSubfolders(RunConfig runConfig)
        {
            ListKmlsInSubfolders(runConfig.InputDirectory);
        }
    }
}
