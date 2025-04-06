using SkyCombImage.RunSpace;
using System.Text.RegularExpressions;


namespace SkyCombImage.ProcessLogic
{
    public class ProcessFolder
    {
        // Lists files in the input directory
        public List<string> SrtFiles = new();
        public List<string> GpxFiles = new();
        public List<string> JpgFiles = new();


        // Function to recursively get file names in subfolders
        private void ListFilesInSubfolders(string folderPath, string filter = "_T")
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
                ListFilesInSubfolders(subfolder, filter);
            }
        }


        public void ListFilesInSubfolders(RunConfig runConfig)
        {
            SrtFiles = new();
            GpxFiles = new();
            JpgFiles = new();
            ListFilesInSubfolders(runConfig.InputDirectory);
        }
    }
}
