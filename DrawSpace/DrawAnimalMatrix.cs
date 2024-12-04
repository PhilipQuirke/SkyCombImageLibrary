using SkyCombImageLibrary.ProcessModel;
using System.Drawing;


namespace SkyCombImageLibrary.DrawSpace
{

    public class AnimalMatrixDrawer
    {
        private static Dictionary<string, int> CreateIndexMap(string[] classes)
        {
            var indexMap = new Dictionary<string, int>();
            for (int i = 0; i < classes.Length; i++)
            {
                indexMap[classes[i]] = i;
            }
            return indexMap;
        }

        public static Bitmap DrawAnimalMatrix(AnimalModelList animals)
        {
            // Define SizeClasses and HeightClasses
            string[] sizeClasses = { "?", "XXS", "XS", "S", "M", "L", "XL", "XXL" };
            string[] heightClasses = { "?", "G", "1f", "2f", "3f", "4f", "5f", "6f+" };

            // Create mappings for quick index lookup
            Dictionary<string, int> sizeClassIndices = CreateIndexMap(sizeClasses);
            Dictionary<string, int> heightClassIndices = CreateIndexMap(heightClasses);

            // Initialize counts and totals
            int[,] counts = new int[heightClasses.Length, sizeClasses.Length];
            int[] sizeClassTotals = new int[sizeClasses.Length];
            int[] heightClassTotals = new int[heightClasses.Length];

            // Populate counts and totals
            foreach (var animal in animals)
            {
                if (sizeClassIndices.TryGetValue(animal.SizeClass, out int sIndex) &&
                    heightClassIndices.TryGetValue(animal.HeightClass, out int hIndex))
                {
                    counts[hIndex, sIndex]++;
                    sizeClassTotals[sIndex]++;
                    heightClassTotals[hIndex]++;
                }
            }

            int cellWidth = 60;
            int cellHeight = 40;
            int labelWidth = 100;
            int labelHeight = 60;
            int imageHeight = 40;
            int totalWidth = labelWidth + cellWidth * sizeClasses.Length + 100;
            int totalHeight = labelHeight + cellHeight * heightClasses.Length + imageHeight + 100;

            Bitmap bmp = new Bitmap(totalWidth, totalHeight);
            using (Graphics g = Graphics.FromImage(bmp))
            {
                // Set high-quality rendering options
                g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

                // Fill background
                g.Clear(Color.White);

                // Fonts and brushes
                Font font = new Font("Arial", 12);
                Brush brush = Brushes.Black;
                Pen pen = new Pen(Color.Black);

                // Draw SizeClass labels and images
                for (int s = 0; s < sizeClasses.Length; s++)
                {
                    int x = labelWidth + s * cellWidth;
                    // Draw SizeClass label
                    g.DrawString(sizeClasses[s], font, brush, x + cellWidth / 2 - 15, 10);

                    // Draw image under SizeClass label (except "?")
                    if (sizeClasses[s] != "?")
                    {
                        Image img = GetSizeClassImage(sizeClasses[s]);
                        if (img != null)
                        {
                            int imgX = x + cellWidth / 2 - img.Width / 2;
                            int imgY = labelHeight - imageHeight + 10;
                            g.DrawImage(img, imgX, imgY, img.Width, img.Height);
                        }
                    }
                }

                // Draw HeightClass labels
                for (int h = 0; h < heightClasses.Length; h++)
                {
                    int y = labelHeight + h * cellHeight;
                    g.DrawString(heightClasses[h], font, brush, 10, y + cellHeight / 2 - 10);
                }

                // Draw counts in cells
                for (int h = 0; h < heightClasses.Length; h++)
                {
                    for (int s = 0; s < sizeClasses.Length; s++)
                    {
                        int count = counts[h, s];
                        if (count > 0)
                        {
                            int x = labelWidth + s * cellWidth;
                            int y = labelHeight + h * cellHeight;
                            g.DrawString(count.ToString(), font, brush, x + cellWidth / 2 - 10, y + cellHeight / 2 - 10);
                        }
                    }
                }

                // Draw grid lines
                DrawGridLines(g, sizeClasses.Length, heightClasses.Length, labelWidth, labelHeight, cellWidth, cellHeight, pen);

                // Draw totals for SizeClasses
                for (int s = 0; s < sizeClasses.Length; s++)
                {
                    int total = sizeClassTotals[s];
                    int x = labelWidth + s * cellWidth;
                    int y = labelHeight + cellHeight * heightClasses.Length + 20;
                    g.DrawString("Total: " + total, font, brush, x + cellWidth / 2 - 25, y);
                }

                // Draw totals for HeightClasses
                for (int h = 0; h < heightClasses.Length; h++)
                {
                    int total = heightClassTotals[h];
                    int x = labelWidth + cellWidth * sizeClasses.Length + 20;
                    int y = labelHeight + h * cellHeight;
                    g.DrawString("Total: " + total, font, brush, x, y + cellHeight / 2 - 10);
                }
            }

            return bmp;
        }

        private static void DrawGridLines(
            Graphics g,
            int sizeClassCount,
            int heightClassCount,
            int labelWidth,
            int labelHeight,
            int cellWidth,
            int cellHeight,
            Pen pen)
        {
            // Vertical lines
            for (int s = 0; s <= sizeClassCount; s++)
            {
                int x = labelWidth + s * cellWidth;
                int yStart = labelHeight;
                int yEnd = labelHeight + cellHeight * heightClassCount;
                g.DrawLine(pen, x, yStart, x, yEnd);
            }

            // Horizontal lines
            for (int h = 0; h <= heightClassCount; h++)
            {
                int y = labelHeight + h * cellHeight;
                int xStart = labelWidth;
                int xEnd = labelWidth + cellWidth * sizeClassCount;
                g.DrawLine(pen, xStart, y, xEnd, y);
            }
        }


        private static Image GetSizeClassImage(string sizeClass)
        {
            // Replace with the actual path to your images
            //string imagePath = $"images/{sizeClass}.png";
            //if (System.IO.File.Exists(imagePath))
            //{
            //    return Image.FromFile(imagePath);
            //}
            return null;
        }
    }
}

