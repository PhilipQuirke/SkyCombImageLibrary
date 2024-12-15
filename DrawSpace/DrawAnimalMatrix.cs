using SkyCombImage.RunSpace;
using SkyCombImageLibrary.ProcessModel;
using System.Drawing;
using System.Linq.Expressions;
using System.Windows.Forms;
using static System.Runtime.InteropServices.JavaScript.JSType;


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

        public static (string message, Bitmap matrix) DrawAnimalMatrix(AnimalModelList animals, List<Image> sizeImages)
        {
            // Define SizeClasses and HeightClasses
            string[] sizeClasses = { "XXS", "XS", "S", "M", "L", "XL", "XXL" };
            string[] heightClasses = {"6f+", "5f", "4f", "3f", "2f", "1f", "G"  };

            // Create mappings for quick index lookup
            Dictionary<string, int> sizeClassIndices = CreateIndexMap(sizeClasses);
            Dictionary<string, int> heightClassIndices = CreateIndexMap(heightClasses);

            // Initialize counts and totals
            int[,] counts = new int[heightClasses.Length, sizeClasses.Length];
            int[] sizeClassTotals = new int[sizeClasses.Length];
            int[] heightClassTotals = new int[heightClasses.Length];
            int categorised = 0;

            // Populate counts and totals
            foreach (var animal in animals)
            {
                if (sizeClassIndices.TryGetValue(animal.SizeClass, out int sIndex) &&
                    heightClassIndices.TryGetValue(animal.HeightClass, out int hIndex))
                {
                    counts[hIndex, sIndex]++;
                    sizeClassTotals[sIndex]++;
                    heightClassTotals[hIndex]++;
                    categorised++;
                }
            }

            int cellWidth = 65;
            int cellHeight = 45;
            int labelWidth = 100;
            int labelHeight = 60;
            int imageHeight = 40;
            int totalWidth = labelWidth + cellWidth * sizeClasses.Length + 200;
            int totalHeight = labelHeight + cellHeight * heightClasses.Length + imageHeight + 70;
            string total;

            Bitmap bmp = new Bitmap(totalWidth, totalHeight);
            using (Graphics g = Graphics.FromImage(bmp))
            {
                // Set high-quality rendering options
                g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

                // Fonts and brushes
                Font font = new Font("Arial", 15);
                Font smallfont = new Font("Arial", 11);
                Brush brush = Brushes.Black;
                Pen pen = new Pen(Color.Black);
                int x, y, h;
                // Draw SizeClass labels and images
                for (int s = 0; s < sizeClasses.Length; s++)
                {
                    x = labelWidth + s * cellWidth;
                    // Draw SizeClass label
                    g.DrawString(sizeClasses[s], smallfont, brush, x + cellWidth / 2 - 15, 10);

                    // Draw image under SizeClass label. Note: removed "?".
                    Image img = sizeImages[s];
                    if (img != null)
                    {
                        int imgX = x + cellWidth / 2 - img.Width / 2;
                        int imgY = labelHeight - imageHeight + 10;
                        g.DrawImage(img, imgX, imgY, img.Width, img.Height);
                    }
                }

                for (h = 0; h < heightClasses.Length ; h++)
                {
                    // Draw HeightClass labels
                    y = labelHeight + h * cellHeight;
                    g.DrawString(heightClasses[h], font, brush, cellWidth / 2, y + cellHeight / 2 - 10);

                    // Draw counts in cells
                    for (int s = 0; s < sizeClasses.Length; s++)
                    {
                        int count = counts[h, s];
                        if (count > 0)
                        {
                            x = labelWidth + s * cellWidth;
                            y = labelHeight + h * cellHeight;
                            g.FillRectangle(percentIndicator(100*count/categorised), x, y, cellWidth, cellHeight);
                            g.DrawString(count.ToString(), font, brush, x + cellWidth / 2 - 10, y + cellHeight / 2 - 10);
                        }
                    }
                }

                // Draw grid lines
                DrawGridLines(g, sizeClasses.Length, heightClasses.Length, labelWidth, labelHeight, cellWidth, cellHeight, pen);


                // Draw totals for SizeClasses
                for (int s = 0; s < sizeClasses.Length; s++)
                {
                    total = sizeClassTotals[s].ToString();
                    x = labelWidth + s * cellWidth;
                    y = labelHeight + cellHeight * heightClasses.Length + cellHeight / 2 - 10; 
                    g.DrawString(total, font, brush, x + cellWidth / 2 - 10, y);
                }

                // Draw totals for HeightClasses
                for (h = 0; h < heightClasses.Length; h++)
                {
                    total = heightClassTotals[h].ToString();
                    x = labelWidth + cellWidth * sizeClasses.Length + 20;
                    y = labelHeight + h * cellHeight;
                    g.DrawString(total, font, brush, x, y + cellHeight / 2 - 10);
                }

                // Get overall total categorised
                total = categorised + " categorised, ";
                // Get uncategorised total
                total = total + (animals.Count - categorised) + " uncategorised";
            }

            return ( total, bmp );
        }

        // Returns the background colour of the rectangle by percent of animals found in it
        private static Brush percentIndicator(double percent)
        {
            Brush result = percent switch
            {
                >= 0 and < 3 => new SolidBrush(Color.Ivory),
                >= 3 and < 6 => new SolidBrush(Color.Cornsilk),
                >= 6 and < 9 => new SolidBrush(Color.Wheat),
                >= 9 and < 12 => new SolidBrush(Color.Tan),
                >= 12 and < 15 => new SolidBrush(Color.Peru),
                _ => new SolidBrush(Color.Brown)
            };
            return result;
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
    }
}

