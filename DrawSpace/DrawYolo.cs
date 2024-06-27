// Copyright SkyComb Limited 2023. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.DrawSpace
{
    // Code to draw stuff on an image for the yolo model process
    public class DrawYolo : Draw
    {
        // Draw the yolo objects 
        public static void Draw( DrawImageConfig config, YoloProcess yolodata, int thisBlockId, ref Image<Bgr, byte> outputImg)
        {
            // Thickness of lines and circles.
            int theThickness = 1;
            if (outputImg.Width > 1000)
                theThickness = 2;

            foreach (var theObject in yolodata.YoloObjects)
            {
                if ((theObject.LastFeature != null) && (theObject.LastFeature.BlockId == thisBlockId))
                {
                    var theColor = theObject.ClassColor;
                    var theObjectBox = theObject.LastFeature.PixelBox;
                    var the_title = "#" + theObject.ObjectId.ToString();

                    // Draw hollow bounding box
                    BoundingRectangle(config, ref outputImg, theObjectBox, theColor, theThickness);

                    // Draw the title text 
                    var theTitlePt = new Point(theObjectBox.X, theObjectBox.Y - 10);
                    Text(ref outputImg, the_title, theTitlePt, 0.5, DroneColors.ColorToBgr(theColor));
                }
            }
        }

    }
}
