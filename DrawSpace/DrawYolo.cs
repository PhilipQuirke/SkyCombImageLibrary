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
                    var the_title = theObject.ClassName + "(" + theObject.ObjectId.ToString() + ")";

                    // Draw hollow bounding box
                    BoundingRectangle(config, ref outputImg, theObjectBox, theColor, theThickness);

                    // Draw solid box and title text inside it
                    var theTitleBox = new Rectangle(theObjectBox.X, theObjectBox.Y - 10, theObjectBox.Width, 40);
                    BoundingRectangle(config, ref outputImg, theTitleBox, theColor, 0);
                    Text(ref outputImg, the_title, new Point(theTitleBox.X + 2, theObjectBox.Y + 2), 0.5, DroneColors.WhiteBgr);
                }
            }
        }

    }
}
