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
        public static void Draw( DrawImageConfig config, YoloProcessAll yolodata, int thisBlockId, ref Image<Bgr, byte> outputImg)
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
                    var theBox = theObject.LastFeature.PixelBox;
                    var the_title = theObject.ClassName + "(" + theObject.ObjectId.ToString() + ")";

                    BoundingRectangle(config, ref outputImg, theBox, theColor, theThickness);
                    Text(ref outputImg, the_title, new Point(theBox.X + 6 * theThickness, theBox.Y), 0.5, DroneColors.ColorToBgr(theColor));
                }
            }
        }

    }
}
