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
        public static void Draw(YoloProcessAll yolodata, int lastBlockId, ref Image<Bgr, byte> outputImg, int maxObjsToDraw = -1)
        {
            // Even if feature has been in the video image for 60 frames, we only draw a 20 block feature tail
            const int TailBlocks = 1;
            if (lastBlockId <= 1)
                return;
            int firstBlockId = Math.Max(1, lastBlockId - TailBlocks);

            // Thickness of lines and circles.
            int theThickness = 1;
            if (outputImg.Width > 1000)
                theThickness = 2;

            // For speed, find index of first Feature to (later) consider drawing
            int firstFeatureIndex = UnknownValue;
            foreach (var theFeature in yolodata.YoloFeatures)
                if (theFeature.BlockId >= firstBlockId)
                {
                    firstFeatureIndex = theFeature.FeatureId;
                    break;
                }
            if (firstFeatureIndex == UnknownValue)
                return;

            int objectsDrawn = 0;
            foreach (var theObject in yolodata.YoloObjects)
            {
                if ((theObject.LastFeature != null) && (theObject.LastFeature.BlockId >= lastBlockId))
                {
                    objectsDrawn++;

                    var theColor = DroneColors.ColorToBgr(theObject.ClassColor); 
                    int prevX = UnknownValue;
                    int prevY = UnknownValue;

                    for (int i = firstFeatureIndex; i < yolodata.YoloFeatures.Count; i++)
                    {
                        var theFeature = yolodata.YoloFeatures[i];
                        if (theFeature.ObjectId == theObject.ObjectId)
                        {
                            var thisX = (int)theFeature.LocationM.EastingM;
                            var thisY = (int)theFeature.LocationM.NorthingM;

                            if (prevX != UnknownValue)
                                Line(ref outputImg, new Point(prevX, prevY), new Point(thisX, thisY), theColor, theThickness);

                            if (theFeature.BlockId == lastBlockId)
                            {
                                Circle(ref outputImg, new Point(thisX, thisY), theColor, theThickness);
                                Text(ref outputImg, theObject.ObjectId.ToString(), new Point(thisX + 6 * theThickness, thisY), 0.5, theColor);
                                break;
                            }

                            prevX = thisX;
                            prevY = thisY;
                        }
                    }
                }
            }

            // Draw the estimated ground velocity (aka opposite of drone velocity) as arrow and text 
            //PointOfViewVelocity(ref outputImg, new Point(40, 40), FlowData.FlowBlocks[lastBlockId] as FlowBlock, DroneColors.WhiteBgr);
        }

    }
}
