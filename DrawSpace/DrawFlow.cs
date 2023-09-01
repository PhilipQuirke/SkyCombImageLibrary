// Copyright SkyComb Limited 2023. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessModel;
using System.Drawing;
using SkyCombGround.CommonSpace;


namespace SkyCombImage.DrawSpace
{
    // Code to draw stuff on an image for the flow model process
    public class DrawFlow : Draw
    {
        // Show the point of view (aka POV and ground) velocity as:
        // - an arrow direction, and 
        // - the speed as text at location "center"
        // This velocity is "relative" to the POV - not "absolute" to say compass North. 
        public static void PointOfViewVelocity(ref Image<Bgr, byte> image, Point center, FlowBlock thisBlock, Bgr color)
        {
            try
            {
                bool BlockVelocityKnown;
                VelocityF BlockSpeed;
                string speedUnit;
                (BlockVelocityKnown, BlockSpeed, speedUnit) = thisBlock.GetFlowGroundVelocity();

                if (!BlockVelocityKnown)
                    return;

                int shiftTextLocationY = 15;

                // If velocity is less than 1/10th of a pixel then dont show an arrow
                if (Math.Abs(BlockSpeed.Value.X) > 0.1 || Math.Abs(BlockSpeed.Value.Y) > 0.1)
                {
                    var radians = Math.Atan2(BlockSpeed.Value.Y, BlockSpeed.Value.X);

                    var to = new PointF(center.X + (float)Math.Cos(radians) * 20, center.Y + (float)Math.Sin(radians) * 20);
                    var left = new PointF(center.X + (float)Math.Cos(radians + 0.2) * 16, center.Y + (float)Math.Sin(radians + 0.2) * 16);
                    var right = new PointF(center.X + (float)Math.Cos(radians - 0.2) * 16, center.Y + (float)Math.Sin(radians - 0.2) * 16);

                    Line(ref image, center, to, color, 1);
                    Line(ref image, left, to, color, 1);
                    Line(ref image, right, to, color, 1);

                    if (to.Y > center.Y)
                        shiftTextLocationY = -5;
                }

                // Draw the speed as text 
                var speed = BlockSpeed.Speed();
                Text(ref image,
                    string.Format("{0} {1}", speed.ToString("0.00"), speedUnit),
                    new Point(center.X - 20, center.Y + shiftTextLocationY), 0.5, DroneColors.WhiteBgr);
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawSpace.Draw.PointOfViewVelocity", ex);
            }
        }


        // Draw the optical flow. Each feature is a circle with a tail
        public static void Draw(FlowProcessAll FlowData, int lastBlockId, ref Image<Bgr, byte> outputImg, int maxObjsToDraw = -1)
        {
            bool isCombProcessDrawing = (maxObjsToDraw > 0);

            // Even if feature has been in the video image for 60 frames, we only draw a 20 block feature tail
            const int TailBlocks = 20;
            if (lastBlockId <= 1)
                return;
            int firstBlockId = Math.Max(1, lastBlockId - TailBlocks);

            // Thickness of lines and circles.
            int theThickness = 1;
            if (outputImg.Width > 1000)
                theThickness = 2;

            // For speed, find index of first Feature to (later) consider drawing
            int firstFeatureIndex = UnknownValue;
            foreach (var theFeature in FlowData.FlowFeatures)
                if (theFeature.BlockId >= firstBlockId)
                {
                    firstFeatureIndex = theFeature.FeatureId;
                    break;
                }
            if (firstFeatureIndex == UnknownValue)
                return;

            int objectsDrawn = 0;
            foreach (var theObject in FlowData.FlowObjects)
            {
                if ((theObject.LastFeature != null) && (theObject.LastFeature.BlockId >= lastBlockId))
                {
                    objectsDrawn++;
                    if (isCombProcessDrawing && (objectsDrawn > maxObjsToDraw))
                        break;

                    var theColor = theObject.Color; // In Flow process, each feature is drawn in its own color
                    if (isCombProcessDrawing)
                        theColor = DroneColors.GreenBgr; // In Comb process, all flow features are drawn the same color

                    int prevX = UnknownValue;
                    int prevY = UnknownValue;

                    if ((theObject.ObjectId == 71) && (lastBlockId >= 20))
                        prevX = UnknownValue;

                    for (int i = firstFeatureIndex; i < FlowData.FlowFeatures.Count; i++)
                    {
                        var theFeature = FlowData.FlowFeatures[i];
                        if (theFeature.ObjectId == theObject.ObjectId)
                        {
                            var thisX = (int)theFeature.Location.X;
                            var thisY = (int)theFeature.Location.Y;

                            if (prevX != UnknownValue)
                                Line(ref outputImg, new Point(prevX, prevY), new Point(thisX, thisY), theColor, theThickness);

                            if (theFeature.BlockId == lastBlockId)
                            {
                                Circle(ref outputImg, new Point(thisX, thisY), theColor, theThickness);
                                if (!isCombProcessDrawing)
                                    Text(ref outputImg, theObject.ObjectId.ToString(), new Point(thisX + 6 * theThickness, thisY), theThickness, theColor);
                                break;
                            }

                            prevX = thisX;
                            prevY = thisY;
                        }
                    }
                }
            }

            // Draw the estimated ground velocity (aka opposite of drone velocity) as arrow and text 
            PointOfViewVelocity(ref outputImg, new Point(40, 40), FlowData.FlowBlocks[lastBlockId] as FlowBlock, DroneColors.WhiteBgr);
        }

    }
}
