// Copyright SkyComb Limited 2024. All rights reserved.
using Emgu.CV;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using System.Drawing;


namespace SkyCombImage.DrawSpace
{
    // Code to draw drone flight path with process objects overlaid
    public class ProcessDrawPath : DroneDrawPath
    {
        // Pixels width when drawing feature locations as squares
        public int FeaturePixels = 4;

        // Pixels width when drawing object locations as squares
        public int ObjectPixels = 6;


        public ProcessDrawScope ProcessDrawScope { get; }
        // Scope of objects to draw. May be null.
        public ObjectDrawScope ObjectDrawScope { get; }
        // Scope of objects to draw. May be null.
        public ProcessObjList? ProcessObjList { get; set; }
        // Object user has hovered mouse over. May be null.
        public ProcessObject? HoverObject { get; set; }


        public ProcessDrawPath(ProcessDrawScope processDrawScope, ProcessObjList? processObjList, ObjectDrawScope objectScope) : base(processDrawScope, true)
        {
            ProcessDrawScope = processDrawScope;
            ObjectDrawScope = objectScope;
            ProcessObjList = processObjList;
            HoverObject = null;
        }


        // Draw object features as crosses (often orange) on the Location range graph //NQ
        private void DrawObjectFeatures(ProcessObject thisObject, ref Image<Bgr, byte> image, FeatureTypeEnum type, Bgr color)
        {
            Bgr highlight = new Bgr(170, 205, 102);
            foreach (var thisFeature in thisObject.ProcessFeatures)
            {
                if ((thisFeature.Value.LocationM != null) &&
                    (thisFeature.Value.Type == type) &&
                    (thisFeature.Value.BlockId < DroneDrawScope.MaxFeatureBlockIdToDraw))
                    Draw.Cross(ref image,
                        DroneLocnMToPixelPoint(thisFeature.Value.LocationM),
                        color, NormalThickness, NormalThickness * FeaturePixels);
                if ((thisFeature.Value.LocationM != null) &&
                    (thisFeature.Value.Type == type) &&
                    (thisFeature.Value.BlockId == DroneDrawScope.MaxFeatureBlockIdToDraw))
                    Draw.Cross(ref image,
                        DroneLocnMToPixelPoint(thisFeature.Value.LocationM),
                        highlight, NormalThickness * 2, NormalThickness * FeaturePixels);
            }
        }


        // Draw object as rectangle (often red)
        private void DrawObject(ProcessObject thisObject, ref Image<Bgr, byte> image, Bgr objectBgr)
        {
            DrawSquare(thisObject.LocationM, ObjectPixels, ref image, objectBgr);
        }


        // Draw drone flight path based on Drone/GroundSpace & RunSpace data.
        // By default we DONT show insignificant or out of scope objects.
        // For long flights, most objects will quickly become inactive, and we just have a few to draw.
        public override void CurrImage(ref Image<Bgr, byte> image, List<Image>? sizeImages = null)
        {
            try
            {
                base.CurrImage(ref image);

                if (HasPathGraphTransform() && (ProcessDrawScope.Process != null))
                {
                    var inObjectBgr = DroneColors.InScopeObjectBgr;   // Red
                    var outObjectBgr = DroneColors.OutScopeObjectBgr; // Gray
                    var realBgr = DroneColors.RealFeatureBgr;         // Orange
                    var unrealBgr = DroneColors.UnrealFeatureBgr;     // Yellow

                    var processConfig = ProcessDrawScope.Process.ProcessConfig;
                    bool showAllFeatures = (processConfig != null &&
                        (processConfig.SaveObjectData == SaveObjectDataEnum.All));


                    // This of objects in process scope
                    var objList = ProcessObjList;

                    // Reduce list of objects by the user filter values.
                    if ((objList != null) && (ObjectDrawScope != null))
                    {
                        objList = ProcessObjList.FilterByObjectScope(null, objList);
                        ObjectDrawScope.NumFilteredObjects = objList.Count;
                    }

                    if (objList != null)
                    {
                        if (showAllFeatures)
                        {
                            // Draw least important then more important stuff
                            // as the significant inscope stuff (draw later) will overdraw this.

                            // Draw significant but out-of-scope objects as grey squares
                            foreach (var thisObject in objList)
                                if (thisObject.Value.Significant &&
                                    (thisObject.Value.LocationM != null) &&
                                    !thisObject.Value.InRunScope(ProcessDrawScope.ProcessScope))
                                    DrawObject(thisObject.Value, ref image, outObjectBgr);

                            // Draw insignificant object features as yellow squares
                            foreach (var thisObject in objList)
                                if (!thisObject.Value.Significant &&
                                    (thisObject.Value.LocationM != null))
                                    DrawObjectFeatures(thisObject.Value, ref image, FeatureTypeEnum.Real, unrealBgr);
                        }

                        // Draw significant in-scope objects as red boxes with orange & yellow features
                        foreach (var thisObject in objList)
                        {
                            var processObject = thisObject.Value;
                            if (processObject.Significant &&
                                (processObject.LocationM != null) &&
                                processObject.InRunScope(ProcessDrawScope.ProcessScope))
                            {
                                // Draw least important then more important stuff as the rectangles will overlap.
                                if (showAllFeatures)
                                    DrawObjectFeatures(processObject, ref image, FeatureTypeEnum.Unreal, unrealBgr);
                                DrawObjectFeatures(processObject, ref image, FeatureTypeEnum.Real, realBgr);
                                DrawObject(processObject, ref image, inObjectBgr);
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessDrawPath.CurrImage", ex);
            }
        }


        // Draw drone flight path and one object in particular.
        // Draw least important then more important stuff as the rectangles will overlap.
        public void CurrImage(ref Image<Bgr, byte> image, ProcessAll process, ProcessObject focusObject)
        {
            try
            {
                if (HasPathGraphTransform() && (process != null))
                {
                    var inObjectBgr = DroneColors.InScopeObjectBgr;   // Red
                    var outObjectBgr = DroneColors.OutScopeObjectBgr; // Gray
                    var realBgr = DroneColors.RealFeatureBgr;         // Orange
                    var unrealBgr = DroneColors.UnrealFeatureBgr;     // Yellow

                    // Draw significant in-scope objects as red boxes with orange & yellow features
                    foreach (var thisObject in process.ProcessObjects)
                    {
                        var processObject = thisObject.Value;
                        if (processObject.Significant &&
                            (processObject.LocationM != null) &&
                            processObject.InRunScope(ProcessDrawScope.ProcessScope))
                        {
                            bool isFocusObject = (focusObject.ObjectId == processObject.ObjectId);
                            if (isFocusObject)
                            {
                                // Draw least important then more important stuff as the rectangles will overlap.
                                DrawObjectFeatures(processObject, ref image, FeatureTypeEnum.Real, realBgr);
                                DrawObject(processObject, ref image, inObjectBgr);
                            }
                            else
                                DrawObject(processObject, ref image, outObjectBgr);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("ProcessDrawPath.CurrImage", ex);
            }
        }


        public override Bitmap CurrBitmap(bool dpiIndependent = false, List<Image>? sizeImages = null)
        {
            var answer = base.CurrBitmap();

            // Draw the hover object details (if any) on the bitmap
            if (HoverObject != null)
            {
                var objName = HoverObject.Name;
                var objHeight = (HoverObject.HeightM <= ProcessObjectModel.UnknownHeight ? "N/A" : Math.Round(HoverObject.HeightM, 1).ToString() + " m");
                var objSize = ((int)HoverObject.SizeCM2).ToString() + " cm2";
                var objHeat = HoverObject.MaxHeat.ToString();
                var objRange = HoverObject.AvgRangeM.ToString() + " m"; // Distance from drone to object
                var objFromS = VideoData.DurationSecToString(HoverObject.RunFromVideoS, 0);

                int vertStep = 20;
                int horizStep = 80;

                // Calculate total text height and width
                int numLines = 6; // Object, Height, Size, Heat, Range, At
                int totalTextHeight = numLines * vertStep;
                int totalTextWidth = horizStep + 120; // Approximate width of data text

                using (Graphics graphics = Graphics.FromImage(answer))
                {
                    // Define a font and brush for the text
                    Font font = new Font("Arial", 11);
                    SolidBrush brush = new SolidBrush(Color.White);

                    // Define the position where you want to draw the text
                    Rectangle objectRect = DroneLocnMToPixelSquare(
                        HoverObject.LocationM, ObjectPixels);

                    // Start positions (default to right of the object)
                    PointF leftPosition = new PointF(objectRect.Right, objectRect.Bottom);

                    // Check if text would extend beyond right edge
                    if (leftPosition.X + totalTextWidth > answer.Width)
                    {
                        // Move text to the left of the object
                        leftPosition.X = Math.Max(10, objectRect.Left - totalTextWidth);
                    }

                    // Check if text would extend beyond bottom edge
                    if (leftPosition.Y + totalTextHeight > answer.Height)
                    {
                        // Move text above the object
                        leftPosition.Y = Math.Max(10, objectRect.Top - totalTextHeight);
                    }

                    PointF rightPosition = new PointF(leftPosition.X + horizStep, leftPosition.Y);

                    // Create a semi-transparent background for the text
                    Rectangle textBackgroundRect = new Rectangle(
                        (int)leftPosition.X - 10,
                        (int)leftPosition.Y - 5,
                        totalTextWidth + 5,
                        totalTextHeight + 10
                    );

                    // Draw semi-transparent black background
                    using (SolidBrush backgroundBrush = new SolidBrush(Color.FromArgb(180, 0, 0, 0)))
                    {
                        graphics.FillRectangle(backgroundBrush, textBackgroundRect);
                    }

                    // Draw a border around the text background for better definition
                    using (Pen borderPen = new Pen(Color.FromArgb(220, 255, 255, 255), 1))
                    {
                        graphics.DrawRectangle(borderPen, textBackgroundRect);
                    }

                    // Draw the text titles on the bitmap
                    graphics.DrawString("Object", font, brush, leftPosition); leftPosition.Y += vertStep;
                    graphics.DrawString("Height", font, brush, leftPosition); leftPosition.Y += vertStep;
                    graphics.DrawString("Size", font, brush, leftPosition); leftPosition.Y += vertStep;
                    graphics.DrawString("Heat", font, brush, leftPosition); leftPosition.Y += vertStep;
                    graphics.DrawString("Range", font, brush, leftPosition); leftPosition.Y += vertStep;
                    graphics.DrawString("At", font, brush, leftPosition); leftPosition.Y += vertStep;

                    objFromS = objFromS.Replace(":", "m") + "s";

                    // Draw the text data on the bitmap
                    graphics.DrawString(objName, font, brush, rightPosition); rightPosition.Y += vertStep;
                    graphics.DrawString(objHeight, font, brush, rightPosition); rightPosition.Y += vertStep;
                    graphics.DrawString(objSize, font, brush, rightPosition); rightPosition.Y += vertStep;
                    graphics.DrawString(objHeat, font, brush, rightPosition); rightPosition.Y += vertStep;
                    graphics.DrawString(objRange, font, brush, rightPosition); rightPosition.Y += vertStep;
                    graphics.DrawString(objFromS, font, brush, rightPosition); rightPosition.Y += vertStep;
                }
            }

            return answer;
        }
    }
}

