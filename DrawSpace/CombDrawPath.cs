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
    // Code to draw drone flight path with comb objects overlaid
    public class CombDrawPath : DroneDrawPath
    {
        // Pixels width when drawing feature locations as squares
        public int FeaturePixels = 4;

        // Pixels width when drawing object locations as squares
        public int ObjectPixels = 6;


        public ProcessDrawScope ProcessDrawScope { get; }
        // Scope of objects to draw. May be null.
        public ObjectDrawScope ObjectDrawScope { get; }
        // Scope of objects to draw. May be null.
        public CombObjList CombObjList { get; set; }
        // Object user has hovered mouse over. May be null.
        public CombObject? HoverObject { get; set; }


        public CombDrawPath(ProcessDrawScope processDrawScope, CombObjList combObjList, ObjectDrawScope objectScope) : base(processDrawScope, true)
        {
            ProcessDrawScope = processDrawScope;
            ObjectDrawScope = objectScope;
            CombObjList = combObjList;
            HoverObject = null;
        }


        // Draw object features as crosses (often orange)
        private void DrawObjectFeatures(CombObject thisObject, ref Image<Bgr, byte> image, CombFeatureTypeEnum type, Bgr color)
        {
            foreach (var thisFeature in thisObject.Features)
                if ((thisFeature.Value.LocationM != null) &&
                    (thisFeature.Value.Type == type) &&
                    (thisFeature.Value.BlockId <= DroneDrawScope.MaxFeatureBlockIdToDraw))
                    Draw.Cross(ref image,
                        DroneLocnMToPixelPoint(thisFeature.Value.LocationM),
                        color, NormalThickness, NormalThickness * FeaturePixels);
        }


        // Draw object as rectangle (often red)
        private void DrawObject(CombObject thisObject, ref Image<Bgr, byte> image, Bgr objectBgr)
        {
            Rectangle objectRect = DroneLocnMToPixelSquare(
                thisObject.LocationM, ObjectPixels);

            image.Draw(objectRect, objectBgr, NormalThickness);
        }


        // Draw drone flight path based on Drone/GroundSpace & RunSpace data.
        // By default we DONT show insignificant or out of scope objects.
        // For long flights, most objects will quickly become inactive, and we just have a few to draw.
        public override void CurrImage(ref Image<Bgr, byte> image)
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

                    bool showAllFeatures = (ProcessObject.Config.SaveObjectData == SaveObjectDataEnum.All);


                    // This of objects in process scope
                    var objList = CombObjList;

                    // Reduce list of objects by the user filter values.
                    if ((objList != null) && (ObjectDrawScope != null))
                    {
                        objList = CombObjList.FilterByObjectScope(ObjectDrawScope);
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
                                    DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Real, unrealBgr);
                        }

                        // Draw significant in-scope objects as red boxes with orange & yellow features
                        foreach (var thisObject in objList)
                            if (thisObject.Value.Significant &&
                                (thisObject.Value.LocationM != null) &&
                                thisObject.Value.InRunScope(ProcessDrawScope.ProcessScope))
                            {
                                // Draw least important then more important stuff as the rectangles will overlap.
                                if (showAllFeatures)
                                    DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Unreal, unrealBgr);
                                DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Real, realBgr);
                                DrawObject(thisObject.Value, ref image, inObjectBgr);
                            }
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombPath.CurrImage", ex);
            }
        }


        // Draw drone flight path and one object in particular.
        // Draw least important then more important stuff as the rectangles will overlap.
        public void CurrImage(ref Image<Bgr, byte> image, CombProcessAll process, ProcessObject focusObject)
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
                    foreach (var thisObject in process.CombObjs.CombObjList)
                        if (thisObject.Value.Significant &&
                            (thisObject.Value.LocationM != null) &&
                            thisObject.Value.InRunScope(ProcessDrawScope.ProcessScope))
                        {
                            bool isFocusObject = (focusObject.ObjectId == thisObject.Value.ObjectId);
                            if (isFocusObject)
                            {
                                // Draw least important then more important stuff as the rectangles will overlap.
                                DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Real, realBgr);
                                DrawObject(thisObject.Value, ref image, inObjectBgr);
                            }
                            else
                                DrawObject(thisObject.Value, ref image, outObjectBgr);
                        }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombPath.CurrImage", ex);
            }
        }


        // Generate a bitmap of the graph as per scope settings.
        public override Bitmap CurrBitmap()
        {
            var answer = base.CurrBitmap();

            // Draw the hover object details (if any) on the bitmap
            if (HoverObject != null)
            {
                var objName = HoverObject.Name;
                var objHeight = (HoverObject.HeightM <= ProcessObjectModel.UnknownHeight ? "N/A" : Math.Round(HoverObject.HeightM,1).ToString() + " m");
                var objSize = ((int)HoverObject.SizeCM2).ToString() + " cm2";
                var objHeat = HoverObject.MaxHeat.ToString();
                var objRange = HoverObject.AvgRangeM.ToString() + " m"; // Distance from drone to object
                var objFromS = VideoData.DurationSecToString(HoverObject.RunFromVideoS, 0);

                int vertStep = 20;
                int horizStep = 80;

                using (Graphics graphics = Graphics.FromImage(answer))
                {
                    // Define a font and brush for the text
                    Font font = new Font("Arial", 11);
                    SolidBrush brush = new SolidBrush(Color.White);

                    // Define the position where you want to draw the text
                    Rectangle objectRect = DroneLocnMToPixelSquare(
                        HoverObject.LocationM, ObjectPixels);
                    PointF leftPosition = new PointF(objectRect.Right, objectRect.Bottom);
                    PointF rightPosition = new PointF(objectRect.Right + horizStep, objectRect.Bottom);

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

