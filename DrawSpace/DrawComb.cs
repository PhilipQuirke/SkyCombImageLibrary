// Copyright SkyComb Limited 2023. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneLogic;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombGround.CommonSpace;
using System.Drawing;


namespace SkyCombImage.DrawSpace
{
    // Code to draw drone flight path with comb objects overlaid
    public class DrawCombFlightPath : DrawPath
    {
        // Pixels width when drawing feature locations as squares
        public int FeaturePixels = 4;

        // Pixels width when drawing object locations as squares
        public int ObjectPixels = 6;


        public DrawScope DrawScope { get; }


        public DrawCombFlightPath(DrawScope drawScope) : base(drawScope, true)
        {
            DrawScope = drawScope;
        }


        // Draw object features as crosses (often orange)
        private void DrawObjectFeatures(CombObject thisObject, ref Image<Bgr, byte> image, CombFeatureTypeEnum type, Bgr color)
        {
            foreach (var thisFeature in thisObject.Features)
                if ((thisFeature.Value.CFM.LocationM != null) &&
                    (thisFeature.Value.CFM.Type == type) &&
                    (thisFeature.Value.CFM.BlockId <= BaseDrawScope.MaxFeatureBlockIdToDraw))
                    Draw.Cross(ref image,
                        DroneLocnMToPixelPoint(thisFeature.Value.CFM.LocationM),
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

                if (HasPathGraphTransform() && (DrawScope.Process != null))
                {
                    var inObjectBgr = DroneColors.InScopeObjectBgr;   // Red
                    var outObjectBgr = DroneColors.OutScopeObjectBgr; // Gray
                    var realBgr = DroneColors.RealFeatureBgr;         // Orange
                    var unrealBgr = DroneColors.UnrealFeatureBgr;     // Yellow

                    bool showAllFeatures = (ProcessObject.Config.SaveObjectData == SaveObjectDataEnum.All);

                    if (showAllFeatures)
                    {
                        // Draw least important then more important stuff
                        // as the significant inscope stuff (draw later) will overdraw this.

                        // Draw significant but out-of-scope objects as grey squares
                        foreach (var thisObject in DrawScope.Process.CombObjs.CombObjList)
                            if (thisObject.Value.Significant &&
                                (thisObject.Value.LocationM != null) &&
                                !thisObject.Value.InRunScope(DrawScope.ProcessScope))
                                DrawObject(thisObject.Value, ref image, outObjectBgr);

                        // Draw insignificant object features as yellow squares
                        foreach (var thisObject in DrawScope.Process.CombObjs.CombObjList)
                            if (!thisObject.Value.Significant &&
                                (thisObject.Value.LocationM != null))
                                DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Real, unrealBgr);
                    }

                    // Draw significant in-scope objects as red boxes with orange & yellow features
                    foreach (var thisObject in DrawScope.Process.CombObjs.CombObjList)
                        if (thisObject.Value.Significant &&
                            (thisObject.Value.LocationM != null) &&
                            thisObject.Value.InRunScope(DrawScope.ProcessScope))
                        {
                            // Draw least important then more important stuff as the rectangles will overlap.
                            if (showAllFeatures)
                                DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Unreal, unrealBgr);
                            DrawObjectFeatures(thisObject.Value, ref image, CombFeatureTypeEnum.Real, realBgr);
                            DrawObject(thisObject.Value, ref image, inObjectBgr);
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
        public void CurrImage( ref Image<Bgr, byte> image, CombProcessAll process, ProcessObject focusObject)
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
                            thisObject.Value.InRunScope(DrawScope.ProcessScope))
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
    }


    // Code to draw images on video frames based on hot spot data
    public class DrawCombVideoFrames : Draw
    {
        // Draw the hot pixels
        public static void HotPixels(
            DrawImageConfig config, ProcessConfigModel processConfig,
            ref Image<Bgr, byte> image, CombFeature feature, Transform transform)
        {
            if (config.DrawPixelColor == Color.White)
                return;

            if (feature.Pixels == null)
                return;

            // Color the hotspots using 8 colors from yellow to red
            int numColors = 8;
            var theShades = GetColorShades(
                Color.FromArgb(255, 255, 255, 0),
                Color.FromArgb(255, 255, 0, 0), numColors);

            // Use a linear threshold from the processConfig.ThresholdValue to 255
            int thresholdStep = (255 - processConfig.ThresholdValue) / numColors;
            int[] threshold = new int[numColors];
            for (int i = 0; i < numColors; i++)
                threshold[i] = processConfig.ThresholdValue + i * thresholdStep;

            foreach (var pixel in feature.Pixels)
            {
                int num = 0;
                for (int i = 0; i < numColors; i++)
                    if (pixel.Heat >= threshold[i])
                        num = i;

                var x = transform.CalcX(pixel.X);
                var y = transform.CalcY(pixel.Y);

                for (int deltaX = 0; deltaX < transform.Scale; deltaX++)
                    for (int deltaY = 0; deltaY < transform.Scale; deltaY++)
                    {
                        image.Data[y + deltaY, x + deltaX, 0] = theShades[num].B;
                        image.Data[y + deltaY, x + deltaX, 1] = theShades[num].G;
                        image.Data[y + deltaY, x + deltaX, 2] = theShades[num].R;
                    }
            }
        }


        // Draw the bounding rectangles of the owned features
        public static void ObjectFeatures(DrawImageConfig config, int focusObjectId,
            ref Image<Bgr, byte> image,
            CombFeature feature, CombObject? combObject,
            Transform transform)
        {
            if (config.DrawRealFeatureColor == Color.White &&
                config.DrawUnrealFeatureColor == Color.White &&
                DroneColors.InScopeObjectColor == Color.White &&
                DroneColors.OutScopeObjectColor == Color.White)
                return;

            if (feature.ObjectId > 0)
            {
                var theColor = Color.White;
                if (focusObjectId > 0)
                    theColor = (feature.ObjectId == focusObjectId ? DroneColors.InScopeObjectColor : DroneColors.RealFeatureColor);
                else if (feature.CFM.Type == CombFeatureTypeEnum.Unreal)
                    theColor = config.DrawUnrealFeatureColor;
                else if (feature.CFM.Type == CombFeatureTypeEnum.Real)
                    theColor = feature.Significant ? DroneColors.InScopeObjectColor : config.DrawRealFeatureColor;

                if (theColor != Color.White)
                {
                    var isFocusObject = (focusObjectId == feature.ObjectId);
                    int thickness = (int)transform.Scale;
                    var scaledRect = transform.CalcRect(feature.CFM.PixelBox);

                    BoundingRectangle(config, ref image, scaledRect, theColor, thickness);

                    // Helps identify points visually on image to facilitate mapping to xls data.
                    // Combined with a very small Video time span to process, can be very useful.
                    if (feature.Significant || isFocusObject ||
                        (focusObjectId == -1)) // Draw object # for all objects if focusObjectId is -1
                    {
                        string name;
                        if (combObject != null)
                            name = combObject.Name;
                        else
                            name = feature.ObjectId.ToString();

                        // Draw the object name to right of the rectangle.
                        image.Draw(name,
                            new Point(scaledRect.X + scaledRect.Width + 3, scaledRect.Y + 8),
                            FontFace.HersheyPlain, transform.Scale, DroneColors.ColorToBgr(theColor), thickness);
                    }
                }
            }
        }


        // Draw all hot pixels for current block, bounding rectangles of the owned features
        public static void CombImage(
            DrawImageConfig drawConfig, ProcessConfigModel processConfig,
            int focusObjectId, ref Image<Bgr, byte> outputImg,
            CombProcessAll process, ProcessBlockModel block, Transform transform)
        {
            try
            {
                bool thermalImage = (transform.XMargin == 0);

                if (process != null)
                {
                    // Draw the leg name on the image (if any) at bottom right
                    if (thermalImage && (block.FlightLegId > 0))
                    {
                        var video = process.Drone.InputVideo;
                        int theY = (int)(video.ImageHeight * 98 / 100); // pixels
                        int theX = (int)(video.ImageWidth * 92 / 100); // pixels
                        var fontScale = video.FontScale;
                        Text(ref outputImg, "Leg " + block.FlightLegName,
                                new Point(theX, theY), fontScale / 2.0f, DroneColors.LegNameBgr, fontScale);
                    }

                    for (int featureId = block.MinFeatureId; featureId <= block.MaxFeatureId; featureId++)
                    {
                        if (process.CombFeatures.ContainsKey(featureId))
                        {
                            var feature = process.CombFeatures[featureId];
                            Assert(feature.CFM.BlockId == block.BlockId, "CombImage: Bad logic");

                            // Draw all hot pixels for the current block 
                            HotPixels(drawConfig, processConfig, ref outputImg, feature, transform);

                            // Draw the bounding rectangle of the owned feature
                            CombObject? theObject = null;
                            if (feature.ObjectId > 0)
                                theObject = process.CombObjs.CombObjList[feature.ObjectId];
                            ObjectFeatures(drawConfig, focusObjectId, ref outputImg, feature, theObject, transform);
                        }
                    }
                }

                if (!thermalImage)
                {
                    int thickness = 4;
                    var activeBgr = DroneColors.ActiveDroneBgr;

                    // We are transforming input (thermal) data to display on this display (optical) video.
                    // Show the portion of this video covered by the transformed input data.
                    outputImg.Draw(
                        new Rectangle(
                            (int)transform.XMargin,
                            (int)transform.YMargin,
                            (int)(outputImg.Width - 2 * transform.XMargin),
                            (int)(outputImg.Height - 2 * transform.YMargin)),
                        activeBgr, thickness);

                    Cross(ref outputImg,
                        new Point(outputImg.Width / 2, outputImg.Height / 2),
                        activeBgr, thickness, thickness * 4);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombVideoFrames.CombImage", ex);
            }
        }


        // Process a single input and (maybe) display video frame for the specified block, returning the modified input&display frames to show 
        public static (Image<Bgr, byte>?, Image<Bgr, byte>?) Draw(
            RunProcessEnum runProcess,
            ProcessConfigModel processConfig, DrawImageConfig drawConfig, Drone drone,
            ProcessBlockModel block, CombProcessAll combProcess, int focusObjectId,
            Image<Bgr, byte> inputFrame, Image<Bgr, byte> displayFrame)
        {
            try
            {
                Image<Bgr, byte>? modifiedInputFrame = null;
                Image<Bgr, byte>? modifiedDisplayFrame = null;

                if (inputFrame != null)
                {
                    modifiedInputFrame = inputFrame.Clone();
                    DrawImage.Palette(drawConfig, ref modifiedInputFrame);
                    if (runProcess == RunProcessEnum.Comb)
                        // Draw hot objects
                        CombImage(drawConfig, processConfig, focusObjectId,
                            ref modifiedInputFrame, combProcess, block, new());
                    else
                        // Handles RunModel = Contour, GFTT, etc.
                        DrawImage.Draw(runProcess, processConfig, drawConfig, ref modifiedInputFrame);

                    if (displayFrame != null)
                    {
                        modifiedDisplayFrame = displayFrame.Clone();

                        // For the display video, we don't show the pixels or real/unreal features. We just show objects.
                        var newDrawConfig = drawConfig.Clone();
                        newDrawConfig.DrawPixelColor = Color.White;
                        newDrawConfig.DrawRealFeatureColor = Color.White;
                        newDrawConfig.DrawUnrealFeatureColor = Color.White;

                        // The display frame may be a different size (aka resolution) from the input frame. Calc the transform.
                        // Optical video can cover a wider field of vision than thermal.
                        // ExcludeMarginRatio is the (unitless) margin on the optical video, not visible in the thermal video, on all optical video edges
                        var inputToDisplayTransform = Transform.ImageToImageTransform(
                            inputFrame.Size, displayFrame.Size, drone.ExcludeDisplayMarginRatio);

                        CombImage(newDrawConfig, processConfig, focusObjectId,
                            ref modifiedDisplayFrame, combProcess, block, inputToDisplayTransform);
                    }
                }
                return (modifiedInputFrame, modifiedDisplayFrame);
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombVideoFrames.DrawVideoFrames", ex);
            }
        }
    }


    // Code to draw ground tree-top & drone alitudes against lineal meters, with comb objects overlaid
    public class DrawCombAltitudeByLinealM : DrawAltitudeByLinealM
    {
        CombProcessAll Process { get; }
        DrawScope DrawScope { get; }


        public DrawCombAltitudeByLinealM(CombProcessAll process, DrawScope drawScope) : base(drawScope, false)
        {
            Process = process;
            DrawScope = drawScope;
            Description +=
                "Object location and height and related error bars (in meters) are shown.";
        }


        // Draw altitude data dependant on Drone/GroundSpace data
        public void Initialise(CombProcessAll process, Size size)
        {
            try
            {
                Initialise(size);

                if (DroneDrawScope.Drone == null)
                    return;

                // Draw significant object as horizontally-stretched H with centroid - showing object duration and height
                if (process != null)
                    GraphObjects(ref BaseImage, process);
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombAltitudeByLinealM.Initialise", ex);
            }
        }


        // Draw object at best estimate of height, location with error bars
        // Draw object "location error" as horizontally-stretched H
        // Draw object "height error" as vertically-stretched H
        public void GraphObjects(ref Image<Bgr, byte> currImage, CombProcessAll process)
        {
            try
            {
                if ((DroneDrawScope.Drone != null) && (process != null) && (process.CombObjs.CombObjList.Count > 0))
                {
                    foreach (var thisObject in process.CombObjs.CombObjList)
                        if (thisObject.Value.Significant)
                        {
                            var avgHeight = TrimHeight(RawDataToHeightPixels(thisObject.Value.DemM + thisObject.Value.HeightM - MinVertRaw, VertRangeRaw));
                            var minHeight = TrimHeight(RawDataToHeightPixels(thisObject.Value.DemM + thisObject.Value.MinHeightM - MinVertRaw, VertRangeRaw));
                            var maxHeight = TrimHeight(RawDataToHeightPixels(thisObject.Value.DemM + thisObject.Value.MaxHeightM - MinVertRaw, VertRangeRaw));

                            var avgLinealM = thisObject.Value.AvgSumLinealM;
                            var errLinealM = thisObject.Value.LocationErrM;
                            int firstWidth = StepToWidth(avgLinealM - errLinealM);
                            int middleWidth = StepToWidth(avgLinealM);
                            int lastWidth = StepToWidth(avgLinealM + errLinealM);

                            var theBgr = DroneColors.OutScopeObjectBgr;
                            if (thisObject.Value.InRunScope(DrawScope.ProcessScope))
                                theBgr = DroneColors.InScopeObjectBgr;

                            DrawObject(ref currImage, theBgr,
                                minHeight, avgHeight, maxHeight,
                                firstWidth, middleWidth, lastWidth);
                        }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombAltitudeByLinealM.GraphObjects", ex);
            }
        }


        // Draw altitude data based on Drone/GroundSpace & RunSpace data
        public override void CurrImage(ref Image<Bgr, byte> image)
        {
            try
            {
                // Draw significant object as horizontally-stretched H with centroid - showing object duration and height
                if (Process != null)
                    GraphObjects(ref image, Process);
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombAltitudeByLinealM.CurrImage", ex);
            }
        }
    }



    // Code to draw ground tree-top & drone altitudes against time, with comb objects overlaid
    public class DrawCombAltitudeByTime : DrawAltitudeByTime
    {
        CombProcessAll Process { get; }
        DrawScope DrawScope { get; }


        public DrawCombAltitudeByTime(CombProcessAll process, DrawScope drawScope) : base(drawScope, false)
        {
            Process = process;
            DrawScope = drawScope;
            Description +=
                "Object location, height, height error bar, and duration seen are shown.";
        }


        // Draw altitude data dependant on Drone/GroundSpace data (but not RunSpace data)
        public void Initialise(CombProcessAll process, Size size)
        {
            try
            {
                Initialise(size);

                if (DroneDrawScope.Drone == null)
                    return;

                // Draw significant object as horizontally-stretched H with centroid - showing object duration and height
                if (process != null)
                {
                    GraphObjects(ref BaseImage, process);
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombAltitudeByTime.Initialise", ex);
            }
        }


        // Draw object at best estimate of height and center of period seen
        // Draw object "visible duration" as horizontally-stretched H
        // Draw object "height error" as vertically-stretched H
        public void GraphObjects(ref Image<Bgr, byte> currImage, CombProcessAll process)
        {
            try
            {
                if ((DroneDrawScope.Drone != null) && (process != null) && (process.CombObjs.CombObjList.Count > 0))
                {
                    foreach (var thisObject in process.CombObjs.CombObjList)
                        if (thisObject.Value.Significant)
                        {
                            var avgHeight = TrimHeight(RawDataToHeightPixels(thisObject.Value.DemM + thisObject.Value.HeightM - MinVertRaw, VertRangeRaw));
                            var minHeight = TrimHeight(RawDataToHeightPixels(thisObject.Value.DemM + thisObject.Value.MinHeightM - MinVertRaw, VertRangeRaw));
                            var maxHeight = TrimHeight(RawDataToHeightPixels(thisObject.Value.DemM + thisObject.Value.MaxHeightM - MinVertRaw, VertRangeRaw));

                            var firstFlightMs = (int)(thisObject.Value.RunFromVideoS * 1000);
                            var lastFlightMs = (int)(thisObject.Value.RunToVideoS * 1000);

                            var firstFlightStep = DroneDrawScope.Drone.MsToNearestFlightStep(firstFlightMs);
                            var lastFlightStep = DroneDrawScope.Drone.MsToNearestFlightStep(lastFlightMs);

                            if ((firstFlightStep != null) && (lastFlightStep != null))
                            {
                                var theBgr = DroneColors.OutScopeObjectBgr;
                                if (thisObject.Value.InRunScope(DrawScope.ProcessScope))
                                    theBgr = DroneColors.InScopeObjectBgr;

                                int firstWidth = StepToWidthBySection(firstFlightStep.FlightSection.TardisId);
                                int lastWidth = StepToWidthBySection(lastFlightStep.FlightSection.TardisId);
                                int middleWidth = (firstWidth + lastWidth) / 2;

                                if (DroneDrawScope.DrawFullFlight())
                                {
                                    Assert(!(firstWidth > Size.Width + 1 || firstWidth < 0), "DrawCombAltitudeByTime.GraphObjects: firstWidth out of bounds");
                                    Assert(!(lastWidth > Size.Width + 1 || lastWidth < 0), "DrawCombAltitudeByTime.GraphObjects: lastWidth out of bounds");
                                    Assert(!(middleWidth > Size.Width + 1 || middleWidth < 0), "DrawCombAltitudeByTime.GraphObjects: middleWidth out of bounds");
                                }

                                DrawObject(ref currImage, theBgr,
                                    minHeight, avgHeight, maxHeight,
                                    firstWidth, middleWidth, lastWidth);
                            }
                        }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombAltitudeByTime.GraphObjects", ex);
            }
        }


        // Draw altitude data based on Drone/GroundSpace & RunSpace data
        public override void CurrImage(ref Image<Bgr, byte> image)
        {
            try
            {
                // Draw significant object as horizontally-stretched H with centroid - showing object duration and height
                if (Process != null)
                    GraphObjects(ref image, Process);
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombAltitudeByTime.CurrImage", ex);
            }
        }
    }


    // Code to draw comb object height - including feature estimates
    public class DrawCombObjectHeight : DrawAltitudeByTime
    {
        public DrawCombObjectHeight(DrawScope drawScope) : base(drawScope, false)
        {
            Description +=
                "Object & object feature height are shown.";
        }


        public void Initialise(CombProcessAll process, Size size, ProcessObject focusObject)
        {
            try
            {
                var image = Draw.NewLightGrayImage(size);

                if (DroneDrawScope.Drone == null)
                {
                    Title = "Drone Height";
                    DrawNoData(ref image);
                }
                else
                {
                    MinVertRaw = (float)Math.Floor(focusObject.MinHeightM);
                    MaxVertRaw = (float)Math.Ceiling(focusObject.MaxHeightM);

                    DrawAxisesAndLabels(ref image);
                    SetHorizLabelsByTime();
                    SetVerticalLabels("m", "0");

                    CalculateStepWidthAndStride(DroneDrawScope.FirstDrawMs, DroneDrawScope.LastDrawMs);

                    BaseImage = image.Clone();
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombObjectHeight.Initialise", ex);
            }
        }


        public void CurrImage(ref Image<Bgr, byte> image, CombProcessAll process, CombObject thisObject)
        {
            try
            {
                var inObjectBgr = DroneColors.InScopeObjectBgr;   // Red
                var realBgr = DroneColors.RealFeatureBgr;         // Orange
                var unrealBgr = DroneColors.UnrealFeatureBgr;     // Yellow

                var objHeightPxs = TrimHeight(RawDataToHeightPixels(thisObject.HeightM - MinVertRaw, VertRangeRaw));

                var firstMs = DroneDrawScope.FirstDrawMs;
                var lastMs = DroneDrawScope.LastDrawMs;
                var objWidthPxs = StepToWidth(firstMs + (lastMs - firstMs) * 0.9f, firstMs);

                // Draw the object average height as red cross at right edge
                // (as calculation is based on last 8 calculations)
                Draw.Cross(ref image,
                    new Point(objWidthPxs, objHeightPxs),
                    inObjectBgr, NormalThickness, NormalThickness * 10);

                // Draw the object features as orange or yellow crosses
                foreach (var thisFeature in thisObject.Features)
                    if ((thisFeature.Value.CFM.HeightM != UnknownValue) &&
                        (thisFeature.Value.CFM.BlockId <= DroneDrawScope.MaxFeatureBlockIdToDraw))
                    {
                        var theBgr = (thisFeature.Value.CFM.Type == CombFeatureTypeEnum.Real ? realBgr : unrealBgr);
                        var thisHeightPxs = TrimHeight(RawDataToHeightPixels(thisFeature.Value.CFM.HeightM - MinVertRaw, VertRangeRaw));
                        var thisWidthPxs = StepToWidth(thisFeature.Value.Block.SumTimeMs, firstMs);

                        Draw.Cross(ref image,
                            new Point(thisWidthPxs, thisHeightPxs),
                            theBgr, NormalThickness, NormalThickness * 4);
                    }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawCombObjectHeight.CurrImage", ex);
            }
        }
    }


}

