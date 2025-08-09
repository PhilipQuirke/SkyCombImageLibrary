// Copyright SkyComb Limited 2025. All rights reserved.
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DrawSpace;
using SkyCombDrone.DroneLogic;
using SkyCombGround.CommonSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;


namespace SkyCombImage.DrawSpace
{
    // Code to draw object etc images on a frame image based on hot spot data
    public class DrawFrameImage : Draw
    {
        // Draw the hot pixels
        public static void HotPixels(
            DrawImageConfig config, ProcessConfigModel processConfig,
            ref Image<Bgr, byte> image, ProcessFeature feature, Transform transform)
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
            int thresholdStep = (255 - processConfig.HeatThresholdValue) / numColors;
            int[] threshold = new int[numColors];
            for (int i = 0; i < numColors; i++)
                threshold[i] = processConfig.HeatThresholdValue + i * thresholdStep;

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
        public static void DrawObjectFeatures(
            DrawImageConfig config, ref Image<Bgr, byte> image, Transform transform,
            ProcessFeature drawFeature,
            string drawObjectName,
            ProcessObject? focusObject)
        {
            if (config.DrawRealFeatureColor == Color.White &&
                config.DrawUnrealFeatureColor == Color.White &&
                DroneColors.InScopeObjectColor == Color.White &&
                DroneColors.OutScopeObjectColor == Color.White)
                return;

            int drawObjectId = drawFeature.ObjectId;
            int focusObjectId = focusObject != null ? focusObject.ObjectId : UnknownValue;
            var isFocusObject = (drawObjectId == focusObjectId);

            if (drawObjectId > 0)
            {
                var theColor = Color.White;
                if (focusObjectId > 0)
                    theColor = (isFocusObject ? DroneColors.InScopeObjectColor : DroneColors.RealFeatureColor);
                else if (drawFeature.Type == FeatureTypeEnum.Unreal)
                    theColor = config.DrawUnrealFeatureColor;
                else if (drawFeature.Type == FeatureTypeEnum.Real)
                    theColor = drawFeature.Significant ? DroneColors.InScopeObjectColor : config.DrawRealFeatureColor;

                if (theColor != Color.White)
                {
                    int thickness = (int)transform.Scale * config.TextExtraScale / 2;
                    var scaledRect = transform.CalcRect(drawFeature.PixelBox);

                    BoundingRectangle(config, ref image, scaledRect, theColor, thickness, config.AreaPadding * config.BoxExtraScale);

                    if (drawObjectName != "")
                        // Helps identify points visually on image to facilitate mapping to xls data.
                        // Combined with a very small video time span to process, can be very useful.
                        if (drawFeature.Significant || isFocusObject ||
                            (focusObjectId < 0)) // Draw object # for all objects?
                        {
                            // Draw the object name to right of the rectangle.
                            int separation_pixels = 8;
                            image.Draw(drawObjectName,
                                new Point(scaledRect.X + scaledRect.Width + separation_pixels, scaledRect.Y + separation_pixels),
                                FontFace.HersheyPlain, transform.Scale * config.TextExtraScale, DroneColors.ColorToBgr(theColor), thickness);
                        }
                }
            }
        }


        // Draw all hot pixels for current block, bounding rectangles of the owned features
        public static void DrawRunProcess(
            DrawImageConfig drawConfig, ProcessConfigModel processConfig,
            ref Image<Bgr, byte> outputImg, Transform transform,
            ProcessObject? focusObject, // Chosen object (if any)
            ProcessBlockModel? block,
            ProcessAll processAll,
            bool drawObjectNames)
        {
            try
            {
                if (block == null)
                    return;

                // Draw the leg name on the image (if any) 
                if (drawObjectNames && (block.FlightLegId > 0))
                {
                    var video = processAll.Drone.InputVideo;
                    if (video != null)
                    {
                        int theY = video.ImageHeight * 99 / 100; // pixels 
                        int theX = video.ImageWidth * 88 / 100; // pixels
                        var fontScale = video.FontScale;
                        Text(ref outputImg, "Leg " + block.FlightLegName,
                                new Point(theX, theY), fontScale / 2.0f, DroneColors.LegNameBgr, fontScale);
                    }
                }

                for (int featureId = block.MinFeatureId; featureId <= block.MaxFeatureId; featureId++)
                {
                    if (processAll.ProcessFeatures.ContainsKey(featureId))
                    {
                        var drawFeature = processAll.ProcessFeatures[featureId];
                        Assert(drawFeature.BlockId == block.BlockId, "ProcessedImage: Bad logic");

                        // Draw all hot pixels for the current block 
                        HotPixels(drawConfig, processConfig, ref outputImg, drawFeature, transform);

                        // Draw the bounding rectangle of the owned feature & object name
                        var drawObjectName = "";
                        if (drawObjectNames)
                        {
                            var drawObjectId = drawFeature.ObjectId;
                            if (drawObjectId > 0)
                                drawObjectName = processAll.ProcessObjects[drawObjectId].Name;
                            if (drawObjectName == "")
                                drawObjectName = "#" + drawObjectId;
                        }
                        DrawObjectFeatures(drawConfig, ref outputImg, transform, drawFeature, drawObjectName, focusObject);
                    }
                }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawFrameImage.DrawRunProcess", ex);
            }
        }


        // Process a single input video frame for the specified block, returning the modified input frames to show 
        public static Image<Bgr, byte>? Draw(
            RunProcessEnum runProcess,
            ProcessConfigModel processConfig, DrawImageConfig drawConfig, Drone drone,
            in Image<Bgr, byte> inputFrame, // Read-only
            ProcessObject? focusObject, // Chosen object (if any)
            ProcessBlockModel? block,
            ProcessAll processAll,
            bool drawObjectNames = true)
        {
            try
            {
                Image<Bgr, byte>? modifiedInputFrame = null;

                if (inputFrame != null)
                {
                    modifiedInputFrame = inputFrame.Clone();

                    if ((runProcess == RunProcessEnum.Comb) || (runProcess == RunProcessEnum.Yolo) || (runProcess == RunProcessEnum.Threshold))
                    {
                        // For Threshold, first apply the thermal coloring
                        if (runProcess == RunProcessEnum.Threshold)
                            DrawImage.Draw(runProcess, processConfig, drawConfig, ref modifiedInputFrame);

                        // Then draw bounding rectangles and object names for all three methods
                        DrawRunProcess(
                            drawConfig, processConfig, ref modifiedInputFrame, new(),
                            focusObject, block, processAll, drawObjectNames);
                    }
                    else
                        // Draw None - no processing
                        { /* Do nothing - just return original image */ }
                }
                return modifiedInputFrame;
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawFrameImage.Draw", ex);
            }
        }


        // Move visibleBox so it lies fully inside imageSize.
        // Doing this for an object seen over frames from top to bottom of thermal images,
        // object will visually drift in from the top for a few frames, then stay centred vertically 
        // for the bulk of the time, then drift out the bottom for a few frames. Nice!
        public static Rectangle MoveVisibleBoxInsideImageSize(Rectangle visibleBox, Size imageSize)
        {
            int pixelWidth = visibleBox.Width;
            int pixelHeight = visibleBox.Height;

            return new Rectangle(
                Math.Max(0, Math.Min(visibleBox.Left, imageSize.Width - pixelWidth)),
                Math.Max(0, Math.Min(visibleBox.Top, imageSize.Height - pixelHeight)),
                pixelWidth,
                pixelHeight);
        }
    }


    // Code to draw ground tree-top & drone alitudes against lineal meters, with objects overlaid
    public class ProcessDrawElevations : DrawElevations
    {
        private ProcessAll Process { get; }
        private ProcessDrawScope DrawScope { get; }

        private RunConfig? RunConfig { get; }


        public ProcessDrawElevations(ProcessAll process, ProcessDrawScope drawScope, RunConfig? runConfig) : base(drawScope)
        {
            Process = process;
            DrawScope = drawScope;
            RunConfig = runConfig;
        }


        // Draw altitude data dependant on Drone/GroundSpace data
        public override void Initialise(Size size)
        {
            base.Initialise(size);

            // Draw significant object as horizontally-stretched H with centroid - showing object duration and height
            // GraphObjects(ref BaseImage); //nq removing this as GraphObjects is called later
        }


        // Draw object at best estimate of height, location with error bars
        public void GraphObjects(ref Image<Bgr, byte> currImage)
        {
            try
            {
                if ((DroneDrawScope.Drone != null) && (Process != null) && (Process.ProcessObjects.Count > 0))
                    foreach (var thisObject in Process.ProcessObjects)
                        if (thisObject.Value.Significant &&
                            (thisObject.Value.HeightM > UnknownHeight) &&
                            ((RunConfig == null) || RunConfig.InRange(thisObject.Value))
                        )
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
            catch (Exception ex)
            {
                throw ThrowException("ProcessDrawElevations.GraphObjects", ex);
            }
        }


        // Draw altitude data based on Drone/GroundSpace & RunSpace data
        public override void CurrImage(ref Image<Bgr, byte> image, List<Image>? sizeImages = null)
        {
            // Draw significant object as horizontally-stretched H with centroid - showing object duration and height
            GraphObjects(ref image);
        }
    }


    // Code to draw object height - including feature estimates
    public class DrawObjectHeight : DrawAltitudeByTime
    {
        private ProcessAll Process { get; }
        private ProcessDrawScope DrawScope { get; }


        public DrawObjectHeight(ProcessAll process, ProcessDrawScope drawScope) : base(drawScope)
        {
            Process = process;
            DrawScope = drawScope;
            Description +=
                "Object & object feature height are shown.";
        }


        public void Initialise(Size size, ProcessObject focusObject)
        {
            try
            {
                Title = "Drone Height";

                MinVertRaw = (float)Math.Max(0, Math.Floor(focusObject.MinHeightM));
                MaxVertRaw = (float)Math.Ceiling(focusObject.MaxHeightM);

                SetVerticalLabels("m");
                SetHorizLabelsByTime();

                NormalCase = false;
                base.Initialise(size);
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawObjectHeight.Initialise", ex);
            }
        }

        // Update the height range graph
        public void CurrImage(ref Image<Bgr, byte> image, ProcessObject thisObject)
        {
            try
            {
                var inObjectBgr = DroneColors.InScopeObjectBgr;   // Red
                var realBgr = DroneColors.RealFeatureBgr;         // Orange
                var unrealBgr = DroneColors.UnrealFeatureBgr;     // Yellow
                Bgr highlight = new Bgr(170, 205, 102);

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
                foreach (var thisFeature in thisObject.ProcessFeatures)
                    if ((thisFeature.Value.HeightM > ProcessObjectModel.UnknownHeight) &&
                        (thisFeature.Value.BlockId <= DroneDrawScope.MaxFeatureBlockIdToDraw))
                    {
                        var theBgr = (thisFeature.Value.Type == FeatureTypeEnum.Real ? realBgr : unrealBgr);
                        theBgr = thisFeature.Value.BlockId < DroneDrawScope.MaxFeatureBlockIdToDraw ? theBgr : highlight;
                        var thisHeightPxs = TrimHeight(RawDataToHeightPixels(thisFeature.Value.HeightM - MinVertRaw, VertRangeRaw));
                        var thisWidthPxs = StepToWidth(thisFeature.Value.Block.SumTimeMs, firstMs);
                        var thisThickness = thisFeature.Value.BlockId < DroneDrawScope.MaxFeatureBlockIdToDraw ? NormalThickness : NormalThickness * 2;

                        Draw.Cross(ref image,
                            new Point(thisWidthPxs, thisHeightPxs),
                            theBgr, thisThickness, NormalThickness * 4);
                    }
            }
            catch (Exception ex)
            {
                throw ThrowException("DrawObjectHeight.CurrImage", ex);
            }
        }
    }
}