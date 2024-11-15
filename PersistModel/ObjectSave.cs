// Copyright SkyComb Limited 2024. All rights reserved. 

using OfficeOpenXml.Drawing.Chart;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.CategorySpace;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombImage.RunSpace;
using System.Drawing;


namespace SkyCombImage.PersistModel
{
    // Save processing model data to a datastore
    public class ObjectSave : DataStoreAccessor
    {
        const string ObjectHeightPivotName = "ObjectHeightPivot";
        const string ObjectSizePivotName = "ObjectSizePivot";
        const string ObjectHeatPivotName = "ObjectHeatPivot";


        public ObjectSave(DroneDataStore data) : base(data)
        {
        }


        public void SaveFeatureList(ProcessAll process, bool saveAll)
        {
            Data.SelectOrAddWorksheet(AnimalImageDataTabName);
            int featureRow = 0;
            foreach (var feature in process.ProcessFeatures)
                if (saveAll || feature.Value.Significant)
                    Data.SetDataListRowKeysAndValues(ref featureRow, feature.Value.GetSettings());

            Data.SetNumberColumnNdp(ProcessFeatureModel.NorthingMSetting, LocationNdp);
            Data.SetNumberColumnNdp(ProcessFeatureModel.EastingMSetting, LocationNdp);
            Data.SetNumberColumnNdp(ProcessFeatureModel.HeightMSetting, HeightNdp);

            Data.SetColumnColor(ProcessFeatureModel.FeatureIdSetting, featureRow, Color.Blue);
            Data.SetColumnColor(ProcessFeatureModel.ObjectIdSetting, featureRow, Color.Blue);
            Data.SetColumnColor(ProcessFeatureModel.BlockIdSetting, featureRow, Color.Blue);
            Data.SetColumnColor(ProcessFeatureModel.HeightMSetting, featureRow, Color.Blue);
            Data.SetColumnColor(ProcessFeatureModel.LegIdSetting, featureRow, Color.Blue);
        }


        public void SaveObjectList(ProcessAll process, bool saveAll)
        {
            Data.SelectOrAddWorksheet(AnimalsDataTabName);
            int objectRow = 0;
            foreach (var theObject in process.ProcessObjects)
                if (saveAll || theObject.Value.NumSigBlocks > 0)
                    Data.SetDataListRowKeysAndValues(ref objectRow, theObject.Value.GetSettings());

            Data.SetColumnWidth(ProcessObjectModel.AttributesSetting, 20);

            Data.SetColumnColor(ProcessObjectModel.ObjectIdSetting, objectRow, Color.Blue);
            Data.SetColumnColor(ProcessObjectModel.NameSetting, objectRow, Color.Blue);
            Data.SetColumnColor(ProcessObjectModel.HeightMSetting, objectRow, Color.Blue);
            Data.SetColumnColor(ProcessObjectModel.SizeCM2Setting, objectRow, Color.Blue);
            Data.SetColumnColor(ProcessObjectModel.LegIdSetting, objectRow, Color.Blue);
            Data.SetColumnColor(ProcessObjectModel.CenterBlockSetting, objectRow, Color.Blue);

            // Highlight cells in green/red to show their accuracy
            Data.AddConditionalRuleGood(ProcessObjectModel.LocationErrMSetting, objectRow, process.ProcessConfig.GoodLocationErrM);
            Data.AddConditionalRuleBad(ProcessObjectModel.LocationErrMSetting, objectRow, process.ProcessConfig.GoodLocationErrM);
            Data.AddConditionalRuleGood(ProcessObjectModel.HeightErrMSetting, objectRow, process.ProcessConfig.GoodHeightErrM);
            Data.AddConditionalRuleBad(ProcessObjectModel.HeightErrMSetting, objectRow, process.ProcessConfig.GoodHeightErrM);
        }


        public void SaveSpanList(ProcessAll process)
        {
            // Save the ProcessSpan data 
            if ((process.ProcessSpans != null) && (process.ProcessSpans.Count > 0))
            {
                Data.SelectOrAddWorksheet(SpanDataTabName);
                int legRow = 0;
                foreach (var leg in process.ProcessSpans)
                    Data.SetDataListRowKeysAndValues(ref legRow, leg.Value.GetSettings());

                Data.SetColumnColor(ProcessSpanModel.SpanIdSetting, legRow, Color.Blue);
                Data.SetColumnColor(ProcessSpanModel.SpanNameSetting, legRow, Color.Blue);
                Data.SetColumnColor(ProcessSpanModel.BestFixAltMSetting, legRow, Color.Blue);
            }
        }


        // Add a height graph for objects and features
        public void AddProcessFeatureObjectHeightGraph(int maxBlockId)
        {
            const string ChartName = "FeatureObjectHeight";
            const string ChartTitle = "Feature & Object Height (in meters)";

            (var chartWs, var lastRow) = Data.PrepareChartArea(AnimalReportTabName, ChartName, AnimalsDataTabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 0, 1, StandardChartRows);
                Data.SetAxises(chart, "Block", "Height", "0.0");
                chart.XAxis.MinValue = 0;
                chart.XAxis.MaxValue = maxBlockId;

                Data.AddScatterSerie(chart, AnimalImageDataTabName, "Feature", ProcessFeatureModel.HeightMSetting, ProcessFeatureModel.BlockIdSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, AnimalsDataTabName, "Objects", ProcessObjectModel.HeightMSetting, ProcessObjectModel.CenterBlockSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add a scatter plot of the objects and features.
        public void AddProcessObjectFeatureScatterGraph()
        {
            const string ChartName = "ObjectScatterPlot";
            const string ChartTitle = "Animal Locations (Northing / Easting) in meters";

            (var chartWs, var lastRow) = Data.PrepareChartArea(AnimalReportTabName, ChartName, AnimalsDataTabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 1, 0, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0", "0");

                Data.AddScatterSerie(chart, AnimalImageDataTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, AnimalsDataTabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add a scatter plot of the process object and feature locations.
        public void AddProcessFlightObjectFeatureGraph()
        {
            const string ChartName = "FlightObjectFeaturePlot";
            const string ChartTitle = "Drone & Animal Locations (Northing / Easting) in meters";

            (var chartWs, var lastRow) = Data.PrepareChartArea(AnimalReportTabName, ChartName, AnimalsDataTabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 1, 1, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0", "0");

                Data.AddScatterSerie(chart, StepDataTabName, "Step", TardisModel.NorthingMSetting, TardisModel.EastingMSetting, DroneColors.InScopeDroneColor);
                Data.AddScatterSerie(chart, AnimalImageDataTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, AnimalsDataTabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add object summary with charts and graphs
        public void SaveObjectReport(int maxBlockId, RunVideoPersist runVideo)
        {
            var processAll = runVideo.ProcessAll;
            var processScope = runVideo;
            var processDrawScope = runVideo.ProcessDrawScope;
            var processObjects = processAll.ProcessObjects;

            Data.SelectOrAddWorksheet(AnimalReportTabName);
            Data.Worksheet.Drawings.Clear();

            Data.SetLargeTitle(AnimalReportTitle);

            int numObjs = processAll.ProcessObjects.NumSignificantObjects;
            int swathe = processAll.GroundData.SwatheModel.M2Seen;
            var density = numObjs * 1000000.0 / swathe;

            int row = 3;
            Data.SetTitle(ref row, 1, "Metrics");

            Data.Worksheet.Cells[4, 1].Value = "Animals";
            Data.Worksheet.Cells[4, 4].Value = numObjs;

            Data.Worksheet.Cells[5, 1].Value = "Flight 'swathe' coverage";
            Data.Worksheet.Cells[5, 4].Value = swathe;
            Data.Worksheet.Cells[5, 5].Value = "m2";

            Data.Worksheet.Cells[6, 1].Value = "Objects density";
            Data.Worksheet.Cells[6, 4].Value = density.ToString("F6");
            Data.Worksheet.Cells[6, 5].Value = "animals/km2";

            AddProcessObjectFeatureScatterGraph();

            if (processObjects.Count > 0)
            {
                var objectDrawScope = new ObjectDrawScope(processAll, processScope, processAll.Drone);
                objectDrawScope.SetObjectRange(processObjects);

                // Draw the histogram of object heights
                row = 3;
                int col = 9;
                Data.SetTitle(ref row, col, "Animal Height Histogram");
                var drawHeightHistogram = new ProcessDrawHeightHistogram(processDrawScope, objectDrawScope, MasterHeightModelList.GetObjectCountByHeightClass(processObjects));
                drawHeightHistogram.Initialise(new Size(350, 150));
                var localBitmap = drawHeightHistogram.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Animal Height Histogram", row-1, col-1);

                // Draw the histogram of object sizes
                row = 3;
                col = 15;
                Data.SetTitle(ref row, col, "Animal Size Histogram");
                var drawSizeHistogram = new ProcessDrawSizeHistogram(processDrawScope, objectDrawScope, MasterSizeModelList.GetObjectCountBySizeClass(processObjects));
                drawSizeHistogram.Initialise(new Size(350, 150));
                localBitmap = drawSizeHistogram.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Animal Size Histogram", row-1, col-1);

                // Draw the flight path with objects and features
                row = 16;
                col = 15;
                Data.SetTitle(ref row, col, "Flight Path with Animals");
                var drawFlightPath = new ProcessDrawPath(processDrawScope, processObjects, objectDrawScope);
                drawFlightPath.Initialise(new Size(825, 825));
                localBitmap = drawFlightPath.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Flight Path with Animals", row-1, col-1);

                // Draw the elevations with objects and features
                row = 47;
                col = 1;
                Data.SetTitle(ref row, col, "Flight and Animals Elevations");
                var drawElevations = new ProcessDrawElevations(processAll, processDrawScope);
                drawElevations.Initialise(new Size(ChartFullWidthPixels, 250));
                localBitmap = drawElevations.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Flight and Animals Elevations", row-1, col-1);
                DroneSave.SaveElevationLegend(Data, row, 23, 1, 1);
                Data.Worksheet.Cells[row, 24].Value = "Drone";
                Data.Worksheet.Cells[row+1, 24].Value = "Surface";
                Data.Worksheet.Cells[row+2, 24].Value = "Ground";
            }
        }
    }
}