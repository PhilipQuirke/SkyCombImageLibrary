// Copyright SkyComb Limited 2024. All rights reserved. 
using OfficeOpenXml.Drawing.Chart;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.DroneModel;
using SkyCombDrone.PersistModel;
using SkyCombImage.CategorySpace;
using SkyCombImage.DrawSpace;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
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
            Data.SelectOrAddWorksheet(FeaturesTabName);
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

            Data.SetLastUpdateDateTime(FeaturesTabName);
        }


        public void SaveObjectList(ProcessAll process, bool saveAll)
        {
            Data.SelectOrAddWorksheet(Objects1TabName);
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

            Data.SetLastUpdateDateTime(Objects1TabName);
        }


        public void SaveSpanList(ProcessAll process)
        {
            // Save the ProcessSpan data 
            if ((process.ProcessSpans != null) && (process.ProcessSpans.Count > 0))
            {
                Data.SelectOrAddWorksheet(SpanTabName);
                int legRow = 0;
                foreach (var leg in process.ProcessSpans)
                    Data.SetDataListRowKeysAndValues(ref legRow, leg.Value.GetSettings());

                Data.SetColumnColor(ProcessSpanModel.SpanIdSetting, legRow, Color.Blue);
                Data.SetColumnColor(ProcessSpanModel.SpanNameSetting, legRow, Color.Blue);
                Data.SetColumnColor(ProcessSpanModel.BestFixAltMSetting, legRow, Color.Blue);

                Data.SetLastUpdateDateTime(SpanTabName);
            }
        }


        // Add a height graph for objects and features
        public void AddProcessFeatureObjectHeightGraph(int maxBlockId)
        {
            const string ChartName = "FeatureObjectHeight";
            const string ChartTitle = "Feature & Object Height (in meters)";

            (var chartWs, var lastRow) = Data.PrepareChartArea(Objects2TabName, ChartName, Objects1TabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 0, 1, StandardChartRows);
                Data.SetAxises(chart, "Block", "Height", "0.0");
                chart.XAxis.MinValue = 0;
                chart.XAxis.MaxValue = maxBlockId;

                Data.AddScatterSerie(chart, FeaturesTabName, "Feature", ProcessFeatureModel.HeightMSetting, ProcessFeatureModel.BlockIdSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, Objects1TabName, "Objects", ProcessObjectModel.HeightMSetting, ProcessObjectModel.CenterBlockSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add a scatter plot of the objects and features.
        public void AddProcessObjectFeatureScatterGraph()
        {
            const string ChartName = "ObjectScatterPlot";
            const string ChartTitle = "Object & Feature Locations (Northing / Easting) in meters";

            (var chartWs, var lastRow) = Data.PrepareChartArea(Objects2TabName, ChartName, Objects1TabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 1, 0, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0", "0");

                Data.AddScatterSerie(chart, FeaturesTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, Objects1TabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add a scatter plot of the process object and feature locations.
        public void AddProcessFlightObjectFeatureGraph()
        {
            const string ChartName = "FlightObjectFeaturePlot";
            const string ChartTitle = "Drone Steps, Object & Feature Locations (Northing / Easting) in meters";

            (var chartWs, var lastRow) = Data.PrepareChartArea(Objects2TabName, ChartName, Objects1TabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 1, 1, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0", "0");

                Data.AddScatterSerie(chart, Steps1TabName, "Step", TardisModel.NorthingMSetting, TardisModel.EastingMSetting, DroneColors.InScopeDroneColor);
                Data.AddScatterSerie(chart, FeaturesTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, Objects1TabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add up to 4  object / feature charts / graphs
        public void SaveObjectGraphs(int maxBlockId, ProcessAll processAll)
        {
            Data.SelectOrAddWorksheet(Objects2TabName);
            Data.Worksheet.Drawings.Clear();
            if (maxBlockId > 0)
            {
                AddProcessFeatureObjectHeightGraph(maxBlockId);
            }
            AddProcessObjectFeatureScatterGraph();
            AddProcessFlightObjectFeatureGraph();

            ProcessObjList processObjects = processAll.ProcessObjects;
            var processScope = new ProcessScope(processAll.Drone);
            var drawScope = new ProcessDrawScope(processAll, processScope, processAll.Drone);
            if (processObjects.Count > 0)
            {
                var objectDrawScope = new ObjectDrawScope(processAll, processScope, processAll.Drone);
                objectDrawScope.SetObjectRange(processObjects);

                // Draw the histogram of object heights
                int row = 1;
                Data.SetTitle(ref row, 1, "Object Height Histogram");
                var drawHeightHistogram = new ProcessDrawHeightHistogram(drawScope, objectDrawScope, MasterHeightModelList.GetObjectCountByHeightClass(processObjects));
                drawHeightHistogram.Initialise(new Size(550, 200));
                var localBitmap = drawHeightHistogram.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Object Height Histogram", 2, 0);

                // Draw the histogram of object sizes
                row = 1;
                Data.SetTitle(ref row, 7, "Object Size Histogram");
                var drawSizeHistogram = new ProcessDrawSizeHistogram(drawScope, objectDrawScope, MasterSizeModelList.GetObjectCountBySizeClass(processObjects));
                drawSizeHistogram.Initialise(new Size(550, 200));
                localBitmap = drawSizeHistogram.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Object Size Histogram", 2, 6);

                Data.SetLastUpdateDateTime(Objects2TabName);
            }
        }
    }
}