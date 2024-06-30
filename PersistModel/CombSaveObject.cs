// Copyright SkyComb Limited 2023. All rights reserved. 
using OfficeOpenXml;
using OfficeOpenXml.Drawing.Chart;
using OfficeOpenXml.Table.PivotTable;
using SkyCombDrone.DroneModel;
using SkyCombImage.ProcessLogic;
using SkyCombImage.ProcessModel;
using SkyCombDrone.CommonSpace;
using SkyCombDrone.PersistModel;
using System.Drawing;


namespace SkyCombImage.PersistModel
{
    // Save Comb processing model data to a datastore
    public class CombSaveObject : DataStoreAccessor
    {
        const string ObjectHeightPivotName = "ObjectHeightPivot";
        const string ObjectSizePivotName = "ObjectSizePivot";
        const string ObjectHeatPivotName = "ObjectHeatPivot";


        public CombSaveObject(DroneDataStore data) : base(data)
        {
        }


        public void SaveObjectList(CombProcess combProcess, bool saveAll)
        {
            Data.SelectOrAddWorksheet(Objects1TabName);
            int objectRow = 0;
            foreach (var theObject in combProcess.CombObjs.CombObjList)
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
            Data.AddConditionalRuleGood(ProcessObjectModel.LocationErrMSetting, objectRow, combProcess.ProcessConfig.GoodLocationErrM);
            Data.AddConditionalRuleBad(ProcessObjectModel.LocationErrMSetting, objectRow, combProcess.ProcessConfig.BadLocationErrM);
            Data.AddConditionalRuleGood(ProcessObjectModel.HeightErrMSetting, objectRow, combProcess.ProcessConfig.GoodHeightErrM);
            Data.AddConditionalRuleBad(ProcessObjectModel.HeightErrMSetting, objectRow, combProcess.ProcessConfig.BadHeightErrM);

            Data.SetLastUpdateDateTime(Objects1TabName);
        }


        // Add a speed graph for comb objects and features and ground speed
        public void AddCombGroundFeatureObjectSpeedGraph(int maxBlockId)
        {
/* PQR
            const string ChartName = "GroundFeatureObjectSpeed";
            const string ChartTitle = "Ground, Feature & Object speed (in pixels / block)";

            (var chartWs, var lastRow) = Data.PrepareChartArea(Objects2TabName, ChartName, Objects1TabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 0, 0, StandardChartRows);
                Data.SetAxises(chart, "Block", "Speed", "0");
                chart.XAxis.MinValue = 0;
                chart.XAxis.MaxValue = maxBlockId;

                Data.AddScatterSerie(chart, FeaturesTabName, "Feature", ProcessFeatureModel.ObjSpeedPxlsSetting, ProcessFeatureModel.BlockIdSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, Objects1TabName, "Objects", ProcessObjectModel.SpeedInPxPerBlockSetting, ProcessObjectModel.CenterBlockSetting, DroneColors.InScopeObjectColor, 6);
            }
*/
        }


        // Add a height graph for comb objects and features
        public void AddCombFeatureObjectHeightGraph(int maxBlockId)
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


        // Add a scatter plot of the comb objects and features.
        public void AddCombObjectFeatureScatterGraph()
        {
            const string ChartName = "ObjectScatterPlot";
            const string ChartTitle = "Object & Feature Locations (Northing / Easting) in meters";

            (var chartWs, var lastRow) = Data.PrepareChartArea(Objects2TabName, ChartName, Objects1TabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 1, 0, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0.00", "0.00");

                Data.AddScatterSerie(chart, FeaturesTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, Objects1TabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add a scatter plot of the comb object and feature locations.
        public void AddCombFlightObjectFeatureGraph()
        {
            const string ChartName = "FlightObjectFeaturePlot";
            const string ChartTitle = "Drone Steps, Object & Feature Locations (Northing / Easting) in meters";

            (var chartWs, var lastRow) = Data.PrepareChartArea(Objects2TabName, ChartName, Objects1TabName);
            if (lastRow > 0)
            {
                var chart = chartWs.Drawings.AddScatterChart(ChartName, eScatterChartType.XYScatter);
                Data.SetChart(chart, ChartTitle, 1, 1, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0.00", "0.00");

                Data.AddScatterSerie(chart, Steps1TabName, "Step", TardisModel.NorthingMSetting, TardisModel.EastingMSetting, DroneColors.InScopeDroneColor);
                Data.AddScatterSerie(chart, FeaturesTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, Objects1TabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add up to 4 Comb object / feature charts / graphs
        public void SaveObjectGraphs(int maxBlockId)
        {
            Data.SelectOrAddWorksheet(Objects2TabName);
            Data.Worksheet.Drawings.Clear();
            if (maxBlockId > 0)
            {
                AddCombGroundFeatureObjectSpeedGraph(maxBlockId);
                AddCombFeatureObjectHeightGraph(maxBlockId);
            }
            AddCombObjectFeatureScatterGraph();
            AddCombFlightObjectFeatureGraph();

            Data.SetLastUpdateDateTime(Objects2TabName);
        }


        // Create pivot of the object height (rounded to nearest 0.5m) vs #objects to support charting
        public void AddObjectHeightPivot(ExcelWorksheet objectWs, int maxObjRow)
        {
            var pivotTable = Data.Worksheet.PivotTables[ObjectHeightPivotName];
            if (pivotTable != null)
            {
                // If user changed legs selected, the number of objects may have changed
                pivotTable.CacheDefinition.SourceRange = objectWs.Cells["A1:Q" + maxObjRow];
                return;
            }


            Data.Worksheet.Cells[23, 4].Value = "# Objects by Height (M) pivot";

            pivotTable = Data.Worksheet.PivotTables.Add(
                Data.Worksheet.Cells["D24"], objectWs.Cells["A1:Q" + maxObjRow], ObjectHeightPivotName);
            pivotTable.DataOnRows = true;
            pivotTable.ColumnGrandTotals = false;
            pivotTable.RowGrandTotals = false;
            pivotTable.ShowDrill = false;

            var rowField = pivotTable.RowFields.Add(pivotTable.Fields["Hght Rnd M"]);
            rowField.Name = "HeightM";
            rowField.Sort = eSortType.Ascending;

            var dataField = pivotTable.DataFields.Add(pivotTable.Fields["Name"]);
            dataField.Function = DataFieldFunctions.Count;
            dataField.Format = "#,##0";
            dataField.Name = "NumObjects";
        }


        // Create pivot of the object size (rounded to nearest 100cm2) vs #objects to support charting
        public void AddObjectSizePivot(ExcelWorksheet objectWs, int maxObjRow)
        {
            var pivotTable = Data.Worksheet.PivotTables[ObjectSizePivotName];
            if (pivotTable != null)
            {
                // If user changed legs selected, the number of objects may have changed
                pivotTable.CacheDefinition.SourceRange = objectWs.Cells["A1:Q" + maxObjRow];
                return;
            }


            Data.Worksheet.Cells[23, 12].Value = "# Objects by Size (CM2) pivot";

            pivotTable = Data.Worksheet.PivotTables.Add(
                Data.Worksheet.Cells["L24"], objectWs.Cells["A1:Q" + maxObjRow], ObjectSizePivotName);
            pivotTable.DataOnRows = true;
            pivotTable.ColumnGrandTotals = false;
            pivotTable.RowGrandTotals = false;
            pivotTable.ShowDrill = false;

            var rowField = pivotTable.RowFields.Add(pivotTable.Fields["Size Rnd CM2"]);
            rowField.Name = "SizeCM2";
            rowField.Sort = eSortType.Ascending;

            var dataField = pivotTable.DataFields.Add(pivotTable.Fields["Name"]);
            dataField.Function = DataFieldFunctions.Count;
            dataField.Format = "#,##0";
            dataField.Name = "NumObjects";
        }


        // Create pivot of the object heat vs #objects to support charting
        public void AddObjectHeatPivot(ExcelWorksheet objectWs, int maxObjRow)
        {
            var pivotTable = Data.Worksheet.PivotTables[ObjectHeatPivotName];
            if (pivotTable != null)
            {
                // If user changed legs selected, the number of objects may have changed
                pivotTable.CacheDefinition.SourceRange = objectWs.Cells["A1:Q" + maxObjRow];
                return;
            }


            Data.Worksheet.Cells[23, 20].Value = "# Objects by Heat (0-255) pivot";

            pivotTable = Data.Worksheet.PivotTables.Add(
                Data.Worksheet.Cells["T24"], objectWs.Cells["A1:Q" + maxObjRow], ObjectHeatPivotName);
            pivotTable.DataOnRows = true;
            pivotTable.ColumnGrandTotals = false;
            pivotTable.RowGrandTotals = false;
            pivotTable.ShowDrill = false;

            var rowField = pivotTable.RowFields.Add(pivotTable.Fields["Max Heat"]);
            rowField.Name = "Max Heat";
            rowField.Sort = eSortType.Ascending;

            var dataField = pivotTable.DataFields.Add(pivotTable.Fields["Name"]);
            dataField.Function = DataFieldFunctions.Count;
            dataField.Format = "#,##0";
            dataField.Name = "NumObjects";
        }


        public void AddObjectHeightGraph()
        {
            const string ChartName = "ObjectHeightPopulation";
            const string ChartTitle = "# Objects by Height (M)";

            if (Data.Worksheet.Drawings[ChartName] != null)
                return;

            var chart = Data.Worksheet.Drawings.AddChart(ChartName, eChartType.ColumnStacked,
                Data.Worksheet.PivotTables[ObjectHeightPivotName]);
            Data.SetChartTitle(chart, ChartTitle);
            chart.SetPosition(2, 0, 3, 0);
            chart.SetSize(500, 300);
            chart.Legend.Remove();
        }


        public void AddObjectSizeGraph()
        {
            const string ChartName = "ObjectSizePopulation";
            const string ChartTitle = "# Objects by Size (CM2)";

            if (Data.Worksheet.Drawings[ChartName] != null)
                return;

            var chart = Data.Worksheet.Drawings.AddChart(ChartName, eChartType.ColumnStacked,
                Data.Worksheet.PivotTables[ObjectSizePivotName]);
            Data.SetChartTitle(chart, ChartTitle);
            chart.SetPosition(2, 0, 11, 0);
            chart.SetSize(500, 300);
            chart.Legend.Remove();
        }


        public void AddObjectHeatGraph()
        {
            const string ChartName = "ObjectHeatPopulation";
            const string ChartTitle = "# Objects by Max Heat (0 - 255)";

            if (Data.Worksheet.Drawings[ChartName] != null)
                return;

            var chart = Data.Worksheet.Drawings.AddChart(ChartName, eChartType.ColumnStacked,
                Data.Worksheet.PivotTables[ObjectHeatPivotName]);
            Data.SetChartTitle(chart, ChartTitle);
            chart.SetPosition(2, 0, 19, 0);
            chart.SetSize(500, 300);
            chart.Legend.Remove();
        }


        // Save population summary data, pivots and graphs
        public void SavePopulation(CombProcess combProcess)
        {
            try
            {
                Data.SelectOrAddWorksheet(PopulationTabName);

                int row = 1;
                Data.SetTitle(ref row, 1, PopulationSummaryTitle, LargeTitleFontSize);
                Data.SetTitleAndDataListColumn(ObjectSummaryTitle, ModelTitleRow, LhsColOffset, combProcess.CombObjs.CombObjList.GetSettings());
                Data.SetColumnWidth(1, 20);

                (var objectWs, int maxObjRow) = Data.EndRow(Objects1TabName);
                if (maxObjRow > 2)
                {
                    AddObjectHeightPivot(objectWs, maxObjRow);
                    AddObjectSizePivot(objectWs, maxObjRow);
                    AddObjectHeatPivot(objectWs, maxObjRow);

                    AddObjectHeightGraph();
                    AddObjectSizeGraph();
                    AddObjectHeatGraph();
                }

                Data.SetLastUpdateDateTime(PopulationTabName);
            }
            catch (Exception ex)
            {
                throw ThrowException("CombSaveObject.SavePopulation", ex);
            }
        }
    }
}