// Copyright SkyComb Limited 2025. All rights reserved. 

using OfficeOpenXml;
using OfficeOpenXml.Drawing.Chart;
using OfficeOpenXml.Table.PivotTable;
using SkyCombDrone.CommonSpace;
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
            Data.SetColumnColor(ProcessObjectModel.NumSigBlocksSetting, objectRow, Color.Blue);

            // Highlight cells in green/red to show their accuracy
            Data.AddConditionalRuleGood(ProcessObjectModel.LocationErrMSetting, objectRow, ProcessConfigModel.GoodLocationErrM);
            Data.AddConditionalRuleBad(ProcessObjectModel.LocationErrMSetting, objectRow, ProcessConfigModel.GoodLocationErrM);
            Data.AddConditionalRuleGood(ProcessObjectModel.HeightErrMSetting, objectRow, ProcessConfigModel.GoodHeightErrM);
            Data.AddConditionalRuleBad(ProcessObjectModel.HeightErrMSetting, objectRow, ProcessConfigModel.GoodHeightErrM);
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
                Data.SetColumnColor(ProcessSpanModel.BestSumLocnErrMSetting, legRow, Color.Blue);
                Data.SetColumnColor(ProcessSpanModel.OrgSumLocnErrMSetting, legRow, Color.Blue);
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
                Data.SetChart(chart, ChartTitle, 1.1f, 0, LargeChartRows);
                Data.SetAxises(chart, "Easting", "Northing", "0", "0");

                Data.AddScatterSerie(chart, AnimalImageDataTabName, "Feature", ProcessFeatureModel.NorthingMSetting, ProcessFeatureModel.EastingMSetting, DroneColors.RealFeatureColor);
                Data.AddScatterSerie(chart, AnimalsDataTabName, "Object", ProcessObjectModel.NorthingMSetting, ProcessObjectModel.EastingMSetting, DroneColors.InScopeObjectColor, 6);
            }
        }


        // Add a pivot table summarizing the animal locations, starting at cell (row,1):
        // Show Leg on the vertical axis.
        // Show columns num objects, average object location error, maximum object location error.
        // Show grand totals.
        public void AddProcessObjectLocationPivot(ExcelWorksheet ws, int startRow, int startCol)
        {
            try { 

                const string PivotName = "AnimalLocationPivot";
                if (ws.PivotTables[PivotName] != null)
                    return;

                // Prepare the pivot table area
                (var pivotWs, var dataWs, var lastDataRow) = Data.PreparePivotArea(AnimalReportTabName, PivotName, AnimalsDataTabName);
                if (pivotWs == null || dataWs == null || lastDataRow <= 1)
                    return;

                // Create pivot table
                var pivotTableRange = dataWs.Cells[1, 1, lastDataRow, 34];
                var pivotCell = pivotWs.Cells[startRow, startCol];
                var pivotTable = pivotWs.PivotTables.Add(pivotCell, pivotTableRange, PivotName);

                // Add row (vertical) field 
                pivotTable.RowFields.Add(pivotTable.Fields["Leg"]);

                // Add data fields
                var numObjectsField = pivotTable.DataFields.Add(pivotTable.Fields["Name"]);
                numObjectsField.Name = "# Objects";
                numObjectsField.Function = DataFieldFunctions.Count;

                var avgLocErrField = pivotTable.DataFields.Add(pivotTable.Fields["Locn Err M"]);
                avgLocErrField.Name = "Avg Locn Err";
                avgLocErrField.Function = DataFieldFunctions.Average;
                avgLocErrField.Format = "0.0"; // 1 decimal place

                var maxLocErrField = pivotTable.DataFields.Add(pivotTable.Fields["Locn Err M"]);
                maxLocErrField.Name = "Max Locn Err";
                maxLocErrField.Function = DataFieldFunctions.Max;
                maxLocErrField.Format = "0.0"; // 1 decimal place

                pivotTable.DataOnRows = false;
                pivotTable.ShowDrill = true;
                pivotTable.ShowHeaders = true;
                pivotTable.UseAutoFormatting = true;
                pivotTable.FirstDataCol = 2;

                Data.AddConditionalFormattingToPivotTable(pivotTable);
            }
            catch (Exception ex)
            {
                // Suppress error
            }
        }


        // Add a pivot table summarizing the animals, starting at cell (startRow,startCol):
        // Show Leg on the vertical axis.
        // Show columns num objects, average object height error, maximum object height error.
        // Show grand totals.
        public void AddProcessObjectHeightPivot(ExcelWorksheet ws, int startRow, int startCol)
        {
            try { 
                const string PivotName = "AnimalHeightPivot";
                if (ws.PivotTables[PivotName] != null)
                    return;

                // Prepare the pivot table area
                (var pivotWs, var dataWs, var lastDataRow) = Data.PreparePivotArea(AnimalReportTabName, PivotName, AnimalsDataTabName);
                if (pivotWs == null || dataWs == null || lastDataRow <= 1)
                    return;

                // Create pivot table
                var pivotTableRange = dataWs.Cells[1, 1, lastDataRow, 34]; 
                var pivotCell = pivotWs.Cells[startRow, startCol];
                var pivotTable = pivotWs.PivotTables.Add(pivotCell, pivotTableRange, PivotName);

                // Add row (vertical) field 
                pivotTable.RowFields.Add(pivotTable.Fields["Leg"]);

                // Add data fields
                var numObjectsField = pivotTable.DataFields.Add(pivotTable.Fields["Name"]);
                numObjectsField.Name = "# Objects";
                numObjectsField.Function = DataFieldFunctions.Count;

                var avgLocErrField = pivotTable.DataFields.Add(pivotTable.Fields["Hght Err M"]);
                avgLocErrField.Name = "Avg Height Err";
                avgLocErrField.Function = DataFieldFunctions.Average;
                avgLocErrField.Format = "0.0"; // 1 decimal place

                var maxLocErrField = pivotTable.DataFields.Add(pivotTable.Fields["Hght Err M"]);
                maxLocErrField.Name = "Max Height Err";
                maxLocErrField.Function = DataFieldFunctions.Max;
                maxLocErrField.Format = "0.0"; // 1 decimal place

                pivotTable.DataOnRows = false;
                pivotTable.ShowDrill = true;
                pivotTable.ShowHeaders = true;
                pivotTable.UseAutoFormatting = true;
                pivotTable.FirstDataCol = 2;

                Data.AddConditionalFormattingToPivotTable(pivotTable);
            }
            catch (Exception ex)
            {
                // Suppress error
            }
        }


        public static void SetKeyResults(ExcelWorksheet ws, int numAnimals, double swatheM2, int col1, int col2)
        {
            var swatheKM2 = swatheM2 / 1000000.0;
            var animalDensity = swatheKM2 > 0 ? numAnimals / swatheKM2 : 0;

            ws.Cells[4, col1].Value = "Animals";
            ws.Cells[4, col2].Value = numAnimals;

            ws.Cells[5, col1].Value = "Flight 'swathe' coverage";
            ws.Cells[5, col2].Value = swatheKM2.ToString("0.000");
            ws.Cells[5, col2 + 1].Value = "km2";

            ws.Cells[6, col1].Value = "Animal density";
            ws.Cells[6, col2].Value = animalDensity.ToString("0.0");
            ws.Cells[6, col2 + 1].Value = "animals/km2";
        }


        // Add object summary with charts and graphs
        public void SaveObjectReport(int maxBlockId, RunWorkerPersist runVideo)
        {
            var processAll = runVideo.ProcessAll;
            var processScope = runVideo;
            var processDrawScope = runVideo.ProcessDrawScope;
            var processObjects = processAll.ProcessObjects;

            Data.SelectOrAddWorksheet(AnimalReportTabName);
            var ws = Data.Worksheet;

            ws.Drawings.Clear();

            Data.SetLargeTitle(AnimalReportTitle);

            int numAnimals = processAll.ProcessObjects.NumSignificantObjects;
            int swatheM2 = processAll.GroundData.SwatheModel.M2Seen;

            int row = 3;
            Data.SetTitle(ref row, 1, "Metrics");
            ObjectSave.SetKeyResults(ws, numAnimals, swatheM2, 1, 4);

            AddProcessObjectFeatureScatterGraph();

            if (processObjects.Count > 0)
            {
                var objectDrawScope = new ObjectDrawScope(processAll, processScope, processAll.Drone);
                objectDrawScope.SetObjectRange(processObjects);

                // Draw the histogram of object sizes
                row = 8;
                int col = 1;
                Data.SetTitle(ref row, col, "Animal Size Histogram");
                var drawSizeHistogram = new ProcessDrawSizeHistogram(processDrawScope, objectDrawScope, MasterSizeModelList.GetObjectCountBySizeClass(processObjects));
                drawSizeHistogram.Initialise(new Size(350, 150));
                var localBitmap = drawSizeHistogram.CurrBitmap(true, runVideo.SizeImages);
                Data.SaveBitmap(localBitmap, "AnimalSizeHistogram", row-1, col-1);

                // Draw the matrix of animal sizes and heights
                row = 3;
                col = 15;
                AnimalModelList animals = new();
                animals.AddProcessObjects(0, processAll.Drone, processObjects, processAll.ProcessSpans);
                (var message, var matrixBitmap) = AnimalMatrixDrawer.DrawAnimalMatrix(animals, runVideo.SizeImages, true);
                Data.SetTitle(ref row, col, "Animal Size Height Matrix: " + message);
                Data.SaveBitmap(matrixBitmap, "AnimalMatrix", row - 1, col - 1, 67);

                // Draw the flight path with objects and features
                row = 18;
                col = 15;
                Data.SetTitle(ref row, col, "Flight Path with Animals");
                var drawFlightPath = new ProcessDrawPath(processDrawScope, processObjects, objectDrawScope);
                drawFlightPath.BackgroundColor = DroneColors.WhiteBgr;
                drawFlightPath.Initialise(new Size(575, 575));
                localBitmap = drawFlightPath.CurrBitmap(true);
                Data.SaveBitmap(localBitmap, "FlightPathWithAnimals", row-1, col-1);

                // Draw the elevations with objects and features
                row = 49;
                col = 1;
                Data.SetTitle(ref row, col, "Flight and Animals Elevations");
                var drawElevations = new ProcessDrawElevations(processAll, processDrawScope, null);
                drawElevations.Initialise(new Size(ChartFullWidthPixels, 250));
                localBitmap = drawElevations.CurrBitmap();
                Data.SaveBitmap(localBitmap, "Flight and Animals Elevations", row-1, col-1);
                DroneSave.SaveElevationLegend(Data, row, 23, 1, 1);
                ws.Cells[row, 24].Value = "Drone";
                ws.Cells[row+1, 24].Value = "Surface";
                ws.Cells[row+2, 24].Value = "Ground";

                // Draw the object pivot
                row = 65;
                col = 1;
                Data.SetTitle(ref row, col, "Animals Location Error by Leg");
                AddProcessObjectLocationPivot(ws, row, col);
                row = 65;
                col = 12;
                Data.SetTitle(ref row, col, "Animals Height Error by Leg");
                AddProcessObjectHeightPivot(ws, row, col);
            }
        }
    }
}