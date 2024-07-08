// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombImage.ProcessLogic;
using SkyCombDrone.DrawSpace;
using SkyCombGround.CommonSpace;
using System.Windows.Forms;


namespace SkyCombImage.DrawSpace
{
    // Code to draw data grid related to objects found in the video
    public class DrawObjectList : Draw
    {

        public static int LoadObjectGrid(List<object[]> objectsData, DataGridView objectGrid, int numListCols)
        {
            try
            {
                objectGrid.Rows.Clear();

                if (objectsData == null)
                    return 0;

                int numObjects = objectsData.Count;
                for (int i = 0; i < numObjects; i++)
                {
                    var rowObjects = new object[numListCols];
                    for (int j = 0; j < numListCols; j++)
                        rowObjects[j] = objectsData[i][j];

                    objectGrid.Rows.Add(rowObjects);

                    if (objectGrid.Rows[i].Cells.Count > ProcessObject.GridLocationMSetting)
                    {
                        var theColor = objectsData[i][ProcessObject.GridLocationGoodSetting].ToString();
                        var theCell = objectGrid.Rows[i].Cells[ProcessObject.GridLocationMSetting];
                        if (theColor == "good")
                            theCell.Style.BackColor = GoodValueColor;
                        else if (theColor == "bad")
                            theCell.Style.BackColor = BadValueColor;
                    }

                    if (objectGrid.Rows[i].Cells.Count > ProcessObject.GridHeightMSetting)
                    {
                        var theColor = objectsData[i][ProcessObject.GridHeightGoodSetting].ToString();
                        var theCell = objectGrid.Rows[i].Cells[ProcessObject.GridHeightMSetting];
                        if (theColor == "good")
                            theCell.Style.BackColor = GoodValueColor;
                        else if (theColor == "bad")
                            theCell.Style.BackColor = BadValueColor;
                    }
                }

                return numObjects;
            }
            catch (Exception ex)
            {
                throw BaseConstants.ThrowException("DrawObjectList.LoadObjectGrid", ex);
            }
        }
    }
}

