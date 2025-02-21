/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkChartXY.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// Hide VTK_DEPRECATED_IN_9_0_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkChartXY.h"

#include "vtkAnnotationLink.h"
#include "vtkAxis.h"
#include "vtkBrush.h"
#include "vtkCallbackCommand.h"
#include "vtkChartLegend.h"
#include "vtkColorSeries.h"
#include "vtkCommand.h"
#include "vtkCompositeDataIterator.h"
#include "vtkContext2D.h"
#include "vtkContextClip.h"
#include "vtkContextKeyEvent.h"
#include "vtkContextMapper2D.h"
#include "vtkContextMouseEvent.h"
#include "vtkContextScene.h"
#include "vtkContextTransform.h"
#include "vtkDataArray.h"
#include "vtkDataSetAttributes.h"
#include "vtkIdTypeArray.h"
#include "vtkInformation.h"
#include "vtkMath.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkObjectFactory.h"
#include "vtkPen.h"
#include "vtkPlotArea.h"
#include "vtkPlotBag.h"
#include "vtkPlotBar.h"
#include "vtkPlotFunctionalBag.h"
#include "vtkPlotGrid.h"
#include "vtkPlotLine.h"
#include "vtkPlotPoints.h"
#include "vtkPlotStacked.h"
#include "vtkPoints2D.h"
#include "vtkSelection.h"
#include "vtkSelectionNode.h"
#include "vtkSmartPointer.h"
#include "vtkStdString.h"
#include "vtkStringArray.h"
#include "vtkTable.h"
#include "vtkTextProperty.h"
#include "vtkTooltipItem.h"
#include "vtkTransform2D.h"
#include "vtkVector.h"
#include "vtkVectorOperators.h"

// My STL containers
#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

//------------------------------------------------------------------------------
class vtkChartXYPrivate
{
public:
  vtkChartXYPrivate()
  {
    this->Colors = vtkSmartPointer<vtkColorSeries>::New();
    this->Clip = vtkSmartPointer<vtkContextClip>::New();
    this->Borders[0] = 60;
    this->Borders[1] = 50;
    this->Borders[2] = 20;
    this->Borders[3] = 20;
  }

  static void InvalidateCache(vtkObject*, unsigned long, void* clientData, void*)
  {
    vtkChartXYPrivate* self = reinterpret_cast<vtkChartXYPrivate*>(clientData);
    self->PlotCacheUpdated = false;
  }

  void UpdatePlotCache()
  {
    if (!this->PlotCacheUpdated)
    {
      this->PlotCache.clear();

      // build map to find array index from its pointer to speed up access
      // the table should be shared between all plots
      std::unordered_map<vtkTable*, std::unordered_map<vtkAbstractArray*, vtkIdType>> colMap;
      for (vtkPlot* plot : this->plots)
      {
        vtkTable* table = plot->GetInput();
        if (table)
        {
          auto it = colMap.find(table);
          if (it == colMap.end())
          {
            std::unordered_map<vtkAbstractArray*, vtkIdType> tableColMap;

            vtkIdType nbCols = table->GetNumberOfColumns();
            for (vtkIdType i = 0; i < nbCols; i++)
            {
              tableColMap[table->GetColumn(i)] = i;
            }

            colMap[table] = std::move(tableColMap);
          }
        }
      }

      // build map to find plot from column index
      for (vtkPlot* plot : this->plots)
      {
        vtkTable* table = plot->GetInput();

        constexpr int idx = 1; // column
        vtkAbstractArray* array = plot->GetData()->GetInputAbstractArrayToProcess(idx, table);

        this->PlotCache[colMap[table][array]] = plot;
      }

      this->PlotCacheUpdated = true;
    }
  }

  std::vector<vtkPlot*> plots;                   // Charts can contain multiple plots of data
  std::vector<vtkContextTransform*> PlotCorners; // Stored by corner...
  std::vector<vtkAxis*> axes;                    // Charts can contain multiple axes
  vtkSmartPointer<vtkColorSeries> Colors;        // Colors in the chart
  vtkSmartPointer<vtkContextClip> Clip;          // Colors in the chart
  int Borders[4];
  vtkTimeStamp TransformCalculatedTime;
  std::unordered_map<vtkIdType, vtkPlot*> PlotCache;
  bool PlotCacheUpdated = false;

  // Associate a flat composite index to a list of related plot
  std::unordered_map<unsigned int, std::vector<vtkPlot*>> MapCompositeIndexToPlots;
  // Reversed association
  std::unordered_map<vtkPlot*, unsigned int> PlotCompositeIndexes;
};

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkChartXY);

//------------------------------------------------------------------------------
vtkChartXY::vtkChartXY()
{
  this->ChartPrivate = new vtkChartXYPrivate;

  this->AutoAxes = true;
  this->HiddenAxisBorder = 20;

  // The plots are drawn in a clipped, transformed area.
  this->AddItem(this->ChartPrivate->Clip);

  // The grid is drawn first in this clipped, transformed area.
  vtkPlotGrid* grid1 = vtkPlotGrid::New();
  this->ChartPrivate->Clip->AddItem(grid1);
  grid1->Delete();

  // The second grid for the far side/top axis
  vtkPlotGrid* grid2 = vtkPlotGrid::New();
  this->ChartPrivate->Clip->AddItem(grid2);
  grid2->Delete();

  // Set up the bottom-left transform, the rest are often not required (set up
  // on demand if used later). Add it as a child item, rendered automatically.
  vtkSmartPointer<vtkContextTransform> corner = vtkSmartPointer<vtkContextTransform>::New();
  this->ChartPrivate->PlotCorners.push_back(corner);
  this->ChartPrivate->Clip->AddItem(corner); // Child list maintains ownership.

  // Next is the axes
  for (int i = 0; i < 4; ++i)
  {
    this->ChartPrivate->axes.push_back(vtkAxis::New());
    // By default just show the left and bottom axes
    this->ChartPrivate->axes.back()->SetVisible(i < 2 ? true : false);
    this->AttachAxisRangeListener(this->ChartPrivate->axes.back());
    this->AddItem(this->ChartPrivate->axes.back());
  }
  this->ChartPrivate->axes[vtkAxis::LEFT]->SetPosition(vtkAxis::LEFT);
  this->ChartPrivate->axes[vtkAxis::BOTTOM]->SetPosition(vtkAxis::BOTTOM);
  this->ChartPrivate->axes[vtkAxis::RIGHT]->SetPosition(vtkAxis::RIGHT);
  this->ChartPrivate->axes[vtkAxis::TOP]->SetPosition(vtkAxis::TOP);

  // Set up the x and y axes - should be configured based on data
  this->ChartPrivate->axes[vtkAxis::LEFT]->SetTitle("Y Axis");
  this->ChartPrivate->axes[vtkAxis::BOTTOM]->SetTitle("X Axis");

  grid1->SetXAxis(this->ChartPrivate->axes[vtkAxis::BOTTOM]);
  grid1->SetYAxis(this->ChartPrivate->axes[vtkAxis::LEFT]);
  grid2->SetXAxis(this->ChartPrivate->axes[vtkAxis::TOP]);
  grid2->SetYAxis(this->ChartPrivate->axes[vtkAxis::RIGHT]);

  // Then the legend is drawn
  this->Legend = vtkSmartPointer<vtkChartLegend>::New();
  this->Legend->SetChart(this);
  this->Legend->SetVisible(false);
  this->AddItem(this->Legend);

  this->PlotTransformValid = false;

  this->DrawBox = false;
  this->DrawSelectionPolygon = false;
  this->DrawNearestPoint = false;
  this->DrawAxesAtOrigin = false;
  this->BarWidthFraction = 0.8f;

  this->Tooltip = vtkSmartPointer<vtkTooltipItem>::New();
  this->Tooltip->SetVisible(false);
  this->AddItem(this->Tooltip);

  this->ForceAxesToBounds = false;
  this->IgnoreNanInBounds = false;
  this->ZoomWithMouseWheel = true;
  this->AdjustLowerBoundForLogPlot = false;

  this->DragPoint = false;
  this->DragPointAlongX = true;
  this->DragPointAlongY = true;
}

//------------------------------------------------------------------------------
vtkChartXY::~vtkChartXY()
{
  for (unsigned int i = 0; i < this->ChartPrivate->plots.size(); ++i)
  {
    this->ChartPrivate->plots[i]->Delete();
  }
  for (size_t i = 0; i < 4; ++i)
  {
    this->ChartPrivate->axes[i]->Delete();
  }
  delete this->ChartPrivate;
  this->ChartPrivate = nullptr;
}

//------------------------------------------------------------------------------
void vtkChartXY::Update()
{
  // Perform any necessary updates that are not graphical
  // Update the plots if necessary
  for (size_t i = 0; i < this->ChartPrivate->plots.size(); ++i)
  {
    this->ChartPrivate->plots[i]->Update();
  }
  this->Legend->Update();

  // Update the selections if necessary.
  if (this->AnnotationLink)
  {
    this->AnnotationLink->Update();
    vtkSelection* selection =
      vtkSelection::SafeDownCast(this->AnnotationLink->GetOutputDataObject(2));
    // Two major selection methods - row based or plot based.
    if (this->SelectionMethod == vtkChart::SELECTION_ROWS)
    {
      // Clear former selections before assigning anything
      this->ReleasePlotSelections();

      for (unsigned int i = 0; i < selection->GetNumberOfNodes(); ++i)
      {
        vtkSelectionNode* node = selection->GetNode(i);
        vtkIdTypeArray* idArray = vtkArrayDownCast<vtkIdTypeArray>(node->GetSelectionList());
        if (node->GetProperties()->Has(vtkSelectionNode::COMPOSITE_INDEX()))
        {
          const int compositeIndex =
            node->GetProperties()->Get(vtkSelectionNode::COMPOSITE_INDEX());

          const auto findIndex = this->ChartPrivate->MapCompositeIndexToPlots.find(compositeIndex);
          if (findIndex != this->ChartPrivate->MapCompositeIndexToPlots.end())
          {
            for (vtkPlot* plot : findIndex->second)
            {
              plot->SetSelection(idArray);
            }
          }
        }
        else
        {
          // Use the first selection node for all plots to select the rows if no composite index
          for (vtkPlot* plot : this->ChartPrivate->plots)
          {
            plot->SetSelection(idArray);
          }
          break;
        }
      }
    }
    else if (this->SelectionMethod == vtkChart::SELECTION_PLOTS)
    {
      for (unsigned int i = 0; i < selection->GetNumberOfNodes(); ++i)
      {
        vtkSelectionNode* node = selection->GetNode(i);
        vtkIdTypeArray* idArray = vtkArrayDownCast<vtkIdTypeArray>(node->GetSelectionList());
        vtkPlot* selectionPlot =
          vtkPlot::SafeDownCast(node->GetProperties()->Get(vtkSelectionNode::PROP()));
        // Now iterate through the plots to update selection data
        std::vector<vtkPlot*>::iterator it = this->ChartPrivate->plots.begin();
        for (; it != this->ChartPrivate->plots.end(); ++it)
        {
          if (selectionPlot == *it)
          {
            (*it)->SetSelection(idArray);
          }
        }
      }
    }
    else if (this->SelectionMethod == vtkChart::SELECTION_COLUMNS)
    {
      // Retrieve all the selected plots
      this->ChartPrivate->UpdatePlotCache();
      std::vector<vtkPlot*> selectedPlots;
      for (unsigned int i = 0; i < selection->GetNumberOfNodes(); ++i)
      {
        vtkSelectionNode* node = selection->GetNode(i);
        vtkIdTypeArray* selectedColumns =
          vtkArrayDownCast<vtkIdTypeArray>(node->GetSelectionList());
        for (vtkIdType j = 0; j < selectedColumns->GetNumberOfTuples(); ++j)
        {
          vtkIdType id = selectedColumns->GetTypedComponent(j, 0);
          selectedPlots.push_back(this->ChartPrivate->PlotCache[id]);
        }
      }
      // Now iterate through the plots to update selection data
      for (vtkPlot* plot : this->ChartPrivate->plots)
      {
        vtkSmartPointer<vtkIdTypeArray> plotSelection;
        bool isSelected =
          std::find(selectedPlots.begin(), selectedPlots.end(), plot) != selectedPlots.end();
        if (isSelected)
        {
          constexpr int idx = 1; // y
          vtkAbstractArray* column =
            plot->GetData()->GetInputAbstractArrayToProcess(idx, plot->GetInput());
          plotSelection = plot->GetSelection();
          if (!plotSelection || plotSelection->GetNumberOfTuples() != column->GetNumberOfTuples())
          {
            plotSelection = vtkSmartPointer<vtkIdTypeArray>::New();
            for (vtkIdType j = 0; j < column->GetNumberOfTuples(); ++j)
            {
              plotSelection->InsertNextValue(j);
            }
          }
        }
        plot->SetSelection(plotSelection);
      }
    }
  }
  else
  {
    vtkDebugMacro("No annotation link set.");
  }

  this->CalculateBarPlots();

  if (this->AutoAxes)
  {
    vtkTuple<bool, 4> visibilities(false);
    for (int i = 0; i < static_cast<int>(this->ChartPrivate->PlotCorners.size()); ++i)
    {
      int visible = 0;
      for (vtkIdType j = 0; j < this->ChartPrivate->PlotCorners[i]->GetNumberOfItems(); ++j)
      {
        if (vtkPlot::SafeDownCast(this->ChartPrivate->PlotCorners[i]->GetItem(j))->GetVisible())
        {
          ++visible;
        }
      }
      if (visible)
      {
        visibilities[i % 4] = true;
        visibilities[(i + 1) % 4] = true;
      }
    }
    for (int i = 0; i < 4; ++i)
    {
      this->ChartPrivate->axes[i]->SetVisible(visibilities[i]);
    }
  }
}

//------------------------------------------------------------------------------
bool vtkChartXY::Paint(vtkContext2D* painter)
{
  // This is where everything should be drawn, or dispatched to other methods.
  vtkDebugMacro(<< "Paint event called.");
  if (!this->Visible)
  {
    // The geometry of the chart must be valid before anything can be drawn
    return false;
  }

  vtkVector2i geometry(0, 0);
  bool recalculateTransform = false;
  if (this->LayoutStrategy == vtkChart::FILL_SCENE)
  {
    geometry = vtkVector2i(this->GetScene()->GetSceneWidth(), this->GetScene()->GetSceneHeight());
    if (geometry.GetX() != this->Geometry[0] || geometry.GetY() != this->Geometry[1])
    {
      recalculateTransform = true;
    }
    this->SetSize(vtkRectf(0.0, 0.0, geometry.GetX(), geometry.GetY()));
  }

  int visiblePlots = 0;
  for (size_t i = 0; i < this->ChartPrivate->plots.size(); ++i)
  {
    if (this->ChartPrivate->plots[i]->GetVisible())
    {
      ++visiblePlots;
    }
  }
  if (visiblePlots == 0 && !this->RenderEmpty)
  {
    // Nothing to plot, so don't draw anything.
    return false;
  }

  this->Update();
  this->UpdateLayout(painter);

  // Axes may have changed during updateLayout
  if (this->ChartPrivate->TransformCalculatedTime < this->ChartPrivate->axes[0]->GetMTime() ||
    this->ChartPrivate->TransformCalculatedTime < this->ChartPrivate->axes[1]->GetMTime() ||
    this->ChartPrivate->TransformCalculatedTime < this->ChartPrivate->axes[2]->GetMTime() ||
    this->ChartPrivate->TransformCalculatedTime < this->ChartPrivate->axes[3]->GetMTime())
  {
    // Cause the plot transform to be recalculated if necessary
    recalculateTransform = true;
  }

  // Recalculate the plot transform, min and max values if necessary
  if (!this->PlotTransformValid)
  {
    this->RecalculatePlotBounds();
    recalculateTransform = true;
  }
  if (this->UpdateLayout(painter) || recalculateTransform)
  {
    this->RecalculatePlotTransforms();
  }

  // Now that plot transforms, including whether to use log scaling and the
  // shift-scale factors, have been updated, we give the vtkPlot instances an
  // opportunity to update caches.
  for (size_t i = 0; i < this->ChartPrivate->plots.size(); ++i)
  {
    this->ChartPrivate->plots[i]->UpdateCache();
  }

  // Update the clipping if necessary
  this->ChartPrivate->Clip->SetClip(this->Point1[0], this->Point1[1],
    this->Point2[0] - this->Point1[0], this->Point2[1] - this->Point1[1]);

  // draw background
  if (this->BackgroundBrush)
  {
    painter->GetPen()->SetLineType(vtkPen::NO_PEN);
    painter->ApplyBrush(this->BackgroundBrush);
    painter->DrawRect(this->Point1[0], this->Point1[1], this->Geometry[0], this->Geometry[1]);
  }

  // Use the scene to render most of the chart.
  this->PaintChildren(painter);

  // Draw the selection box if necessary
  if (this->DrawBox)
  {
    painter->GetBrush()->SetColor(255, 255, 255, 0);
    painter->GetPen()->SetColor(0, 0, 0, 255);
    painter->GetPen()->SetWidth(1.0);
    painter->GetPen()->SetLineType(vtkPen::SOLID_LINE);
    painter->DrawRect(this->MouseBox.GetX(), this->MouseBox.GetY(), this->MouseBox.GetWidth(),
      this->MouseBox.GetHeight());
  }

  // Draw the selection polygon if necessary
  if (this->DrawSelectionPolygon)
  {
    painter->GetBrush()->SetColor(255, 0, 0, 0);
    painter->GetPen()->SetColor(0, 255, 0, 255);
    painter->GetPen()->SetWidth(2.0);
    painter->GetPen()->SetLineType(vtkPen::SOLID_LINE);

    const vtkContextPolygon& polygon = this->SelectionPolygon;

    // draw each line segment
    for (vtkIdType i = 0; i < polygon.GetNumberOfPoints() - 1; i++)
    {
      const vtkVector2f& a = polygon.GetPoint(i);
      const vtkVector2f& b = polygon.GetPoint(i + 1);

      painter->DrawLine(a.GetX(), a.GetY(), b.GetX(), b.GetY());
    }

    // draw a line from the end to the start
    if (polygon.GetNumberOfPoints() >= 3)
    {
      const vtkVector2f& start = polygon.GetPoint(0);
      const vtkVector2f& end = polygon.GetPoint(polygon.GetNumberOfPoints() - 1);

      painter->DrawLine(start.GetX(), start.GetY(), end.GetX(), end.GetY());
    }
  }

  if (this->Title)
  {
    int offset = 0; // title margin.
    vtkAxis* topAxis = this->ChartPrivate->axes[vtkAxis::TOP];
    if (topAxis->GetVisible())
    {
      vtkRectf bounds = topAxis->GetBoundingRect(painter);
      offset += static_cast<int>(bounds.GetHeight());
    }
    vtkPoints2D* rect = vtkPoints2D::New();
    rect->InsertNextPoint(this->Point1[0], this->Point2[1] + offset);
    rect->InsertNextPoint(this->Point2[0] - this->Point1[0], 10);
    painter->ApplyTextProp(this->TitleProperties);
    painter->DrawStringRect(rect, this->Title);
    rect->Delete();
  }

  return true;
}

//------------------------------------------------------------------------------
void vtkChartXY::CalculateBarPlots()
{
  // Calculate the width, spacing and offsets for the bar plot - they are grouped
  size_t n = this->ChartPrivate->plots.size();
  std::vector<vtkPlotBar*> bars;
  for (size_t i = 0; i < n; ++i)
  {
    vtkPlotBar* bar = vtkPlotBar::SafeDownCast(this->ChartPrivate->plots[i]);
    if (bar && bar->GetVisible())
    {
      bars.push_back(bar);
    }
  }
  if (!bars.empty())
  {
    // We have some bar plots - work out offsets etc.
    float barWidth = 0.1;
    vtkPlotBar* bar = bars[0];
    if (!bar->GetUseIndexForXSeries())
    {
      vtkTable* table = bar->GetData()->GetInput();
      if (table)
      {
        vtkDataArray* x = bar->GetData()->GetInputArrayToProcess(0, table);
        if (x && x->GetNumberOfTuples() > 1)
        {
          double x0 = x->GetTuple1(0);
          double x1 = x->GetTuple1(1);
          float width = static_cast<float>(fabs(x1 - x0) * this->BarWidthFraction);
          barWidth = width / bars.size();
        }
      }
    }
    else
    {
      barWidth = 1.0f / bars.size() * this->BarWidthFraction;
    }

    // Now set the offsets and widths on each bar
    // The offsetIndex deals with the fact that half the bars
    // must shift to the left of the point and half to the right
    int offsetIndex = static_cast<int>(bars.size() - 1);
    for (size_t i = 0; i < bars.size(); ++i)
    {
      bars[i]->SetWidth(barWidth);
      bars[i]->SetOffset(offsetIndex * (barWidth / 2));
      // Increment by two since we need to shift by half widths
      // but make room for entire bars. Increment backwards because
      // offsets are always subtracted and Positive offsets move
      // the bar leftwards.  Negative offsets will shift the bar
      // to the right.
      offsetIndex -= 2;
      // bars[i]->SetOffset(float(bars.size()-i-1)*(barWidth/2));
    }
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::RecalculatePlotTransforms()
{
  for (int i = 0; i < int(this->ChartPrivate->PlotCorners.size()); ++i)
  {
    if (this->ChartPrivate->PlotCorners[i]->GetNumberOfItems())
    {
      vtkAxis* xAxis = nullptr;
      vtkAxis* yAxis = nullptr;
      // Get the appropriate axes, and recalculate the transform.
      switch (i)
      {
        case 0:
        {
          xAxis = this->ChartPrivate->axes[vtkAxis::BOTTOM];
          yAxis = this->ChartPrivate->axes[vtkAxis::LEFT];
          break;
        }
        case 1:
          xAxis = this->ChartPrivate->axes[vtkAxis::BOTTOM];
          yAxis = this->ChartPrivate->axes[vtkAxis::RIGHT];
          break;
        case 2:
          xAxis = this->ChartPrivate->axes[vtkAxis::TOP];
          yAxis = this->ChartPrivate->axes[vtkAxis::RIGHT];
          break;
        case 3:
          xAxis = this->ChartPrivate->axes[vtkAxis::TOP];
          yAxis = this->ChartPrivate->axes[vtkAxis::LEFT];
          break;
        default:
          vtkWarningMacro("Error: default case in recalculate plot transforms.");
      }
      this->CalculatePlotTransform(
        xAxis, yAxis, this->ChartPrivate->PlotCorners[i]->GetTransform());
      // Now we need to set the scale factor on the plots to ensure they rescale
      // their input data when necessary.
      vtkRectd shiftScale(
        xAxis->GetShift(), yAxis->GetShift(), xAxis->GetScalingFactor(), yAxis->GetScalingFactor());
      for (vtkIdType j = 0; j < this->ChartPrivate->PlotCorners[i]->GetNumberOfItems(); ++j)
      {
        vtkPlot* plot = vtkPlot::SafeDownCast(this->ChartPrivate->PlotCorners[i]->GetItem(j));
        if (plot)
        {
          plot->SetShiftScale(shiftScale);
        }
      }
    }
  }
  this->PlotTransformValid = true;
  this->ChartPrivate->TransformCalculatedTime.Modified();
}

//------------------------------------------------------------------------------
int vtkChartXY::GetPlotCorner(vtkPlot* plot)
{
  vtkAxis* x = plot->GetXAxis();
  vtkAxis* y = plot->GetYAxis();
  if (x == this->ChartPrivate->axes[vtkAxis::BOTTOM] &&
    y == this->ChartPrivate->axes[vtkAxis::LEFT])
  {
    return 0;
  }
  else if (x == this->ChartPrivate->axes[vtkAxis::BOTTOM] &&
    y == this->ChartPrivate->axes[vtkAxis::RIGHT])
  {
    return 1;
  }
  else if (x == this->ChartPrivate->axes[vtkAxis::TOP] &&
    y == this->ChartPrivate->axes[vtkAxis::RIGHT])
  {
    return 2;
  }
  else if (x == this->ChartPrivate->axes[vtkAxis::TOP] &&
    y == this->ChartPrivate->axes[vtkAxis::LEFT])
  {
    return 3;
  }
  else
  {
    // Should never happen.
    return 4;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::SetPlotCorner(vtkPlot* plot, int corner)
{
  if (corner < 0 || corner > 3)
  {
    vtkWarningMacro("Invalid corner specified, should be between 0 and 3: " << corner);
    return;
  }
  if (this->GetPlotCorner(plot) == corner)
  {
    return;
  }
  this->RemovePlotFromCorners(plot);
  // Grow the plot corners if necessary
  while (static_cast<int>(this->ChartPrivate->PlotCorners.size() - 1) < corner)
  {
    vtkNew<vtkContextTransform> transform;
    this->ChartPrivate->PlotCorners.push_back(transform);
    this->ChartPrivate->Clip->AddItem(transform); // Clip maintains ownership.
  }
  this->ChartPrivate->PlotCorners[corner]->AddItem(plot);
  if (corner == 0)
  {
    plot->SetXAxis(this->ChartPrivate->axes[vtkAxis::BOTTOM]);
    plot->SetYAxis(this->ChartPrivate->axes[vtkAxis::LEFT]);
  }
  else if (corner == 1)
  {
    plot->SetXAxis(this->ChartPrivate->axes[vtkAxis::BOTTOM]);
    plot->SetYAxis(this->ChartPrivate->axes[vtkAxis::RIGHT]);
  }
  else if (corner == 2)
  {
    plot->SetXAxis(this->ChartPrivate->axes[vtkAxis::TOP]);
    plot->SetYAxis(this->ChartPrivate->axes[vtkAxis::RIGHT]);
  }
  else if (corner == 3)
  {
    plot->SetXAxis(this->ChartPrivate->axes[vtkAxis::TOP]);
    plot->SetYAxis(this->ChartPrivate->axes[vtkAxis::LEFT]);
  }
  this->PlotTransformValid = false;
}

//------------------------------------------------------------------------------
void vtkChartXY::RecalculatePlotBounds()
{
  // Get the bounds of each plot, and each axis  - ordering as laid out below
  double y1[] = { 0.0, 0.0 }; // left -> 0
  double x1[] = { 0.0, 0.0 }; // bottom -> 1
  double y2[] = { 0.0, 0.0 }; // right -> 2
  double x2[] = { 0.0, 0.0 }; // top -> 3
  // Store whether the ranges have been initialized - follows same order
  bool initialized[] = { false, false, false, false };

  std::vector<vtkPlot*>::iterator it;
  double bounds[4] = { 0.0, 0.0, 0.0, 0.0 };
  for (it = this->ChartPrivate->plots.begin(); it != this->ChartPrivate->plots.end(); ++it)
  {
    if ((*it)->GetVisible() == false)
    {
      continue;
    }
    (*it)->GetBounds(bounds);
    if (bounds[1] - bounds[0] < 0.0)
    {
      // skip uninitialized bounds.
      continue;
    }
    int corner = this->GetPlotCorner(*it);

    // Initialize the appropriate ranges, or push out the ranges
    if ((corner == 0 || corner == 3)) // left
    {
      if (!initialized[0])
      {
        y1[0] = bounds[2];
        y1[1] = bounds[3];
        initialized[0] = true;
      }
      else
      {
        if (this->IgnoreNanInBounds)
        {
          if (y1[0] > bounds[2] || std::isnan(y1[0])) // min
          {
            y1[0] = bounds[2];
          }
          if (y1[1] < bounds[3] || std::isnan(y1[1])) // max
          {
            y1[1] = bounds[3];
          }
        }
        else
        {
          if (y1[0] > bounds[2] || std::isnan(bounds[2])) // min
          {
            y1[0] = bounds[2];
          }
          if (y1[1] < bounds[3] || std::isnan(bounds[3])) // max
          {
            y1[1] = bounds[3];
          }
        }
      }
    }
    if ((corner == 0 || corner == 1)) // bottom
    {
      if (!initialized[1])
      {
        x1[0] = bounds[0];
        x1[1] = bounds[1];
        initialized[1] = true;
      }
      else
      {
        if (this->IgnoreNanInBounds)
        {
          if (x1[0] > bounds[0] || std::isnan(x1[0])) // min
          {
            x1[0] = bounds[0];
          }
          if (x1[1] < bounds[1] || std::isnan(x1[1])) // max
          {
            x1[1] = bounds[1];
          }
        }
        else
        {
          if (x1[0] > bounds[0] || std::isnan(bounds[0])) // min
          {
            x1[0] = bounds[0];
          }
          if (x1[1] < bounds[1] || std::isnan(bounds[1])) // max
          {
            x1[1] = bounds[1];
          }
        }
      }
    }
    if ((corner == 1 || corner == 2)) // right
    {
      if (!initialized[2])
      {
        y2[0] = bounds[2];
        y2[1] = bounds[3];
        initialized[2] = true;
      }
      else
      {
        if (this->IgnoreNanInBounds)
        {
          if (y2[0] > bounds[2] || std::isnan(y2[0])) // min
          {
            y2[0] = bounds[2];
          }
          if (y2[1] < bounds[3] || std::isnan(y2[1])) // max
          {
            y2[1] = bounds[3];
          }
        }
        else
        {
          if (y2[0] > bounds[2] || std::isnan(bounds[2])) // min
          {
            y2[0] = bounds[2];
          }
          if (y2[1] < bounds[3] || std::isnan(bounds[3])) // max
          {
            y2[1] = bounds[3];
          }
        }
      }
    }
    if ((corner == 2 || corner == 3)) // top
    {
      if (!initialized[3])
      {
        x2[0] = bounds[0];
        x2[1] = bounds[1];
        initialized[3] = true;
      }
      else
      {
        if (this->IgnoreNanInBounds)
        {
          if (x2[0] > bounds[0] || std::isnan(x2[0])) // min
          {
            x2[0] = bounds[0];
          }
          if (x2[1] < bounds[1] || std::isnan(x2[1])) // max
          {
            x2[1] = bounds[1];
          }
        }
        else
        {
          if (x2[0] > bounds[0] || std::isnan(bounds[0])) // min
          {
            x2[0] = bounds[0];
          }
          if (x2[1] < bounds[1] || std::isnan(bounds[1])) // max
          {
            x2[1] = bounds[1];
          }
        }
      }
    }
  }

  // Now set the newly calculated bounds on the axes
  for (int i = 0; i < 4; ++i)
  {
    vtkAxis* axis = this->ChartPrivate->axes[i];
    double* range = nullptr;
    switch (i)
    {
      case 0:
        range = y1;
        break;
      case 1:
        range = x1;
        break;
      case 2:
        range = y2;
        break;
      case 3:
        range = x2;
        break;
      default:
        return;
    }

    if (this->AdjustLowerBoundForLogPlot && axis->GetLogScale() &&
      (range[0] <= 0.0 || vtkMath::IsNan(range[0])))
    {
      if (range[1] <= 0.0 || vtkMath::IsNan(range[1]))
      {
        // All of the data is negative, so we arbitrarily set the axis range to
        // be positive and show no data
        range[1] = 1.;
      }

      // The minimum value is set to either 4 decades below the max or to 1,
      // regardless of the true minimum value (which is less than 0).
      if (axis->GetLogScaleActive())
      {
        // Need to adjust in log (scaled) space
        double candidateMin = range[1] - 4.0;
        range[0] = (candidateMin < 0.0 ? candidateMin : 0.0);
      }
      else
      {
        // Need to adjust in unscaled space
        double candidateMin = range[1] * 1.0e-4;
        range[0] = (candidateMin < 1.0 ? candidateMin : 1.0);
      }
    }
    if (this->ForceAxesToBounds && (range[0] != range[1]))
    {
      axis->SetMinimumLimit(range[0]);
      axis->SetMaximumLimit(range[1]);
    }
    if (axis->GetBehavior() == vtkAxis::AUTO && initialized[i])
    {
      axis->SetRange(range[0], range[1]);
      axis->AutoScale();
    }
  }

  this->Modified();
}

//------------------------------------------------------------------------------
void vtkChartXY::ReleasePlotSelections()
{
  std::vector<vtkPlot*>::iterator it = this->ChartPrivate->plots.begin();
  for (; it != this->ChartPrivate->plots.end(); ++it)
  {
    vtkPlot* plot = *it;
    if (!plot)
    {
      continue;
    }
    vtkNew<vtkIdTypeArray> emptySelectionArray;
    emptySelectionArray->Initialize();
    plot->SetSelection(emptySelectionArray);
  }
}

//------------------------------------------------------------------------------
bool vtkChartXY::UpdateLayout(vtkContext2D* painter)
{
  // The main use of this method is currently to query the visible axes for
  // their bounds, and to update the chart in response to that.
  bool changed = false;

  vtkVector2i tileScale = this->Scene->GetLogicalTileScale();
  vtkVector2i hiddenAxisBorder = tileScale * this->HiddenAxisBorder;

  // Axes
  if (this->LayoutStrategy == vtkChart::FILL_SCENE || this->LayoutStrategy == vtkChart::FILL_RECT)
  {
    for (int i = 0; i < 4; ++i)
    {
      int border = 0;
      vtkAxis* axis = this->ChartPrivate->axes[i];
      axis->Update();
      if (axis->GetVisible())
      {
        vtkRectf bounds = axis->GetBoundingRect(painter);
        if (i == vtkAxis::TOP || i == vtkAxis::BOTTOM)
        { // Horizontal axes
          border = int(bounds.GetHeight());
        }
        else
        { // Vertical axes
          border = int(bounds.GetWidth());
        }
      }
      border += this->GetLegendBorder(painter, i);
      if (i == vtkAxis::TOP && this->Title)
      {
        painter->ApplyTextProp(this->TitleProperties);
        float bounds[4];
        painter->ComputeStringBounds(this->Title, bounds);
        if (bounds[3] > 0)
        {
          border += (5 * tileScale.GetY()) /* title margin */
            + bounds[3];                   // add the title text height to the border.
        }
      }

      if (i == vtkAxis::TOP || i == vtkAxis::BOTTOM)
      {
        border = std::max(border, hiddenAxisBorder.GetY());
      }
      else
      {
        border = std::max(border, hiddenAxisBorder.GetX());
      }

      if (this->ChartPrivate->Borders[i] != border)
      {
        this->ChartPrivate->Borders[i] = border;
        changed = true;
      }
    }
  }

  if (this->DrawAxesAtOrigin)
  {
    this->SetBorders(hiddenAxisBorder.GetX(), hiddenAxisBorder.GetY(),
      this->ChartPrivate->Borders[2], this->ChartPrivate->Borders[3]);
    // Get the screen coordinates for the origin, and move the axes there.
    vtkVector2f origin(0.0);
    vtkTransform2D* transform = this->ChartPrivate->PlotCorners[0]->GetTransform();
    transform->TransformPoints(origin.GetData(), origin.GetData(), 1);
    // Need to clamp the axes in the plot area.
    if (int(origin[0]) < this->Point1[0])
    {
      origin[0] = this->Point1[0];
    }
    if (int(origin[0]) > this->Point2[0])
    {
      origin[0] = this->Point2[0];
    }
    if (int(origin[1]) < this->Point1[1])
    {
      origin[1] = this->Point1[1];
    }
    if (int(origin[1]) > this->Point2[1])
    {
      origin[1] = this->Point2[1];
    }

    this->ChartPrivate->axes[vtkAxis::BOTTOM]->SetPoint1(this->Point1[0], origin[1]);
    this->ChartPrivate->axes[vtkAxis::BOTTOM]->SetPoint2(this->Point2[0], origin[1]);
    this->ChartPrivate->axes[vtkAxis::LEFT]->SetPoint1(origin[0], this->Point1[1]);
    this->ChartPrivate->axes[vtkAxis::LEFT]->SetPoint2(origin[0], this->Point2[1]);
  }
  else
  {
    if (this->LayoutStrategy == vtkChart::AXES_TO_RECT)
    {
      this->SetBorders(0, 0, 0, 0);
      this->ChartPrivate->axes[0]->GetBoundingRect(painter);
      this->ChartPrivate->axes[1]->GetBoundingRect(painter);
      this->ChartPrivate->axes[2]->GetBoundingRect(painter);
      this->ChartPrivate->axes[3]->GetBoundingRect(painter);
    }
    else
    {
      this->SetBorders(this->ChartPrivate->Borders[0], this->ChartPrivate->Borders[1],
        this->ChartPrivate->Borders[2], this->ChartPrivate->Borders[3]);
    }
    // This is where we set the axes up too
    // Y axis (left)
    this->ChartPrivate->axes[0]->SetPoint1(this->Point1[0], this->Point1[1]);
    this->ChartPrivate->axes[0]->SetPoint2(this->Point1[0], this->Point2[1]);
    // X axis (bottom)
    this->ChartPrivate->axes[1]->SetPoint1(this->Point1[0], this->Point1[1]);
    this->ChartPrivate->axes[1]->SetPoint2(this->Point2[0], this->Point1[1]);
    // Y axis (right)
    this->ChartPrivate->axes[2]->SetPoint1(this->Point2[0], this->Point1[1]);
    this->ChartPrivate->axes[2]->SetPoint2(this->Point2[0], this->Point2[1]);
    // X axis (top)
    this->ChartPrivate->axes[3]->SetPoint1(this->Point1[0], this->Point2[1]);
    this->ChartPrivate->axes[3]->SetPoint2(this->Point2[0], this->Point2[1]);

    for (int i = 0; i < 4; ++i)
    {
      this->ChartPrivate->axes[i]->Update();
    }
  }
  this->SetLegendPosition(this->Legend->GetBoundingRect(painter));

  return changed;
}

//------------------------------------------------------------------------------
int vtkChartXY::GetLegendBorder(vtkContext2D* painter, int axisPosition)
{
  if (!this->Legend->GetVisible() || this->Legend->GetInline())
  {
    return 0;
  }

  vtkVector2i tileScale = this->Scene->GetLogicalTileScale();

  int padding = 10;
  vtkVector2i legendSize(0, 0);
  vtkVector2i legendAlignment(
    this->Legend->GetHorizontalAlignment(), this->Legend->GetVerticalAlignment());
  this->Legend->Update();
  vtkRectf rect = this->Legend->GetBoundingRect(painter);
  legendSize.Set(static_cast<int>(rect.GetWidth()), static_cast<int>(rect.GetHeight()));

  // Figure out the correct place and alignment based on the legend layout.
  if (axisPosition == vtkAxis::LEFT && legendAlignment.GetX() == vtkChartLegend::LEFT)
  {
    return legendSize.GetX() + padding * tileScale.GetX();
  }
  else if (axisPosition == vtkAxis::RIGHT && legendAlignment.GetX() == vtkChartLegend::RIGHT)
  {
    return legendSize.GetX() + padding * tileScale.GetX();
  }
  else if ((axisPosition == vtkAxis::TOP || axisPosition == vtkAxis::BOTTOM) &&
    (legendAlignment.GetX() == vtkChartLegend::LEFT ||
      legendAlignment.GetX() == vtkChartLegend::RIGHT))
  {
    return 0;
  }
  else if (axisPosition == vtkAxis::TOP && legendAlignment.GetY() == vtkChartLegend::TOP)
  {
    return legendSize.GetY() + padding * tileScale.GetY();
  }
  else if (axisPosition == vtkAxis::BOTTOM && legendAlignment.GetY() == vtkChartLegend::BOTTOM)
  {
    return legendSize.GetY() + padding * tileScale.GetY();
  }
  else
  {
    return 0;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::SetLegendPosition(const vtkRectf& rect)
{
  // Put the legend in the top corner of the chart
  vtkVector2f pos(0, 0);
  int padding = 5;
  vtkVector2i legendAlignment(
    this->Legend->GetHorizontalAlignment(), this->Legend->GetVerticalAlignment());

  if (legendAlignment[0] == vtkChartLegend::CUSTOM || legendAlignment[1] == vtkChartLegend::CUSTOM)
  {
    return;
  }

  if (this->Legend->GetInline())
  {
    switch (this->Legend->GetHorizontalAlignment())
    {
      case vtkChartLegend::LEFT:
        pos.SetX(this->Point1[0]);
        break;
      case vtkChartLegend::CENTER:
        pos.SetX(
          ((this->Point2[0] - this->Point1[0]) / 2.0) - rect.GetWidth() / 2.0 + this->Point1[0]);
        break;
      case vtkChartLegend::RIGHT:
      default:
        pos.SetX(this->Point2[0] - rect.GetWidth());
    }
    switch (this->Legend->GetVerticalAlignment())
    {
      case vtkChartLegend::TOP:
        pos.SetY(this->Point2[1] - rect.GetHeight());
        break;
      case vtkChartLegend::CENTER:
        pos.SetY(
          (this->Point2[1] - this->Point1[1]) / 2.0 - rect.GetHeight() / 2.0 + this->Point1[1]);
        break;
      case vtkChartLegend::BOTTOM:
      default:
        pos.SetY(this->Point1[1]);
    }
  }
  else
  {
    // Non-inline legends.
    if (legendAlignment.GetX() == vtkChartLegend::LEFT)
    {
      pos.SetX(this->Point1[0] - this->ChartPrivate->Borders[vtkAxis::LEFT] + padding);
    }
    else if (legendAlignment.GetX() == vtkChartLegend::RIGHT)
    {
      pos.SetX(
        this->Point2[0] + this->ChartPrivate->Borders[vtkAxis::RIGHT] - rect.GetWidth() - padding);
    }
    else if (legendAlignment.GetX() == vtkChartLegend::CENTER)
    {
      pos.SetX(
        ((this->Point2[0] - this->Point1[0]) / 2.0) - (rect.GetWidth() / 2.0) + this->Point1[0]);
      // Check for the special case where the legend is on the top or bottom
      if (legendAlignment.GetY() == vtkChartLegend::TOP)
      {
        pos.SetY(
          this->Point2[1] + this->ChartPrivate->Borders[vtkAxis::TOP] - rect.GetHeight() - padding);
      }
      else if (legendAlignment.GetY() == vtkChartLegend::BOTTOM)
      {
        pos.SetY(this->Point1[1] - this->ChartPrivate->Borders[vtkAxis::BOTTOM] + padding);
      }
    }
    // Vertical alignment
    if (legendAlignment.GetX() != vtkChartLegend::CENTER)
    {
      if (legendAlignment.GetY() == vtkChartLegend::TOP)
      {
        pos.SetY(this->Point2[1] - rect.GetHeight());
      }
      else if (legendAlignment.GetY() == vtkChartLegend::BOTTOM)
      {
        pos.SetY(this->Point1[1]);
      }
    }
    if (legendAlignment.GetY() == vtkChartLegend::CENTER)
    {
      pos.SetY(
        ((this->Point2[1] - this->Point1[1]) / 2.0) - (rect.GetHeight() / 2.0) + this->Point1[1]);
    }
  }

  this->Legend->SetPoint(pos);
}

//------------------------------------------------------------------------------
vtkPlot* vtkChartXY::AddPlot(int type, unsigned int blockIndex)
{
  // Use a variable to return the object created (or nullptr), this is necessary
  // as the HP compiler is broken (thinks this function does not return) and
  // the MS compiler generates a warning about unreachable code if a redundant
  // return is added at the end.
  vtkPlot* plot = nullptr;
  vtkColor3ub color = this->ChartPrivate->Colors->GetColorRepeating(
    static_cast<int>(this->ChartPrivate->plots.size()));
  switch (type)
  {
    case LINE:
    {
      vtkPlotLine* line = vtkPlotLine::New();
      line->GetPen()->SetColor(color.GetData());
      plot = line;
      break;
    }
    case POINTS:
    {
      vtkPlotPoints* points = vtkPlotPoints::New();
      points->GetPen()->SetColor(color.GetData());
      plot = points;
      break;
    }
    case BAR:
    {
      vtkPlotBar* bar = vtkPlotBar::New();
      bar->GetBrush()->SetColor(color.GetData());
      plot = bar;
      break;
    }
    case FUNCTIONALBAG:
    {
      vtkPlotFunctionalBag* bag = vtkPlotFunctionalBag::New();
      bag->GetPen()->SetColor(color.GetData());
      bag->GetBrush()->SetColor(color.GetData());
      plot = bag;
      break;
    }
    case STACKED:
    {
      vtkPlotStacked* stacked = vtkPlotStacked::New();
      stacked->SetParent(this);
      stacked->GetBrush()->SetColor(color.GetData());
      plot = stacked;
      break;
    }
    case BAG:
    {
      vtkPlotBag* bag = vtkPlotBag::New();
      bag->SetParent(this);
      bag->GetBrush()->SetColor(color.GetData());
      plot = bag;
      break;
    }
    case AREA:
    {
      vtkPlotArea* area = vtkPlotArea::New();
      area->SetParent(this);
      area->GetBrush()->SetColor(color.GetData());
      plot = area;
      break;
    }

    default:
      plot = nullptr;
  }
  if (plot)
  {
    this->AddPlot(plot, blockIndex);
    plot->Delete();
  }
  return plot;
}

//------------------------------------------------------------------------------
vtkPlot* vtkChartXY::AddPlot(int type)
{
  return this->AddPlot(type, 0);
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::AddPlot(vtkPlot* plot, unsigned int blockIndex)
{
  if (plot == nullptr)
  {
    return -1;
  }
  plot->Register(this);
  this->ChartPrivate->plots.push_back(plot);
  this->ChartPrivate->PlotCacheUpdated = false;

  vtkNew<vtkCallbackCommand> invalidateCacheCallback;
  invalidateCacheCallback->SetClientData(this->ChartPrivate);
  invalidateCacheCallback->SetCallback(vtkChartXYPrivate::InvalidateCache);
  plot->AddObserver(vtkCommand::ModifiedEvent, invalidateCacheCallback);

  vtkIdType plotIndex = static_cast<vtkIdType>(this->ChartPrivate->plots.size() - 1);
  this->SetPlotCorner(plot, 0);
  // Ensure that the bounds are recalculated
  this->PlotTransformValid = false;
  // Mark the scene as dirty
  if (this->Scene)
  {
    this->Scene->SetDirty(true);
  }

  this->ChartPrivate->MapCompositeIndexToPlots[blockIndex].emplace_back(plot);
  this->ChartPrivate->PlotCompositeIndexes[plot] = blockIndex;

  return plotIndex;
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::AddPlot(vtkPlot* plot)
{
  return this->AddPlot(plot, 0);
}

//------------------------------------------------------------------------------
bool vtkChartXY::RemovePlot(vtkIdType index)
{
  if (index < static_cast<vtkIdType>(this->ChartPrivate->plots.size()))
  {
    this->RemovePlotFromCorners(this->ChartPrivate->plots[index]);

    // Remove plot from our block search structures
    vtkPlot* plot = this->ChartPrivate->plots[index];
    const auto& flatIndex = this->ChartPrivate->PlotCompositeIndexes.find(plot);
    auto& vecPlots = this->ChartPrivate->MapCompositeIndexToPlots[flatIndex->second];
    this->ChartPrivate->PlotCompositeIndexes.erase(flatIndex);
    vecPlots.erase(std::find(vecPlots.begin(), vecPlots.end(), plot));

    // Delete plot
    plot->Delete();
    this->ChartPrivate->plots.erase(this->ChartPrivate->plots.begin() + index);

    // Ensure that the bounds are recalculated
    this->PlotTransformValid = false;
    if (this->Scene)
    {
      // Mark the scene as dirty
      this->Scene->SetDirty(true);
    }
    return true;
  }
  else
  {
    return false;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::ClearPlots()
{
  for (unsigned int i = 0; i < this->ChartPrivate->plots.size(); ++i)
  {
    this->ChartPrivate->plots[i]->Delete();
  }
  this->ChartPrivate->plots.clear();
  // Clear the corners too
  for (int i = 0; i < int(this->ChartPrivate->PlotCorners.size()); ++i)
  {
    this->ChartPrivate->PlotCorners[i]->ClearItems();
    if (i > 0)
    {
      this->ChartPrivate->Clip->RemoveItem(this->ChartPrivate->PlotCorners[i]);
    }
  }
  this->ChartPrivate->PlotCorners.resize(1);

  // Ensure that the bounds are recalculated
  this->PlotTransformValid = false;
  if (this->Scene)
  {
    // Mark the scene as dirty
    this->Scene->SetDirty(true);
  }
}

//------------------------------------------------------------------------------
vtkPlot* vtkChartXY::GetPlot(vtkIdType index)
{
  if (static_cast<vtkIdType>(this->ChartPrivate->plots.size()) > index)
  {
    return this->ChartPrivate->plots[index];
  }
  else
  {
    return nullptr;
  }
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::GetPlotIndex(vtkPlot* plot)
{
  int corner = this->GetPlotCorner(plot);
  return corner >= 0 && corner < 4 ? this->ChartPrivate->PlotCorners[corner]->GetItemIndex(plot)
                                   : static_cast<vtkIdType>(-1);
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::RaisePlot(vtkPlot* plot)
{
  vtkIdType plotIndex = this->GetPlotIndex(plot);
  int corner = this->GetPlotCorner(plot);
  if (corner < 0 || corner >= 4)
  {
    return plotIndex;
  }
  return this->ChartPrivate->PlotCorners[corner]->Raise(plotIndex);
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::StackPlotAbove(vtkPlot* plot, vtkPlot* under)
{
  vtkIdType plotIndex = this->GetPlotIndex(plot);
  vtkIdType underIndex = this->GetPlotIndex(under);
  int corner = this->GetPlotCorner(plot);
  if (corner < 0 || corner >= 4 || underIndex != this->GetPlotCorner(under))
  {
    return plotIndex;
  }
  return this->ChartPrivate->PlotCorners[corner]->StackAbove(plotIndex, underIndex);
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::LowerPlot(vtkPlot* plot)
{
  vtkIdType plotIndex = this->GetPlotIndex(plot);
  int corner = this->GetPlotCorner(plot);
  if (corner < 0 || corner >= 4)
  {
    return plotIndex;
  }
  return this->ChartPrivate->PlotCorners[corner]->Lower(plotIndex);
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::StackPlotUnder(vtkPlot* plot, vtkPlot* above)
{
  vtkIdType plotIndex = this->GetPlotIndex(plot);
  vtkIdType aboveIndex = this->GetPlotIndex(above);
  int corner = this->GetPlotCorner(plot);
  if (corner < 0 || corner >= 4 || corner != this->GetPlotCorner(above))
  {
    return plotIndex;
  }
  return this->ChartPrivate->PlotCorners[corner]->StackUnder(plotIndex, aboveIndex);
}

//------------------------------------------------------------------------------
void vtkChartXY::SetShowLegend(bool visible)
{
  this->vtkChart::SetShowLegend(visible);
  this->Legend->SetVisible(visible);
}

//------------------------------------------------------------------------------
vtkChartLegend* vtkChartXY::GetLegend()
{
  return this->Legend;
}

//------------------------------------------------------------------------------
void vtkChartXY::SetTooltip(vtkTooltipItem* tooltip)
{
  if (tooltip == this->Tooltip)
  {
    // nothing to change
    return;
  }

  if (this->Tooltip)
  {
    // remove current tooltip from scene
    this->RemoveItem(this->Tooltip);
  }

  this->Tooltip = tooltip;

  if (this->Tooltip)
  {
    // add new tooltip to scene
    this->AddItem(this->Tooltip);
  }
}

//------------------------------------------------------------------------------
vtkTooltipItem* vtkChartXY::GetTooltip()
{
  return this->Tooltip;
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::GetNumberOfPlots()
{
  return static_cast<vtkIdType>(this->ChartPrivate->plots.size());
}

//------------------------------------------------------------------------------
vtkAxis* vtkChartXY::GetAxis(int axisIndex)
{
  if (axisIndex < 4)
  {
    return this->ChartPrivate->axes[axisIndex];
  }
  else
  {
    return nullptr;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::SetAxis(int axisIndex, vtkAxis* axis)
{
  if ((axisIndex < 4) && (axisIndex >= 0))
  {
    vtkAxis* old_axis = this->ChartPrivate->axes[axisIndex];
    this->ChartPrivate->axes[axisIndex] = axis;
    this->ChartPrivate->axes[axisIndex]->SetVisible(old_axis->GetVisible());

    // remove the old axis
    this->RemoveItem(old_axis);

    this->AttachAxisRangeListener(this->ChartPrivate->axes[axisIndex]);
    this->AddItem(this->ChartPrivate->axes[axisIndex]);

    this->ChartPrivate->axes[axisIndex]->SetPosition(axisIndex);

    vtkPlotGrid* grid1 = static_cast<vtkPlotGrid*>(this->ChartPrivate->Clip->GetItem(0));
    vtkPlotGrid* grid2 = static_cast<vtkPlotGrid*>(this->ChartPrivate->Clip->GetItem(1));
    switch (axisIndex)
    {
      case vtkAxis::BOTTOM:
        grid1->SetXAxis(this->ChartPrivate->axes[vtkAxis::BOTTOM]);
        break;
      case vtkAxis::LEFT:
        grid1->SetYAxis(this->ChartPrivate->axes[vtkAxis::LEFT]);
        break;
      case vtkAxis::TOP:
        grid2->SetXAxis(this->ChartPrivate->axes[vtkAxis::TOP]);
        break;
      case vtkAxis::RIGHT:
        grid2->SetYAxis(this->ChartPrivate->axes[vtkAxis::RIGHT]);
        break;
    }
  }
}

//------------------------------------------------------------------------------
vtkIdType vtkChartXY::GetNumberOfAxes()
{
  return 4;
}

//------------------------------------------------------------------------------
void vtkChartXY::RecalculateBounds()
{
  // Ensure that the bounds are recalculated
  this->PlotTransformValid = false;
  if (this->Scene)
  {
    // Mark the scene as dirty
    this->Scene->SetDirty(true);
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::SetSelectionMethod(int method)
{
  if (method == this->SelectionMethod)
  {
    return;
  }
  if (method == vtkChart::SELECTION_PLOTS)
  {
    // Clear the selection on the plots which may be shared between all of them.
    // Now iterate through the plots to update selection data
    std::vector<vtkPlot*>::iterator it = this->ChartPrivate->plots.begin();
    for (; it != this->ChartPrivate->plots.end(); ++it)
    {
      (*it)->SetSelection(nullptr);
    }
  }
  Superclass::SetSelectionMethod(method);
}

//------------------------------------------------------------------------------
void vtkChartXY::RemovePlotSelections()
{
  std::vector<vtkPlot*>::iterator it = this->ChartPrivate->plots.begin();
  for (; it != this->ChartPrivate->plots.end(); ++it)
  {
    vtkPlot* plot = *it;
    if (!plot)
    {
      continue;
    }
    vtkNew<vtkIdTypeArray> emptySelectionArray;
    emptySelectionArray->Initialize();
    plot->SetSelection(emptySelectionArray);
  }
  this->InvokeEvent(vtkCommand::SelectionChangedEvent);
}

//------------------------------------------------------------------------------
bool vtkChartXY::Hit(const vtkContextMouseEvent& mouse)
{
  if (!this->Interactive)
  {
    return false;
  }
  vtkVector2i pos(mouse.GetScreenPos());
  if (pos[0] > this->Point1[0] && pos[0] < this->Point2[0] && pos[1] > this->Point1[1] &&
    pos[1] < this->Point2[1])
  {
    return true;
  }
  else
  {
    return false;
  }
}

//------------------------------------------------------------------------------
bool vtkChartXY::MouseEnterEvent(const vtkContextMouseEvent&)
{
  // Find the nearest point on the curves and snap to it
  this->DrawNearestPoint = true;
  return true;
}

//------------------------------------------------------------------------------
bool vtkChartXY::MouseMoveEvent(const vtkContextMouseEvent& mouse)
{
  // Iterate through each corner, and check for a nearby point
  for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
  {
    if (this->ChartPrivate->PlotCorners[i]->MouseMoveEvent(mouse))
    {
      return true;
    }
  }

  if (mouse.GetButton() == this->Actions.Pan())
  {
    // Figure out how much the mouse has moved by in plot coordinates - pan
    vtkVector2d screenPos(mouse.GetScreenPos().Cast<double>().GetData());
    vtkVector2d lastScreenPos(mouse.GetLastScreenPos().Cast<double>().GetData());
    vtkVector2d pos(0.0, 0.0);
    vtkVector2d last(0.0, 0.0);

    // Go from screen to scene coordinates to work out the delta
    vtkAxis* xAxis = this->ChartPrivate->axes[vtkAxis::BOTTOM];
    vtkAxis* yAxis = this->ChartPrivate->axes[vtkAxis::LEFT];
    vtkTransform2D* transform = this->ChartPrivate->PlotCorners[0]->GetTransform();
    transform->InverseTransformPoints(screenPos.GetData(), pos.GetData(), 1);
    transform->InverseTransformPoints(lastScreenPos.GetData(), last.GetData(), 1);
    vtkVector2d delta = last - pos;
    delta[0] /= xAxis->GetScalingFactor();
    delta[1] /= yAxis->GetScalingFactor();

    // Now move the axes and recalculate the transform
    delta[0] = delta[0] > 0 ? std::min(delta[0], xAxis->GetMaximumLimit() - xAxis->GetMaximum())
                            : std::max(delta[0], xAxis->GetMinimumLimit() - xAxis->GetMinimum());
    delta[1] = delta[1] > 0 ? std::min(delta[1], yAxis->GetMaximumLimit() - yAxis->GetMaximum())
                            : std::max(delta[1], yAxis->GetMinimumLimit() - yAxis->GetMinimum());
    xAxis->SetRange(xAxis->GetMinimum() + delta[0], xAxis->GetMaximum() + delta[0]);
    yAxis->SetRange(yAxis->GetMinimum() + delta[1], yAxis->GetMaximum() + delta[1]);

    if (this->ChartPrivate->PlotCorners.size() == 2)
    {
      // Figure out the right axis position, if greater than 2 both will be done
      // in the else if block below.
      screenPos = vtkVector2d(mouse.GetScreenPos().Cast<double>().GetData());
      lastScreenPos = vtkVector2d(mouse.GetLastScreenPos().Cast<double>().GetData());
      pos = vtkVector2d(0.0, 0.0);
      last = vtkVector2d(0.0, 0.0);
      yAxis = this->ChartPrivate->axes[vtkAxis::RIGHT];
      transform = this->ChartPrivate->PlotCorners[1]->GetTransform();
      transform->InverseTransformPoints(screenPos.GetData(), pos.GetData(), 1);
      transform->InverseTransformPoints(lastScreenPos.GetData(), last.GetData(), 1);
      delta = last - pos;
      delta[0] /= xAxis->GetScalingFactor();
      delta[1] /= yAxis->GetScalingFactor();

      // Now move the axes and recalculate the transform
      delta[1] = delta[1] > 0 ? std::min(delta[1], yAxis->GetMaximumLimit() - yAxis->GetMaximum())
                              : std::max(delta[1], yAxis->GetMinimumLimit() - yAxis->GetMinimum());
      yAxis->SetRange(yAxis->GetMinimum() + delta[1], yAxis->GetMaximum() + delta[1]);
    }
    else if (this->ChartPrivate->PlotCorners.size() > 2)
    {
      // Figure out the right and top axis positions.
      // Go from screen to scene coordinates to work out the delta
      screenPos = vtkVector2d(mouse.GetScreenPos().Cast<double>().GetData());
      lastScreenPos = vtkVector2d(mouse.GetLastScreenPos().Cast<double>().GetData());
      pos = vtkVector2d(0.0, 0.0);
      last = vtkVector2d(0.0, 0.0);
      xAxis = this->ChartPrivate->axes[vtkAxis::TOP];
      yAxis = this->ChartPrivate->axes[vtkAxis::RIGHT];
      transform = this->ChartPrivate->PlotCorners[2]->GetTransform();
      transform->InverseTransformPoints(screenPos.GetData(), pos.GetData(), 1);
      transform->InverseTransformPoints(lastScreenPos.GetData(), last.GetData(), 1);
      delta = last - pos;
      delta[0] /= xAxis->GetScalingFactor();
      delta[1] /= yAxis->GetScalingFactor();

      // Now move the axes and recalculate the transform
      delta[0] = delta[0] > 0 ? std::min(delta[0], xAxis->GetMaximumLimit() - xAxis->GetMaximum())
                              : std::max(delta[0], xAxis->GetMinimumLimit() - xAxis->GetMinimum());
      delta[1] = delta[1] > 0 ? std::min(delta[1], yAxis->GetMaximumLimit() - yAxis->GetMaximum())
                              : std::max(delta[1], yAxis->GetMinimumLimit() - yAxis->GetMinimum());
      xAxis->SetRange(xAxis->GetMinimum() + delta[0], xAxis->GetMaximum() + delta[0]);
      yAxis->SetRange(yAxis->GetMinimum() + delta[1], yAxis->GetMaximum() + delta[1]);
    }

    this->RecalculatePlotTransforms();
    // Mark the scene as dirty
    this->Scene->SetDirty(true);

    this->InvokeEvent(vtkCommand::InteractionEvent);
  }
  else if (mouse.GetButton() == this->Actions.Zoom() || mouse.GetButton() == this->Actions.Select())
  {
    this->MouseBox.SetWidth(mouse.GetPos().GetX() - this->MouseBox.GetX());
    this->MouseBox.SetHeight(mouse.GetPos().GetY() - this->MouseBox.GetY());
    // Mark the scene as dirty
    this->Scene->SetDirty(true);
  }
  else if (mouse.GetButton() == this->Actions.ZoomAxis())
  {
    vtkVector2d screenPos(mouse.GetScreenPos().Cast<double>().GetData());
    vtkVector2d lastScreenPos(mouse.GetLastScreenPos().Cast<double>().GetData());

    vtkAxis* axes[] = { this->ChartPrivate->axes[vtkAxis::BOTTOM],
      this->ChartPrivate->axes[vtkAxis::LEFT], this->ChartPrivate->axes[vtkAxis::TOP],
      this->ChartPrivate->axes[vtkAxis::RIGHT] };

    for (int i = 0; i < 4; i++)
    {
      vtkAxis* axis = axes[i];
      if (!axis)
      {
        continue;
      }

      // bottom, top -> 0, right, left -> 1
      int side = i % 2;

      // get mouse delta in the given direction for the axis
      double delta = lastScreenPos[side] - screenPos[side];
      if (std::abs(delta) == 0)
      {
        continue;
      }

      // scale and invert delta
      delta /= -100.0;

      // zoom axis range
      double min = axis->GetMinimum();
      double max = axis->GetMaximum();
      double frac = (max - min) * 0.1;
      if (frac > 0.0)
      {
        min += delta * frac;
        max -= delta * frac;
      }
      else
      {
        min -= delta * frac;
        max += delta * frac;
      }
      axis->SetRange(min, max);
      axis->RecalculateTickSpacing();
    }

    this->RecalculatePlotTransforms();

    // Mark the scene as dirty
    this->Scene->SetDirty(true);

    this->InvokeEvent(vtkCommand::InteractionEvent);
  }
  else if (mouse.GetButton() == this->Actions.SelectPolygon())
  {
    if (this->SelectionPolygon.GetNumberOfPoints() > 0)
    {
      vtkVector2f lastPoint =
        this->SelectionPolygon.GetPoint(this->SelectionPolygon.GetNumberOfPoints() - 1);

      if ((lastPoint - mouse.GetPos()).SquaredNorm() > 100)
      {
        this->SelectionPolygon.AddPoint(mouse.GetPos());
      }

      // Mark the scene as dirty
      this->Scene->SetDirty(true);
    }
  }
  else if (mouse.GetButton() == this->Actions.ClickAndDrag() && this->DragPoint &&
    (this->DragPointAlongX || this->DragPointAlongY))
  {
    // Iterate through each corner, and check for a nearby point
    std::vector<vtkContextTransform*>::iterator it = this->ChartPrivate->PlotCorners.begin();
    for (; it != this->ChartPrivate->PlotCorners.end(); ++it)
    {
      vtkContextTransform* plotCorner = *it;
      if (!plotCorner)
      {
        continue;
      }

      int items = static_cast<int>(plotCorner->GetNumberOfItems());
      if (items == 0)
      {
        continue;
      }

      vtkVector2f position;
      vtkTransform2D* transform = plotCorner->GetTransform();
      transform->InverseTransformPoints(mouse.GetPos().GetData(), position.GetData(), 1);
      for (int j = 0; j < items; ++j)
      {
        vtkPlot* plot = vtkPlot::SafeDownCast(plotCorner->GetItem(j));
        if (!plot || plot->IsA("vtkPlotBar"))
        {
          continue;
        }
        vtkIdTypeArray* selectionArray = plot->GetSelection();
        if (!selectionArray || selectionArray->GetNumberOfValues() < 1)
        {
          continue;
        }
        if (selectionArray->GetNumberOfValues() > 1)
        {
          vtkDebugMacro("Move event (Click and Drag) found more than one point to update.");
        }
        vtkIdType index = selectionArray->GetValue(0);
        if (this->DragPointAlongX)
        {
          vtkDataArray* xArray = plot->GetData()->GetInputArrayToProcess(0, plot->GetInput());
          xArray->SetVariantValue(index, position.GetX());
        }
        if (this->DragPointAlongY)
        {
          vtkDataArray* yArray = plot->GetData()->GetInputArrayToProcess(1, plot->GetInput());
          yArray->SetVariantValue(index, position.GetY());
        }
        plot->GetSelection()->Modified();
        plot->GetInput()->Modified();
        this->Scene->SetDirty(true);
      }
    }
  }
  else if (mouse.GetButton() == vtkContextMouseEvent::NO_BUTTON)
  {
    this->Scene->SetDirty(true);

    if (this->Tooltip)
    {
      this->Tooltip->SetVisible(this->LocatePointInPlots(mouse));
    }
  }

  return true;
}

//------------------------------------------------------------------------------
int vtkChartXY::LocatePointInPlot(const vtkVector2f& position, const vtkVector2f& tolerance,
  vtkVector2f& plotPos, vtkPlot* plot, vtkIdType& segmentIndex)
{
  if (plot && plot->GetVisible())
  {
    return plot->GetNearestPoint(position, tolerance, &plotPos, &segmentIndex);
  }
  return -1;
}

//------------------------------------------------------------------------------
bool vtkChartXY::LocatePointInPlots(const vtkContextMouseEvent& mouse, int invokeEvent)
{
  size_t n = this->ChartPrivate->plots.size();
  vtkVector2i pos(mouse.GetScreenPos());
  if (pos[0] > this->Point1[0] && pos[0] < this->Point2[0] && pos[1] > this->Point1[1] &&
    pos[1] < this->Point2[1] && n)
  {
    // Iterate through each corner, and check for a nearby point
    for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
    {
      int items = static_cast<int>(this->ChartPrivate->PlotCorners[i]->GetNumberOfItems());
      if (items)
      {
        vtkVector2f plotPos, position;
        vtkTransform2D* transform = this->ChartPrivate->PlotCorners[i]->GetTransform();
        transform->InverseTransformPoints(mouse.GetPos().GetData(), position.GetData(), 1);
        // Use a tolerance of +/- 5 pixels
        vtkVector2f tolerance(std::fabs(5 * (1.0 / transform->GetMatrix()->GetElement(0, 0))),
          std::fabs(5 * (1.0 / transform->GetMatrix()->GetElement(1, 1))));
        // Iterate through the visible plots and return on the first hit
        vtkIdType segmentIndex = -1;

        for (int j = items - 1; j >= 0; --j)
        {
          vtkPlot* plot = vtkPlot::SafeDownCast(this->ChartPrivate->PlotCorners[i]->GetItem(j));
          int seriesIndex = LocatePointInPlot(position, tolerance, plotPos, plot, segmentIndex);
          if (seriesIndex >= 0)
          {
            // We found a point, set up the tooltip and return. Note: we do not
            // need to apply the shift nor scale from the plot because they
            // have been applied in LocatePointInPlot() already.
            vtkVector2d plotPosd(plotPos[0], plotPos[1]);
            this->SetTooltipInfo(mouse, plotPosd, seriesIndex, plot, segmentIndex);
            if (invokeEvent >= 0)
            {
              vtkChartPlotData plotIndex;
              plotIndex.SeriesName = plot->GetLabel();
              plotIndex.Position = plotPos;
              plotIndex.ScreenPosition = mouse.GetScreenPos();
              plotIndex.Index = seriesIndex;
              // Invoke an event, with the client data supplied
              this->InvokeEvent(invokeEvent, static_cast<void*>(&plotIndex));

              if (invokeEvent == vtkCommand::SelectionChangedEvent)
              {
                // Construct a new selection with the selected point in it.
                vtkNew<vtkIdTypeArray> selectionIds;
                selectionIds->InsertNextValue(seriesIndex);
                plot->SetSelection(selectionIds);

                if (this->AnnotationLink)
                {
                  vtkChartXY::MakeSelection(this->AnnotationLink, selectionIds, plot);
                }
              }
            }
            return true;
          }
        }
      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
void vtkChartXY::SetTooltipInfo(const vtkContextMouseEvent& mouse, const vtkVector2d& plotPos,
  vtkIdType seriesIndex, vtkPlot* plot, vtkIdType segmentIndex)
{
  if (!this->Tooltip)
  {
    return;
  }

  // Have the plot generate its tooltip label
  vtkStdString tooltipLabel = plot->GetTooltipLabel(plotPos, seriesIndex, segmentIndex);

  // Set the tooltip
  this->Tooltip->SetText(tooltipLabel);
  this->Tooltip->SetPosition(mouse.GetScreenPos()[0] + 2, mouse.GetScreenPos()[1] + 2);
}

//------------------------------------------------------------------------------
bool vtkChartXY::MouseLeaveEvent(const vtkContextMouseEvent&)
{
  this->DrawNearestPoint = false;

  if (this->Tooltip)
  {
    this->Tooltip->SetVisible(false);
  }

  return true;
}

//------------------------------------------------------------------------------
bool vtkChartXY::MouseButtonPressEvent(const vtkContextMouseEvent& mouse)
{
  if (this->Tooltip)
  {
    this->Tooltip->SetVisible(false);
  }

  // Iterate through each corner, and check for a nearby point
  for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
  {
    if (this->ChartPrivate->PlotCorners[i]->MouseButtonPressEvent(mouse))
    {
      return true;
    }
  }
  if (mouse.GetButton() == this->Actions.Pan())
  {
    // The mouse panning action.
    this->MouseBox.Set(mouse.GetPos().GetX(), mouse.GetPos().GetY(), 0.0, 0.0);
    this->DrawBox = false;
    return true;
  }
  else if (mouse.GetButton() == this->Actions.Zoom() || mouse.GetButton() == this->Actions.Select())
  {
    // Selection, for now at least...
    this->MouseBox.Set(mouse.GetPos().GetX(), mouse.GetPos().GetY(), 0.0, 0.0);
    this->DrawBox = true;
    return true;
  }
  else if (mouse.GetButton() == this->Actions.ZoomAxis())
  {
    this->MouseBox.Set(mouse.GetPos().GetX(), mouse.GetPos().GetY(), 0.0, 0.0);
    this->DrawBox = false;
    return true;
  }
  else if (mouse.GetButton() == this->Actions.SelectPolygon())
  {
    this->SelectionPolygon.Clear();
    this->SelectionPolygon.AddPoint(mouse.GetPos());
    this->DrawSelectionPolygon = true;
    return true;
  }
  else if (mouse.GetButton() == this->Actions.ClickAndDrag())
  {
    this->ReleasePlotSelections();
    this->DragPoint = this->LocatePointInPlots(mouse, vtkCommand::SelectionChangedEvent);
    this->InvokeEvent(vtkCommand::SelectionChangedEvent);
    return true;
  }
  else if (mouse.GetButton() == this->ActionsClick.Select() ||
    mouse.GetButton() == this->ActionsClick.Notify())
  {
    return true;
  }
  else
  {
    return false;
  }
}

//------------------------------------------------------------------------------
bool vtkChartXY::MouseButtonReleaseEvent(const vtkContextMouseEvent& mouse)
{
  // Iterate through each corner, and check for a nearby point
  for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
  {
    if (this->ChartPrivate->PlotCorners[i]->MouseButtonReleaseEvent(mouse))
    {
      return true;
    }
  }

  // Check single action click interaction/selection
  // First check that the selection actions are invalid or it is a pan selection
  this->MouseBox.SetWidth(mouse.GetPos().GetX() - this->MouseBox.GetX());
  this->MouseBox.SetHeight(mouse.GetPos().GetY() - this->MouseBox.GetY());
  bool isActionSelectInvalid = fabs(this->MouseBox.GetWidth()) < 0.5 &&
    fabs(this->MouseBox.GetHeight()) < 0.5 && mouse.GetButton() == this->Actions.Select();
  bool isActionSelectPolygonInvalid = this->SelectionPolygon.GetNumberOfPoints() < 2 &&
    mouse.GetButton() == this->Actions.SelectPolygon();
  bool isActionPan = mouse.GetButton() == this->Actions.Pan();

  if (isActionSelectInvalid || isActionSelectPolygonInvalid || isActionPan)
  {
    this->MouseBox.SetWidth(0.0);
    this->MouseBox.SetHeight(0.0);
    this->SelectionPolygon.Clear();
    this->DrawBox = false;
    this->DrawSelectionPolygon = false;
    // Find the relative interaction/selection point
    if (mouse.GetButton() == this->ActionsClick.Notify())
    {
      this->LocatePointInPlots(mouse, vtkCommand::InteractionEvent);
    }
    if (mouse.GetButton() == this->ActionsClick.Select())
    {
      this->LocatePointInPlots(mouse, vtkCommand::SelectionChangedEvent);
      this->InvokeEvent(vtkCommand::SelectionChangedEvent);
    }
    if (mouse.GetButton() != this->ActionsClick.Notify() &&
      mouse.GetButton() != this->ActionsClick.Select())
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  if (mouse.GetButton() == this->Actions.Select() ||
    mouse.GetButton() == this->Actions.SelectPolygon())
  {
    // Modifiers or selection modes can affect how selection is performed.
    int selectionMode = vtkChartXY::GetMouseSelectionMode(mouse, this->SelectionMode);
    bool polygonMode(mouse.GetButton() == this->Actions.SelectPolygon());
    this->Scene->SetDirty(true);

    // Update the polygon or box with the last mouse position.
    if (polygonMode)
    {
      this->SelectionPolygon.AddPoint(mouse.GetPos());
      this->DrawSelectionPolygon = false;
    }
    else
    {
      this->MouseBox.SetWidth(mouse.GetPos().GetX() - this->MouseBox.GetX());
      this->MouseBox.SetHeight(mouse.GetPos().GetY() - this->MouseBox.GetY());
      this->DrawBox = false;
    }

    // Check whether we have a valid selection area, exit early if not.
    if (polygonMode && this->SelectionPolygon.GetNumberOfPoints() < 3)
    {
      // There is no polygon to select points in.
      this->SelectionPolygon.Clear();
      return true;
    }
    else if (fabs(this->MouseBox.GetWidth()) < 0.5 || fabs(this->MouseBox.GetHeight()) < 0.5)
    {
      // The box is too small, and no useful selection can be made.
      this->MouseBox.SetWidth(0.0);
      this->MouseBox.SetHeight(0.0);
      return true;
    }

    // Iterate through the plots and build a selection. Two main behaviors are
    // supported - row-based selections add all rows from all plots and set that
    // as the selection, plot-based selections create a selection node for each
    // plot.
    if (this->SelectionMethod == vtkChart::SELECTION_ROWS)
    {
      MapIndexToIds oldSelection;
      MapIndexToIds accumulateSelection;
      // There is only one global selection, we build up a union of all rows
      // selected in all charts and set that on all plots.
      for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
      {
        int items = static_cast<int>(this->ChartPrivate->PlotCorners[i]->GetNumberOfItems());
        if (items)
        {
          vtkTransform2D* transform = this->ChartPrivate->PlotCorners[i]->GetTransform();
          vtkVector2f min;
          vtkVector2f max;
          vtkContextPolygon polygon;
          this->TransformBoxOrPolygon(polygonMode, transform, mouse.GetPos(), min, max, polygon);

          // Iterate through the plots and create the selection.
          for (int j = 0; j < items; ++j)
          {
            vtkPlot* plot = vtkPlot::SafeDownCast(this->ChartPrivate->PlotCorners[i]->GetItem(j));
            if (plot && plot->GetVisible() && plot->GetSelectable())
            {
              const auto findPlot = this->ChartPrivate->PlotCompositeIndexes.find(plot);
              // Skipping unrelated plot
              if (findPlot == this->ChartPrivate->PlotCompositeIndexes.end())
              {
                continue;
              }
              const unsigned int flatIndex = findPlot->second;

              // Build old selection (there is only really one old selection in this mode)
              if (i == 0)
              {
                const auto findOld = oldSelection.find(flatIndex);
                if (findOld == oldSelection.end())
                {
                  auto array = vtkSmartPointer<vtkIdTypeArray>::New();
                  array->DeepCopy(plot->GetSelection());
                  oldSelection.insert({ flatIndex, array });
                }
                else
                {
                  vtkChartXY::BuildSelection(nullptr, vtkContextScene::SELECTION_ADDITION,
                    findOld->second, plot->GetSelection(), nullptr);
                }
              }

              // Populate the selection using the appropriate shape.
              if (polygonMode)
              {
                plot->SelectPointsInPolygon(polygon);
              }
              else
              {
                plot->SelectPoints(min, max);
              }

              // Accumulate the selection in each plot and block.
              auto findSel = accumulateSelection.find(flatIndex);
              if (findSel == accumulateSelection.end())
              {
                findSel =
                  accumulateSelection.insert({ flatIndex, vtkSmartPointer<vtkIdTypeArray>::New() })
                    .first;
              }
              vtkChartXY::BuildSelection(nullptr, vtkContextScene::SELECTION_ADDITION,
                findSel->second.GetPointer(), plot->GetSelection(), nullptr);
            }
          }
        }
      }

      // Now add the accumulated selection to the old selection.
      vtkChartXY::BuildSelection(selectionMode, accumulateSelection, oldSelection);
      vtkChartXY::MakeSelection(this->AnnotationLink, accumulateSelection);
    }
    else if (this->SelectionMethod == vtkChart::SELECTION_PLOTS)
    {
      vtkNew<vtkIdTypeArray> oldSelection;
      vtkNew<vtkIdTypeArray> accumulateSelection;
      // We are performing plot based selections.
      for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
      {
        int items = static_cast<int>(this->ChartPrivate->PlotCorners[i]->GetNumberOfItems());
        if (items)
        {
          vtkTransform2D* transform = this->ChartPrivate->PlotCorners[i]->GetTransform();
          vtkVector2f min;
          vtkVector2f max;
          vtkContextPolygon polygon;
          this->TransformBoxOrPolygon(polygonMode, transform, mouse.GetPos(), min, max, polygon);

          for (int j = 0; j < items; ++j)
          {
            vtkPlot* plot = vtkPlot::SafeDownCast(this->ChartPrivate->PlotCorners[i]->GetItem(j));
            if (plot && plot->GetVisible() && plot->GetSelectable())
            {
              oldSelection->DeepCopy(plot->GetSelection());
              // Populate the selection using the appropriate shape.
              if (polygonMode)
              {
                plot->SelectPointsInPolygon(polygon);
              }
              else
              {
                plot->SelectPoints(min, max);
              }

              // Combine the selection in this plot with any previous selection.
              vtkChartXY::BuildSelection(
                this->AnnotationLink, selectionMode, plot->GetSelection(), oldSelection, plot);
            }
          }
        }
      }
    }
    else if (this->SelectionMethod == vtkChart::SELECTION_COLUMNS)
    {
      vtkNew<vtkIdTypeArray> oldSelection;
      vtkNew<vtkIdTypeArray> accumulateSelection;
      if (this->AnnotationLink)
      {
        this->AnnotationLink->Update();
        vtkSelection* selection =
          vtkSelection::SafeDownCast(this->AnnotationLink->GetOutputDataObject(2));
        vtkSelectionNode* node =
          selection->GetNumberOfNodes() > 0 ? selection->GetNode(0) : nullptr;
        if (node)
        {
          oldSelection->DeepCopy(vtkArrayDownCast<vtkIdTypeArray>(node->GetSelectionList()));
        }
      }
      vtkNew<vtkIdTypeArray> plotSelection;
      // We are performing plot based selections.
      for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
      {
        int items = static_cast<int>(this->ChartPrivate->PlotCorners[i]->GetNumberOfItems());
        if (items)
        {
          vtkTransform2D* transform = this->ChartPrivate->PlotCorners[i]->GetTransform();
          vtkVector2f min;
          vtkVector2f max;
          vtkContextPolygon polygon;
          this->TransformBoxOrPolygon(polygonMode, transform, mouse.GetPos(), min, max, polygon);

          for (int j = 0; j < items; ++j)
          {
            vtkPlot* plot = vtkPlot::SafeDownCast(this->ChartPrivate->PlotCorners[i]->GetItem(j));
            if (plot && plot->GetVisible() && plot->GetSelectable())
            {
              bool selected = false;
              // Populate the selection using the appropriate shape.
              if (polygonMode)
              {
                selected = plot->SelectPointsInPolygon(polygon);
              }
              else
              {
                selected = plot->SelectPoints(min, max);
              }
              if (selected)
              {
                int idx = 1; // y
                vtkAbstractArray* column =
                  plot->GetData()->GetInputAbstractArrayToProcess(idx, plot->GetInput());
                int columnID = -1;
                plot->GetInput()->GetRowData()->GetAbstractArray(column->GetName(), columnID);
                if (plotSelection->GetNumberOfTuples() != column->GetNumberOfTuples())
                {
                  plotSelection->SetNumberOfTuples(0);
                  for (vtkIdType k = 0; k < column->GetNumberOfTuples(); ++k)
                  {
                    plotSelection->InsertNextValue(k);
                  }
                }
                plot->SetSelection(plotSelection);
                accumulateSelection->InsertNextValue(columnID);
              }
            }
          }
        }
      }
      vtkIdType* ptrSelection =
        reinterpret_cast<vtkIdType*>(accumulateSelection->GetVoidPointer(0));
      std::sort(ptrSelection, ptrSelection + accumulateSelection->GetNumberOfTuples());
      // Now add the accumulated selection to the old selection
      vtkChartXY::BuildSelection(
        this->AnnotationLink, selectionMode, accumulateSelection, oldSelection, nullptr);
    }

    this->InvokeEvent(vtkCommand::SelectionChangedEvent);
    this->MouseBox.SetWidth(0.0);
    this->MouseBox.SetHeight(0.0);
    this->SelectionPolygon.Clear();
    return true;
  }
  else if (mouse.GetButton() == this->Actions.Zoom())
  {
    // Check whether a valid zoom box was drawn
    if (fabs(this->MouseBox.GetWidth()) < 0.5 || fabs(this->MouseBox.GetHeight()) < 0.5)
    {
      // Invalid box size - do nothing
      this->MouseBox.SetWidth(0.0);
      this->MouseBox.SetHeight(0.0);
      this->DrawBox = false;
      return true;
    }

    // Zoom into the chart by the specified amount, and recalculate the bounds
    vtkVector2f point2(mouse.GetPos());

    this->ZoomInAxes(this->ChartPrivate->axes[vtkAxis::BOTTOM],
      this->ChartPrivate->axes[vtkAxis::LEFT], this->MouseBox.GetData(), point2.GetData());
    this->ZoomInAxes(this->ChartPrivate->axes[vtkAxis::TOP],
      this->ChartPrivate->axes[vtkAxis::RIGHT], this->MouseBox.GetData(), point2.GetData());

    this->RecalculatePlotTransforms();
    this->MouseBox.SetWidth(0.0);
    this->MouseBox.SetHeight(0.0);
    this->DrawBox = false;
    // Mark the scene as dirty
    this->Scene->SetDirty(true);
    this->InvokeEvent(vtkCommand::InteractionEvent);
    return true;
  }
  else if (mouse.GetButton() == this->Actions.ZoomAxis())
  {
    return true;
  }
  else if (mouse.GetButton() == this->Actions.ClickAndDrag())
  {
    this->ReleasePlotSelections();
    this->InvokeEvent(vtkCommand::SelectionChangedEvent);
    this->DragPoint = false;
    return true;
  }
  return false;
}

void vtkChartXY::ZoomInAxes(vtkAxis* x, vtkAxis* y, float* originf, float* maxf)
{
  vtkNew<vtkTransform2D> transform;
  this->CalculateUnscaledPlotTransform(x, y, transform);
  vtkVector2d origin(originf[0], originf[1]);
  vtkVector2d max(maxf[0], maxf[1]);
  vtkVector2d torigin;
  transform->InverseTransformPoints(origin.GetData(), torigin.GetData(), 1);
  vtkVector2d tmax;
  transform->InverseTransformPoints(max.GetData(), tmax.GetData(), 1);

  // Ensure we preserve the directionality of the axes
  if (x->GetMaximum() > x->GetMinimum())
  {
    x->SetRange(
      torigin[0] < tmax[0] ? torigin[0] : tmax[0], torigin[0] > tmax[0] ? torigin[0] : tmax[0]);
  }
  else
  {
    x->SetRange(
      torigin[0] > tmax[0] ? torigin[0] : tmax[0], torigin[0] < tmax[0] ? torigin[0] : tmax[0]);
  }
  if (y->GetMaximum() > y->GetMinimum())
  {
    y->SetRange(
      torigin[1] < tmax[1] ? torigin[1] : tmax[1], torigin[1] > tmax[1] ? torigin[1] : tmax[1]);
  }
  else
  {
    y->SetRange(
      torigin[1] > tmax[1] ? torigin[1] : tmax[1], torigin[1] < tmax[1] ? torigin[1] : tmax[1]);
  }
  x->RecalculateTickSpacing();
  y->RecalculateTickSpacing();
}

//------------------------------------------------------------------------------
bool vtkChartXY::MouseWheelEvent(const vtkContextMouseEvent&, int delta)
{
  if (this->Tooltip)
  {
    this->Tooltip->SetVisible(false);
  }
  if (!this->ZoomWithMouseWheel)
  {
    return false;
  }

  // Get the bounds of each plot.
  for (int i = 0; i < 4; ++i)
  {
    vtkAxis* axis = this->ChartPrivate->axes[i];
    double min = axis->GetMinimum();
    double max = axis->GetMaximum();
    double frac = (max - min) * 0.1;
    if (frac > 0.0)
    {
      min += delta * frac;
      max -= delta * frac;
    }
    else
    {
      min -= delta * frac;
      max += delta * frac;
    }
    axis->SetRange(min, max);
    axis->RecalculateTickSpacing();
  }

  this->RecalculatePlotTransforms();

  // Mark the scene as dirty
  this->Scene->SetDirty(true);

  this->InvokeEvent(vtkCommand::InteractionEvent);

  return true;
}

//------------------------------------------------------------------------------
bool vtkChartXY::KeyPressEvent(const vtkContextKeyEvent& key)
{
  switch (key.GetKeyCode())
  {
    // Reset the chart axes
    case 'r':
    case 'R':
      this->RecalculateBounds();
      this->Scene->SetDirty(true);
  }

  return true;
}

const std::vector<vtkContextTransform*>& vtkChartXY::GetTransforms() const
{
  return this->ChartPrivate->PlotCorners;
}

//------------------------------------------------------------------------------
bool vtkChartXY::RemovePlotFromCorners(vtkPlot* plot)
{
  // We know the plot will only ever be in one of the corners
  for (size_t i = 0; i < this->ChartPrivate->PlotCorners.size(); ++i)
  {
    if (this->ChartPrivate->PlotCorners[i]->RemoveItem(plot))
    {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
inline void vtkChartXY::TransformBoxOrPolygon(bool polygonMode, vtkTransform2D* transform,
  const vtkVector2f& mousePosition, vtkVector2f& min, vtkVector2f& max, vtkContextPolygon& polygon)
{
  if (polygonMode)
  {
    vtkNew<vtkTransform2D> inverseTransform;
    inverseTransform->SetMatrix(transform->GetMatrix());
    inverseTransform->Inverse();
    polygon = this->SelectionPolygon.Transformed(inverseTransform);
  }
  else
  {
    transform->InverseTransformPoints(this->MouseBox.GetData(), min.GetData(), 1);
    transform->InverseTransformPoints(mousePosition.GetData(), max.GetData(), 1);
    // Normalize the rectangle selection area before using it.
    if (min.GetX() > max.GetX())
    {
      float tmp = min.GetX();
      min.SetX(max.GetX());
      max.SetX(tmp);
    }
    if (min.GetY() > max.GetY())
    {
      float tmp = min.GetY();
      min.SetY(max.GetY());
      max.SetY(tmp);
    }
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Axes: " << endl;
  for (int i = 0; i < 4; ++i)
  {
    this->ChartPrivate->axes[i]->PrintSelf(os, indent.GetNextIndent());
  }
  if (this->ChartPrivate)
  {
    os << indent << "Number of plots: " << this->ChartPrivate->plots.size() << endl;
    for (unsigned int i = 0; i < this->ChartPrivate->plots.size(); ++i)
    {
      os << indent << "Plot " << i << ":" << endl;
      this->ChartPrivate->plots[i]->PrintSelf(os, indent.GetNextIndent());
    }
  }
  os << indent << "ZoomWithMouseWheel: " << this->ZoomWithMouseWheel << endl;
}

//------------------------------------------------------------------------------
void vtkChartXY::MakeSelection(vtkAnnotationLink* link, vtkIdTypeArray* selectionIds, vtkPlot* plot)
{
  assert(link != nullptr && selectionIds != nullptr);

  if (plot)
  {
    // We are building up plot-based selections, using multiple nodes.
    vtkSelection* selection = link->GetCurrentSelection();
    vtkSmartPointer<vtkSelectionNode> node;
    for (unsigned int i = 0; i < selection->GetNumberOfNodes(); ++i)
    {
      vtkSelectionNode* tmp = selection->GetNode(i);
      vtkPlot* selectionPlot =
        vtkPlot::SafeDownCast(tmp->GetProperties()->Get(vtkSelectionNode::PROP()));
      if (selectionPlot == plot)
      {
        node = tmp;
        break;
      }
    }
    if (!node)
    {
      node = vtkSmartPointer<vtkSelectionNode>::New();
      selection->AddNode(node.GetPointer());
      node->SetContentType(vtkSelectionNode::INDICES);
      node->SetFieldType(vtkSelectionNode::POINT);
      node->GetProperties()->Set(vtkSelectionNode::PROP(), plot);
      node->GetProperties()->Set(vtkSelectionNode::SOURCE(), plot->GetInput());
    }
    node->SetSelectionList(selectionIds);
  }
  else
  {
    // Use a simple single selection node layout, remove previous selections.
    vtkNew<vtkSelection> selection;
    vtkNew<vtkSelectionNode> node;
    selection->AddNode(node);
    node->SetContentType(vtkSelectionNode::INDICES);
    node->SetFieldType(vtkSelectionNode::POINT);
    node->SetSelectionList(selectionIds);
    link->SetCurrentSelection(selection);
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::MakeSelection(vtkAnnotationLink* link, const MapIndexToIds& inSelection)
{
  if (!link)
  {
    return;
  }

  vtkNew<vtkSelection> selection;

  // If only one key and this key is 0, we consider input is not a composite dataset and we
  // handle the selection without COMPOSITE_INDEX
  if (inSelection.size() == 1 && inSelection.count(0) > 0)
  {
    vtkNew<vtkSelectionNode> node;
    selection->AddNode(node);
    node->SetContentType(vtkSelectionNode::INDICES);
    node->SetFieldType(vtkSelectionNode::POINT);
    node->SetSelectionList(inSelection.at(0));
  }
  else
  {
    for (const auto& pair : inSelection)
    {
      if (pair.second->GetNumberOfValues() > 0)
      {
        vtkNew<vtkSelectionNode> node;
        node->SetContentType(vtkSelectionNode::INDICES);
        node->SetFieldType(vtkSelectionNode::POINT);
        node->GetProperties()->Set(vtkSelectionNode::COMPOSITE_INDEX(), pair.first);
        node->SetSelectionList(pair.second);

        selection->AddNode(node);
      }
    }
  }

  link->SetCurrentSelection(selection);
}

//------------------------------------------------------------------------------
void vtkChartXY::MinusSelection(vtkIdTypeArray* selection, vtkIdTypeArray* oldSelection)
{
  // We rely on the selection id arrays being sorted.
  std::vector<vtkIdType> output;
  vtkIdType* ptrSelection = static_cast<vtkIdType*>(selection->GetVoidPointer(0));
  vtkIdType* ptrOldSelection = static_cast<vtkIdType*>(oldSelection->GetVoidPointer(0));
  vtkIdType oldSize = oldSelection->GetNumberOfTuples();
  vtkIdType size = selection->GetNumberOfTuples();
  vtkIdType iOld = 0;

  vtkIdType i = 0;
  while (i < size && iOld < oldSize)
  {
    if (ptrSelection[i] > ptrOldSelection[iOld]) // Skip the value.
    {
      output.push_back(ptrOldSelection[iOld++]);
    }
    else if (ptrSelection[i] == ptrOldSelection[iOld]) // Match - remove.
    {
      ++i;
      ++iOld;
    }
    else if (ptrSelection[i] < ptrOldSelection[iOld]) // Add the new value.
    {
      ++i;
    }
  }

  while (iOld < oldSize)
  {
    output.push_back(ptrOldSelection[iOld++]);
  }
  selection->SetNumberOfTuples(static_cast<vtkIdType>(output.size()));
  ptrSelection = static_cast<vtkIdType*>(selection->GetVoidPointer(0));
  for (std::vector<vtkIdType>::iterator it = output.begin(); it != output.end();
       ++it, ++ptrSelection)
  {
    *ptrSelection = *it;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::AddSelection(vtkIdTypeArray* selection, vtkIdTypeArray* oldSelection)
{
  // Add all unique array indices to create a new combined array.
  vtkIdType* ptrSelection = static_cast<vtkIdType*>(selection->GetVoidPointer(0));
  vtkIdType* ptrOldSelection = static_cast<vtkIdType*>(oldSelection->GetVoidPointer(0));
  std::vector<vtkIdType> output(selection->GetNumberOfTuples() + oldSelection->GetNumberOfTuples());
  std::vector<vtkIdType>::iterator it;
  it = std::set_union(ptrSelection, ptrSelection + selection->GetNumberOfTuples(), ptrOldSelection,
    ptrOldSelection + oldSelection->GetNumberOfTuples(), output.begin());
  int newSize = static_cast<int>(it - output.begin());
  selection->SetNumberOfTuples(newSize);
  ptrSelection = static_cast<vtkIdType*>(selection->GetVoidPointer(0));
  for (std::vector<vtkIdType>::iterator i = output.begin(); i != it; ++i, ++ptrSelection)
  {
    *ptrSelection = *i;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::ToggleSelection(vtkIdTypeArray* selection, vtkIdTypeArray* oldSelection)
{
  // We rely on the selection id arrays being sorted.
  std::vector<vtkIdType> output;
  vtkIdType* ptrSelection = static_cast<vtkIdType*>(selection->GetVoidPointer(0));
  vtkIdType* ptrOldSelection = static_cast<vtkIdType*>(oldSelection->GetVoidPointer(0));
  vtkIdType oldSize = oldSelection->GetNumberOfTuples();
  vtkIdType size = selection->GetNumberOfTuples();
  vtkIdType i = 0;
  vtkIdType iOld = 0;
  while (i < size && iOld < oldSize)
  {
    if (ptrSelection[i] > ptrOldSelection[iOld]) // Retain the value.
    {
      output.push_back(ptrOldSelection[iOld++]);
    }
    else if (ptrSelection[i] == ptrOldSelection[iOld]) // Match - toggle.
    {
      ++i;
      ++iOld;
    }
    else if (ptrSelection[i] < ptrOldSelection[iOld]) // Add the new value.
    {
      output.push_back(ptrSelection[i++]);
    }
  }
  while (i < size)
  {
    output.push_back(ptrSelection[i++]);
  }
  while (iOld < oldSize)
  {
    output.push_back(ptrOldSelection[iOld++]);
  }
  selection->SetNumberOfTuples(static_cast<vtkIdType>(output.size()));
  ptrSelection = static_cast<vtkIdType*>(selection->GetVoidPointer(0));
  for (std::vector<vtkIdType>::iterator it = output.begin(); it != output.end();
       ++it, ++ptrSelection)
  {
    *ptrSelection = *it;
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::BuildSelection(vtkAnnotationLink* link, int selectionMode,
  vtkIdTypeArray* plotSelection, vtkIdTypeArray* oldSelection, vtkPlot* plot)
{
  if (!plotSelection || !oldSelection)
  {
    return;
  }

  // Build a selection and set it on the annotation link if not null.
  switch (selectionMode)
  {
    case vtkContextScene::SELECTION_ADDITION:
      AddSelection(plotSelection, oldSelection);
      break;
    case vtkContextScene::SELECTION_SUBTRACTION:
      MinusSelection(plotSelection, oldSelection);
      break;
    case vtkContextScene::SELECTION_TOGGLE:
      ToggleSelection(plotSelection, oldSelection);
      break;
    case vtkContextScene::SELECTION_DEFAULT:
    default:
      // Nothing necessary - overwrite the old selection.
      break;
  }

  if (link)
  {
    MakeSelection(link, plotSelection, plot);
  }
}

//------------------------------------------------------------------------------
void vtkChartXY::BuildSelection(
  int selectionMode, MapIndexToIds& selection, const MapIndexToIds& oldSelection)
{
  // Some set of keys useful for processing our selections
  // Having a map of indices here is kind of overkill since we don't really need the values
  // but it is necessary for doing the intersection of keys in a cross-plateform way
  MapIndexToIds intersection;
  MapIndexToIds uniqueOldSelection;
  MapIndexToIds uniqueSelection;
  using MapPair = std::pair<unsigned int, vtkSmartPointer<vtkIdTypeArray>>;
  const auto compKey = [](const MapPair& p1, const MapPair& p2) { return p1.first < p2.first; };
  std::set_intersection(selection.begin(), selection.end(), oldSelection.begin(),
    oldSelection.end(), std::inserter(intersection, intersection.begin()), compKey);
  std::set_difference(oldSelection.begin(), oldSelection.end(), selection.begin(), selection.end(),
    std::inserter(uniqueOldSelection, uniqueOldSelection.begin()), compKey);

  switch (selectionMode)
  {
    case vtkContextScene::SELECTION_ADDITION:
      for (const auto& pair : intersection)
      {
        AddSelection(selection.at(pair.first), oldSelection.at(pair.first));
      }
      for (const auto& pair : uniqueOldSelection)
      {
        selection.insert(pair);
      }
      break;

    case vtkContextScene::SELECTION_SUBTRACTION:
      for (const auto& pair : intersection)
      {
        MinusSelection(selection.at(pair.first), oldSelection.at(pair.first));
      }
      for (const auto& pair : uniqueOldSelection)
      {
        selection.insert(pair);
      }
      // Remove selection not affecting old selected blocks because we're substracting
      std::set_difference(selection.begin(), selection.end(), oldSelection.begin(),
        oldSelection.end(), std::inserter(uniqueSelection, uniqueSelection.begin()), compKey);
      for (const auto& pair : uniqueSelection)
      {
        selection.erase(pair.first);
      }
      break;

    case vtkContextScene::SELECTION_TOGGLE:
      for (const auto& pair : intersection)
      {
        ToggleSelection(selection.at(pair.first), oldSelection.at(pair.first));
      }
      for (const auto& pair : uniqueOldSelection)
      {
        selection.insert(pair);
      }
      break;

    case vtkContextScene::SELECTION_DEFAULT:
    default:
      // Nothing necessary - overwrite the old selection.
      break;
  }
}

//------------------------------------------------------------------------------
int vtkChartXY::GetMouseSelectionMode(const vtkContextMouseEvent& mouse, int selectionMode)
{
  // Mouse modifiers override the current selection mode.
  if (mouse.GetModifiers() & vtkContextMouseEvent::SHIFT_MODIFIER &&
    mouse.GetModifiers() & vtkContextMouseEvent::CONTROL_MODIFIER)
  {
    return vtkContextScene::SELECTION_TOGGLE;
  }
  else if (mouse.GetModifiers() & vtkContextMouseEvent::CONTROL_MODIFIER)
  {
    return vtkContextScene::SELECTION_ADDITION;
  }
  else if (mouse.GetModifiers() & vtkContextMouseEvent::SHIFT_MODIFIER)
  {
    return vtkContextScene::SELECTION_SUBTRACTION;
  }
  return selectionMode;
}
