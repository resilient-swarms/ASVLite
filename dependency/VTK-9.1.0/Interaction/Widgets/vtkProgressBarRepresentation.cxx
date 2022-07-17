/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkProgressBarRepresentation.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

    This software is distributed WITHOUT ANY WARRANTY; without even
    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
    PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkProgressBarRepresentation.h"

#include "vtkActor2D.h"
#include "vtkCellArray.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper2D.h"
#include "vtkPropCollection.h"
#include "vtkProperty2D.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkUnsignedCharArray.h"

vtkStandardNewMacro(vtkProgressBarRepresentation);

vtkProgressBarRepresentation::vtkProgressBarRepresentation()
{
  this->ProgressRate = 0;
  this->ProgressBarColor[0] = 0;
  this->ProgressBarColor[1] = 1;
  this->ProgressBarColor[2] = 0;
  this->BackgroundColor[0] = 1;
  this->BackgroundColor[1] = 1;
  this->BackgroundColor[2] = 1;
  this->DrawBackground = true;
  this->DrawFrame = true;
  this->Padding[0] = 0.017;
  this->Padding[1] = 0.1;

  // Set up the geometry
  double size[2];
  this->GetSize(size);
  this->Position2Coordinate->SetValue(0.48 * size[0], 0.08 * size[1]);
  this->ProportionalResizeOff();
  this->Moving = 1;
  this->SetShowBorder(vtkBorderRepresentation::BORDER_ACTIVE);

  // Create the geometry in canonical coordinates
  this->Points = vtkPoints::New();
  this->Points->SetDataTypeToDouble();
  this->Points->SetNumberOfPoints(8);
  this->Points->SetPoint(0, this->Padding[0], this->Padding[1], 0.0);
  this->Points->SetPoint(1, this->Padding[0], 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(2, 1.0 - this->Padding[0], 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(3, 1.0 - this->Padding[0], this->Padding[1], 0.0);
  double progressPoint = this->Padding[0] + (this->ProgressRate * (1.0 - 2.0 * this->Padding[0]));
  this->Points->SetPoint(4, this->Padding[0], this->Padding[1], 0.0);
  this->Points->SetPoint(5, this->Padding[0], 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(6, progressPoint, 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(7, progressPoint, this->Padding[1], 0.0);

  // Progress bar
  vtkNew<vtkCellArray> polys;
  vtkIdType polysIds[4] = { 4, 5, 6, 7 };
  polys->InsertNextCell(4, polysIds);

  vtkNew<vtkPolyData> polydata;
  polydata->SetPoints(this->Points);
  polydata->SetPolys(polys);

  // Create cell data to color cells
  this->ProgressBarData = vtkUnsignedCharArray::New();
  this->ProgressBarData->SetName("Color");
  this->ProgressBarData->SetNumberOfComponents(3);
  this->ProgressBarData->SetNumberOfTuples(8);
  polydata->GetPointData()->SetScalars(this->ProgressBarData);

  // Add a transform to position progress bar
  // and a mapper and actor
  vtkNew<vtkTransformPolyDataFilter> transformFilter;
  transformFilter->SetTransform(this->BWTransform);
  transformFilter->SetInputData(polydata);
  vtkNew<vtkPolyDataMapper2D> mapper;
  mapper->SetInputConnection(transformFilter->GetOutputPort());
  this->Property = vtkProperty2D::New();
  this->Actor = vtkActor2D::New();
  this->Actor->SetMapper(mapper);
  this->Actor->SetProperty(this->Property);

  // Add transform, mapper and actor
  // Frame
  vtkNew<vtkCellArray> lines;
  vtkIdType linesIds[5] = { 0, 1, 2, 3, 0 };
  lines->InsertNextCell(5, linesIds);

  vtkNew<vtkPolyData> framePolydata;
  framePolydata->SetPoints(this->Points);
  framePolydata->SetLines(lines);
  vtkNew<vtkTransformPolyDataFilter> frameTransformFilter;
  frameTransformFilter->SetTransform(this->BWTransform);
  frameTransformFilter->SetInputData(framePolydata);
  vtkNew<vtkPolyDataMapper2D> frameMapper;
  frameMapper->SetInputConnection(frameTransformFilter->GetOutputPort());
  this->FrameActor = vtkActor2D::New();
  this->FrameActor->SetMapper(frameMapper);
  this->FrameActor->SetProperty(this->Property);

  // Background cell
  vtkNew<vtkCellArray> background;
  background->InsertNextCell(4, linesIds);

  // Background polydata
  vtkNew<vtkPolyData> backgroundPolydata;
  backgroundPolydata->SetPoints(this->Points);
  backgroundPolydata->SetPolys(background);

  // first four points of ProgressBarData are the background
  // so we use the same array (which is good as we are using the
  // same points and there are 8 of them so we need 8 colors
  // anyhow even though our cells only use the first 4
  backgroundPolydata->GetPointData()->SetScalars(this->ProgressBarData);

  // Add transform, mapper and actor
  vtkNew<vtkTransformPolyDataFilter> backgroundTransformFilter;
  backgroundTransformFilter->SetTransform(this->BWTransform);
  backgroundTransformFilter->SetInputData(backgroundPolydata);
  vtkNew<vtkPolyDataMapper2D> backgroundMapper;
  backgroundMapper->SetInputConnection(backgroundTransformFilter->GetOutputPort());
  this->BackgroundActor = vtkActor2D::New();
  this->BackgroundActor->SetMapper(backgroundMapper);
}

//------------------------------------------------------------------------------
vtkProgressBarRepresentation::~vtkProgressBarRepresentation()
{
  this->Points->Delete();
  this->ProgressBarData->Delete();
  this->Property->Delete();
  this->Actor->Delete();
  this->BackgroundActor->Delete();
  this->FrameActor->Delete();
}

//------------------------------------------------------------------------------
void vtkProgressBarRepresentation::BuildRepresentation()
{
  // Reposition progress bar points
  this->Points->SetPoint(0, this->Padding[0], this->Padding[1], 0.0);
  this->Points->SetPoint(1, this->Padding[0], 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(2, 1.0 - this->Padding[0], 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(3, 1.0 - this->Padding[0], this->Padding[1], 0.0);
  double progressPoint = this->Padding[0] + (this->ProgressRate * (1.0 - 2.0 * this->Padding[0]));
  this->Points->SetPoint(4, this->Padding[0], this->Padding[1], 0.0);
  this->Points->SetPoint(5, this->Padding[0], 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(6, progressPoint, 1.0 - this->Padding[1], 0.0);
  this->Points->SetPoint(7, progressPoint, this->Padding[1], 0.0);
  this->Points->Modified();

  // Set color
  double backgroundColor[3] = { this->BackgroundColor[0] * 255, this->BackgroundColor[1] * 255,
    this->BackgroundColor[2] * 255 };
  double progressBarColor[3] = { this->ProgressBarColor[0] * 255, this->ProgressBarColor[1] * 255,
    this->ProgressBarColor[2] * 255 };
  for (int i = 0; i < 4; i++)
  {
    this->ProgressBarData->SetTuple(i, backgroundColor);
    this->ProgressBarData->SetTuple(i + 4, progressBarColor);
  }

  // Note that the transform is updated by the superclass
  this->Superclass::BuildRepresentation();
}

//------------------------------------------------------------------------------
void vtkProgressBarRepresentation::GetActors2D(vtkPropCollection* pc)
{
  if (this->DrawBackground)
  {
    pc->AddItem(this->BackgroundActor);
  }
  if (this->DrawFrame)
  {
    pc->AddItem(this->FrameActor);
  }
  pc->AddItem(this->Actor);
  this->Superclass::GetActors2D(pc);
}

//------------------------------------------------------------------------------
void vtkProgressBarRepresentation::ReleaseGraphicsResources(vtkWindow* w)
{
  if (this->DrawBackground)
  {
    this->BackgroundActor->ReleaseGraphicsResources(w);
  }
  if (this->DrawFrame)
  {
    this->FrameActor->ReleaseGraphicsResources(w);
  }
  this->Actor->ReleaseGraphicsResources(w);
  this->Superclass::ReleaseGraphicsResources(w);
}

//------------------------------------------------------------------------------
int vtkProgressBarRepresentation::RenderOverlay(vtkViewport* w)
{
  int count = this->Superclass::RenderOverlay(w);
  if (this->DrawBackground)
  {
    count += this->BackgroundActor->RenderOverlay(w);
  }
  if (this->DrawFrame)
  {
    count += this->FrameActor->RenderOverlay(w);
  }
  count += this->Actor->RenderOverlay(w);
  return count;
}

//------------------------------------------------------------------------------
int vtkProgressBarRepresentation::RenderOpaqueGeometry(vtkViewport* w)
{
  int count = this->Superclass::RenderOpaqueGeometry(w);
  if (this->DrawBackground)
  {
    count += this->BackgroundActor->RenderOpaqueGeometry(w);
  }
  if (this->DrawFrame)
  {
    count += this->FrameActor->RenderOpaqueGeometry(w);
  }
  count += this->Actor->RenderOpaqueGeometry(w);
  return count;
}

//------------------------------------------------------------------------------
int vtkProgressBarRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* w)
{
  int count = this->Superclass::RenderTranslucentPolygonalGeometry(w);
  if (this->DrawBackground)
  {
    count += this->BackgroundActor->RenderTranslucentPolygonalGeometry(w);
  }
  if (this->DrawFrame)
  {
    count += this->FrameActor->RenderTranslucentPolygonalGeometry(w);
  }
  count += this->Actor->RenderTranslucentPolygonalGeometry(w);
  return count;
}

//------------------------------------------------------------------------------
vtkTypeBool vtkProgressBarRepresentation::HasTranslucentPolygonalGeometry()
{
  int result = this->Superclass::HasTranslucentPolygonalGeometry();
  if (this->DrawBackground)
  {
    result |= this->BackgroundActor->HasTranslucentPolygonalGeometry();
  }
  if (this->DrawFrame)
  {
    result |= this->FrameActor->HasTranslucentPolygonalGeometry();
  }
  result |= this->Actor->HasTranslucentPolygonalGeometry();
  return result;
}

//------------------------------------------------------------------------------
void vtkProgressBarRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  if (this->Property)
  {
    os << indent << "Property:\n";
    this->Property->PrintSelf(os, indent.GetNextIndent());
  }
  else
  {
    os << indent << "Property: (none)\n";
  }
  os << indent << "ProgressRate: " << this->ProgressRate << "\n";
  os << indent << "ProgressBarColor: " << this->ProgressBarColor[0] << " "
     << this->ProgressBarColor[1] << " " << this->ProgressBarColor[2] << "\n";
  os << indent << "DrawBackground: " << this->DrawBackground << "\n";
  os << indent << "DrawFrame: " << this->DrawFrame << "\n";
  os << indent << "Padding: " << this->Padding[0] << ", " << this->Padding[1] << "\n";
  os << indent << "BackgroundColor: " << this->BackgroundColor[0] << " " << this->BackgroundColor[1]
     << " " << this->BackgroundColor[2] << "\n";
}
