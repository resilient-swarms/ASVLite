/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestUnstructuredGridGeometryFilter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// This test covers the vtkUnstructuredGridGeometryFilter class on all
// types of cells.
// The command line arguments are:
// -I        => run in interactive mode; unless this is used, the program will
//              not allow interaction and exit
// -D <path> => path to the data; the data should be in <path>/Data/

// If READ_FILE is defined, the unstructured grid is read from a file otherwise
// it is created cell by cell.
//#define READ_FILE

// If WRITE_RESULT is defined, the result of the surface filter is saved.
//#define WRITE_RESULT

// If USE_SHRINK is defined, each face is shrink to easily detect bad faces.
#define USE_SHRINK

// If FAST_GEOMETRY is defined, a vtkDataSetSurfaceFilter is used instead
// of a vtkGeometryFilter at the end of the pipeline
//#define FAST_GEOMETRY

// If USE_CULLING is defined, backface culling is used to detect any bad
// ordering of points defining a face.
#define USE_CULLING

// The configuration for the regression test is:
// USE_SHRINK and USE_CULLING

#include "vtkActor.h"
#include "vtkCellData.h"
#include "vtkCellType.h"
#include "vtkCellTypes.h"
#include "vtkLookupTable.h"
#include "vtkPolyData.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkTestUtilities.h"
#include "vtkUnstructuredGridGeometryFilter.h"
#include "vtkXMLUnstructuredGridReader.h"
#include <cassert>
#ifdef FAST_GEOMETRY
#include "vtkDataSetSurfaceFilter.h"
#else
#include "vtkGeometryFilter.h"
#endif
#include "vtkCamera.h"
#include "vtkPointData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkTimerLog.h"

#ifndef READ_FILE
#include "vtkBiQuadraticQuad.h"
#include "vtkBiQuadraticQuadraticHexahedron.h"
#include "vtkBiQuadraticQuadraticWedge.h"
#include "vtkBiQuadraticTriangle.h"
#include "vtkCubicLine.h"
#include "vtkFloatArray.h"
#include "vtkHexagonalPrism.h"
#include "vtkHexahedron.h"
#include "vtkIdTypeArray.h"
#include "vtkLine.h"
#include "vtkPentagonalPrism.h"
#include "vtkPixel.h"
#include "vtkPolyLine.h"
#include "vtkPolyVertex.h"
#include "vtkPolygon.h"
#include "vtkPolyhedron.h"
#include "vtkPyramid.h"
#include "vtkQuad.h"
#include "vtkQuadraticEdge.h"
#include "vtkQuadraticHexahedron.h"
#include "vtkQuadraticLinearQuad.h"
#include "vtkQuadraticLinearWedge.h"
#include "vtkQuadraticPyramid.h"
#include "vtkQuadraticQuad.h"
#include "vtkQuadraticTetra.h"
#include "vtkQuadraticTriangle.h"
#include "vtkQuadraticWedge.h"
#include "vtkTetra.h"
#include "vtkTriQuadraticHexahedron.h"
#include "vtkTriQuadraticPyramid.h"
#include "vtkTriangle.h"
#include "vtkTriangleStrip.h"
#include "vtkUnstructuredGrid.h"
#include "vtkVertex.h"
#include "vtkVoxel.h"
#include "vtkWedge.h"
#endif

#ifdef WRITE_RESULT
#include "vtkXMLUnstructuredGridWriter.h"
#endif // #ifdef WRITE_RESULT

#ifdef USE_SHRINK
#include "vtkShrinkFilter.h"
#endif

int TestUnstructuredGridGeometryFilter(int argc, char* argv[])
{
  // Standard rendering classes
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(renderer);
  auto iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);

#ifdef READ_FILE
  // Load the mesh geometry and data from a file
  auto reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
  char* cfname = vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/quadraticTetra01.vtu");

  reader->SetFileName(cfname);
  delete[] cfname;

  // Force reading
  reader->Update();
#else
  // Create an unstructured grid.
  auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto scalars = vtkSmartPointer<vtkFloatArray>::New();
  scalars->SetName("ramp");
  auto cellIds = vtkSmartPointer<vtkIdTypeArray>::New();
  cellIds->SetName("cellIds");

  float scalar = 0.0;
  const float scalarStep = 0.1;
  vtkIdType cellId = 0;
  double xOffset = 0.0;
  double yOffset = 0.0;
  vtkIdType pointId = 0;

  // About 60 cells.
  grid->Allocate(65, 65);

  // 0D: vertex
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto vertex = vtkSmartPointer<vtkVertex>::New();
  vertex->GetPointIds()->SetId(0, pointId);
  ++pointId;
  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(vertex->GetCellType(), vertex->GetPointIds());

  // 0D: polyvertex
  xOffset += 1.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
  polyVertex->GetPointIds()->SetNumberOfIds(2);
  polyVertex->GetPointIds()->SetId(0, pointId);
  ++pointId;
  polyVertex->GetPointIds()->SetId(1, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds());

  // 1D: line, polyline, quadratic edge and Cubic Line
  yOffset += 2.0;
  //  xOffset+=1.0;
  xOffset = 0.0;
  // 1D: line
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto line = vtkSmartPointer<vtkLine>::New();
  line->GetPointIds()->SetId(0, pointId);
  ++pointId;
  line->GetPointIds()->SetId(1, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(line->GetCellType(), line->GetPointIds());

  // 1D: polyline
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(3);
  polyLine->GetPointIds()->SetId(0, pointId);
  ++pointId;
  polyLine->GetPointIds()->SetId(1, pointId);
  ++pointId;
  polyLine->GetPointIds()->SetId(2, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polyLine->GetCellType(), polyLine->GetPointIds());

  // 1D: quadratic edge
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadEdge = vtkSmartPointer<vtkQuadraticEdge>::New();
  quadEdge->GetPointIds()->SetId(0, pointId);
  ++pointId;
  quadEdge->GetPointIds()->SetId(1, pointId);
  ++pointId;
  quadEdge->GetPointIds()->SetId(2, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadEdge->GetCellType(), quadEdge->GetPointIds());

  // 1D: CubicLine
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 3.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.25, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  points->InsertNextPoint(xOffset + 0.25, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto cubLine = vtkSmartPointer<vtkCubicLine>::New();
  cubLine->GetPointIds()->SetId(0, pointId);
  ++pointId;
  cubLine->GetPointIds()->SetId(1, pointId);
  ++pointId;
  cubLine->GetPointIds()->SetId(2, pointId);
  ++pointId;
  cubLine->GetPointIds()->SetId(3, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(cubLine->GetCellType(), cubLine->GetPointIds());

  // 2D: triangle, triangle strip, polygon (triangle, quad, pentagon, hexagon),
  // pixel, quad.
  // quadratic quad, biquadratic quad, quadratic linear quad, biquadratic Triangle

  // 2D: triangle
  yOffset += 3.0;
  xOffset = 0.0;
  //  xOffset+=2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0, pointId);
  ++pointId;
  triangle->GetPointIds()->SetId(1, pointId);
  ++pointId;
  triangle->GetPointIds()->SetId(2, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(triangle->GetCellType(), triangle->GetPointIds());

  // 2D: triangle strip
  // vertices are placed like that (first triangle is 0-1-2):
  // 0 2 4
  // 1 3 5
  // Be careful: the figure in the text book shows:
  // 1 3 5
  // 0 2 4
  // which create normals in the wrong way.

  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, -0.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 1.0, -2.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto triangleStrip = vtkSmartPointer<vtkTriangleStrip>::New();
  triangleStrip->GetPointIds()->SetNumberOfIds(6);
  int i = 0;
  while (i < 6)
  {
    triangleStrip->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(triangleStrip->GetCellType(), triangleStrip->GetPointIds());

  // 2D: polygon-triangle
  xOffset += 3.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(3);
  polygon->GetPointIds()->SetId(0, pointId);
  ++pointId;
  polygon->GetPointIds()->SetId(1, pointId);
  ++pointId;
  polygon->GetPointIds()->SetId(2, pointId);
  ++pointId;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polygon->GetCellType(), polygon->GetPointIds());

  // 2D: polygon-quad
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.8, yOffset + 0.8, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(4);
  i = 0;
  while (i < 4)
  {
    polygon->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polygon->GetCellType(), polygon->GetPointIds());

  // 2D: polygon-pentagon
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.2, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(5);
  i = 0;
  while (i < 5)
  {
    polygon->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polygon->GetCellType(), polygon->GetPointIds());

  // 2D: polygon-hexagon
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.2, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.1, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(6);
  i = 0;
  while (i < 6)
  {
    polygon->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polygon->GetCellType(), polygon->GetPointIds());

  // 2D: pixel
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto pixel = vtkSmartPointer<vtkPixel>::New();
  i = 0;
  while (i < 4)
  {
    pixel->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pixel->GetCellType(), pixel->GetPointIds());

  // 2D: quad
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.8, yOffset + 0.8, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quad = vtkSmartPointer<vtkQuad>::New();
  i = 0;
  while (i < 4)
  {
    quad->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quad->GetCellType(), quad->GetPointIds());

  // 2D: quadratic triangle
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.3, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.3, yOffset + 1.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.2, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticTriangle = vtkSmartPointer<vtkQuadraticTriangle>::New();
  i = 0;
  while (i < 6)
  {
    quadraticTriangle->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticTriangle->GetCellType(), quadraticTriangle->GetPointIds());

  // 2D: biquadratic triangle
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.3, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.3, yOffset + 1.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.2, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.2, yOffset + 0.9, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto BiQuadraticTriangle = vtkSmartPointer<vtkBiQuadraticTriangle>::New();
  i = 0;
  while (i < 7)
  {
    BiQuadraticTriangle->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(BiQuadraticTriangle->GetCellType(), BiQuadraticTriangle->GetPointIds());

  // 2D: quadratic quad
  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.8, yOffset + 0.8, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset - 0.2, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.2, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.7, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.3, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticQuad = vtkSmartPointer<vtkQuadraticQuad>::New();
  i = 0;
  while (i < 8)
  {
    quadraticQuad->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticQuad->GetCellType(), quadraticQuad->GetPointIds());

  // 2D: biquadratic quad: add a center point  vtkBiQuadraticQuad.h

  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.8, yOffset + 0.8, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset - 0.2, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.2, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.7, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.3, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.4, yOffset + 0.4, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto biquadraticQuad = vtkSmartPointer<vtkBiQuadraticQuad>::New();
  i = 0;
  while (i < 9)
  {
    biquadraticQuad->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(biquadraticQuad->GetCellType(), biquadraticQuad->GetPointIds());

  // 2D: quadratic linear quad, no center, no mid-edge on sides

  xOffset += 2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.8, yOffset + 0.8, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset - 0.2, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.7, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticLinearQuad = vtkSmartPointer<vtkQuadraticLinearQuad>::New();
  i = 0;
  while (i < 6)
  {
    quadraticLinearQuad->GetPointIds()->SetId(i, pointId);
    ++pointId;
    ++i;
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticLinearQuad->GetCellType(), quadraticLinearQuad->GetPointIds());

  // 3D: tetra, voxel, hexahedron, wedge, pyramid, pentagonal prism,
  // hexagonal prism,
  // quadratic tetra, quadratic hexa, quadratic wedge, quadratic pyramid,
  // triquadratic hexa, quadratic linear wedge, biquadratic quadratic wedge, biquadratic pyramid,
  // biquadratic quadratic hexa

  // 3D: tetra: 2 tetra with one common face
  yOffset += 3.0;
  xOffset = 0.0;
  //  xOffset+=2.0;
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.3, -2.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto tetra = vtkSmartPointer<vtkTetra>::New();
  tetra->GetPointIds()->SetId(0, pointId);
  tetra->GetPointIds()->SetId(1, pointId + 1);
  tetra->GetPointIds()->SetId(2, pointId + 2);
  tetra->GetPointIds()->SetId(3, pointId + 3);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(tetra->GetCellType(), tetra->GetPointIds());

  tetra = vtkSmartPointer<vtkTetra>::New();
  tetra->GetPointIds()->SetId(0, pointId);
  tetra->GetPointIds()->SetId(1, pointId + 2);
  tetra->GetPointIds()->SetId(2, pointId + 1);
  tetra->GetPointIds()->SetId(3, pointId + 4);

  pointId += 5;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(tetra->GetCellType(), tetra->GetPointIds());

  // 3D: voxel: 2 voxels with one common face
  xOffset += 2.0;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto voxel = vtkSmartPointer<vtkVoxel>::New();
  voxel->GetPointIds()->SetId(0, pointId);
  voxel->GetPointIds()->SetId(1, pointId + 1);
  voxel->GetPointIds()->SetId(2, pointId + 2);
  voxel->GetPointIds()->SetId(3, pointId + 3);
  voxel->GetPointIds()->SetId(4, pointId + 4);
  voxel->GetPointIds()->SetId(5, pointId + 5);
  voxel->GetPointIds()->SetId(6, pointId + 6);
  voxel->GetPointIds()->SetId(7, pointId + 7);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(voxel->GetCellType(), voxel->GetPointIds());

  voxel = vtkSmartPointer<vtkVoxel>::New();
  voxel->GetPointIds()->SetId(0, pointId + 4);
  voxel->GetPointIds()->SetId(1, pointId + 5);
  voxel->GetPointIds()->SetId(2, pointId + 6);
  voxel->GetPointIds()->SetId(3, pointId + 7);
  voxel->GetPointIds()->SetId(4, pointId + 8);
  voxel->GetPointIds()->SetId(5, pointId + 9);
  voxel->GetPointIds()->SetId(6, pointId + 10);
  voxel->GetPointIds()->SetId(7, pointId + 11);

  pointId += 12;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(voxel->GetCellType(), voxel->GetPointIds());

  // 3D: hexahedron: 2 hexahedra with one common face
  xOffset += 2.0;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto hexahedron = vtkSmartPointer<vtkHexahedron>::New();
  hexahedron->GetPointIds()->SetId(0, pointId);
  hexahedron->GetPointIds()->SetId(1, pointId + 1);
  hexahedron->GetPointIds()->SetId(2, pointId + 2);
  hexahedron->GetPointIds()->SetId(3, pointId + 3);
  hexahedron->GetPointIds()->SetId(4, pointId + 4);
  hexahedron->GetPointIds()->SetId(5, pointId + 5);
  hexahedron->GetPointIds()->SetId(6, pointId + 6);
  hexahedron->GetPointIds()->SetId(7, pointId + 7);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(hexahedron->GetCellType(), hexahedron->GetPointIds());

  hexahedron = vtkSmartPointer<vtkHexahedron>::New();
  hexahedron->GetPointIds()->SetId(0, pointId + 4);
  hexahedron->GetPointIds()->SetId(1, pointId + 5);
  hexahedron->GetPointIds()->SetId(2, pointId + 6);
  hexahedron->GetPointIds()->SetId(3, pointId + 7);
  hexahedron->GetPointIds()->SetId(4, pointId + 8);
  hexahedron->GetPointIds()->SetId(5, pointId + 9);
  hexahedron->GetPointIds()->SetId(6, pointId + 10);
  hexahedron->GetPointIds()->SetId(7, pointId + 11);

  pointId += 12;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(hexahedron->GetCellType(), hexahedron->GetPointIds());

  // 3D: wedge: 3 wedges, some share a quad face, some share a triangle face
  xOffset += 2.0;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.9, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto wedge = vtkSmartPointer<vtkWedge>::New();
  wedge->GetPointIds()->SetId(0, pointId);
  wedge->GetPointIds()->SetId(1, pointId + 1);
  wedge->GetPointIds()->SetId(2, pointId + 2);
  wedge->GetPointIds()->SetId(3, pointId + 3);
  wedge->GetPointIds()->SetId(4, pointId + 4);
  wedge->GetPointIds()->SetId(5, pointId + 5);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(wedge->GetCellType(), wedge->GetPointIds());

  wedge = vtkSmartPointer<vtkWedge>::New();
  wedge->GetPointIds()->SetId(0, pointId + 3);
  wedge->GetPointIds()->SetId(1, pointId + 4);
  wedge->GetPointIds()->SetId(2, pointId + 5);
  wedge->GetPointIds()->SetId(3, pointId + 6);
  wedge->GetPointIds()->SetId(4, pointId + 7);
  wedge->GetPointIds()->SetId(5, pointId + 8);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(wedge->GetCellType(), wedge->GetPointIds());

  wedge = vtkSmartPointer<vtkWedge>::New();
  wedge->GetPointIds()->SetId(0, pointId + 2);
  wedge->GetPointIds()->SetId(1, pointId + 1);
  wedge->GetPointIds()->SetId(2, pointId + 9);
  wedge->GetPointIds()->SetId(3, pointId + 5);
  wedge->GetPointIds()->SetId(4, pointId + 4);
  wedge->GetPointIds()->SetId(5, pointId + 10);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(wedge->GetCellType(), wedge->GetPointIds());

  pointId += 11;

  // 3D: pyramid: 3 pyramids, some share the base quad face,
  // some share a triangle face
  xOffset += 2.0;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -1.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset - 1.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.9);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto pyramid = vtkSmartPointer<vtkPyramid>::New();
  pyramid->GetPointIds()->SetId(0, pointId);
  pyramid->GetPointIds()->SetId(1, pointId + 1);
  pyramid->GetPointIds()->SetId(2, pointId + 2);
  pyramid->GetPointIds()->SetId(3, pointId + 3);
  pyramid->GetPointIds()->SetId(4, pointId + 4);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());

  pyramid = vtkSmartPointer<vtkPyramid>::New();
  pyramid->GetPointIds()->SetId(0, pointId + 3);
  pyramid->GetPointIds()->SetId(1, pointId + 2);
  pyramid->GetPointIds()->SetId(2, pointId + 1);
  pyramid->GetPointIds()->SetId(3, pointId);
  pyramid->GetPointIds()->SetId(4, pointId + 5);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());

  pyramid = vtkSmartPointer<vtkPyramid>::New();
  pyramid->GetPointIds()->SetId(0, pointId + 1);
  pyramid->GetPointIds()->SetId(1, pointId + 6);
  pyramid->GetPointIds()->SetId(2, pointId + 7);
  pyramid->GetPointIds()->SetId(3, pointId + 2);
  pyramid->GetPointIds()->SetId(4, pointId + 4);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());

  pointId += 8;

  // 3D: pentagonal prism: a wedge with a pentagonal base
  // Becareful, base face ordering is different from wedge...

  xOffset += 4.0;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.5, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.5, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.5, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 15
  points->InsertNextPoint(xOffset + 2.5, yOffset + 0.75, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.5, yOffset + 2.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 2.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18
  points->InsertNextPoint(xOffset + 2.5, yOffset + 0.75, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.5, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 2.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto pentagonalPrism = vtkSmartPointer<vtkPentagonalPrism>::New();
  pentagonalPrism->GetPointIds()->SetId(0, pointId);
  pentagonalPrism->GetPointIds()->SetId(1, pointId + 4);
  pentagonalPrism->GetPointIds()->SetId(2, pointId + 3);
  pentagonalPrism->GetPointIds()->SetId(3, pointId + 2);
  pentagonalPrism->GetPointIds()->SetId(4, pointId + 1);
  pentagonalPrism->GetPointIds()->SetId(5, pointId + 5);
  pentagonalPrism->GetPointIds()->SetId(6, pointId + 9);
  pentagonalPrism->GetPointIds()->SetId(7, pointId + 8);
  pentagonalPrism->GetPointIds()->SetId(8, pointId + 7);
  pentagonalPrism->GetPointIds()->SetId(9, pointId + 6);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pentagonalPrism->GetCellType(), pentagonalPrism->GetPointIds());

  pentagonalPrism = vtkSmartPointer<vtkPentagonalPrism>::New();
  pentagonalPrism->GetPointIds()->SetId(0, pointId + 5);
  pentagonalPrism->GetPointIds()->SetId(1, pointId + 9);
  pentagonalPrism->GetPointIds()->SetId(2, pointId + 8);
  pentagonalPrism->GetPointIds()->SetId(3, pointId + 7);
  pentagonalPrism->GetPointIds()->SetId(4, pointId + 6);
  pentagonalPrism->GetPointIds()->SetId(5, pointId + 10);
  pentagonalPrism->GetPointIds()->SetId(6, pointId + 14);
  pentagonalPrism->GetPointIds()->SetId(7, pointId + 13);
  pentagonalPrism->GetPointIds()->SetId(8, pointId + 12);
  pentagonalPrism->GetPointIds()->SetId(9, pointId + 11);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pentagonalPrism->GetCellType(), pentagonalPrism->GetPointIds());

  pentagonalPrism = vtkSmartPointer<vtkPentagonalPrism>::New();
  pentagonalPrism->GetPointIds()->SetId(0, pointId + 2);
  pentagonalPrism->GetPointIds()->SetId(1, pointId + 3);
  pentagonalPrism->GetPointIds()->SetId(2, pointId + 17);
  pentagonalPrism->GetPointIds()->SetId(3, pointId + 16);
  pentagonalPrism->GetPointIds()->SetId(4, pointId + 15);
  pentagonalPrism->GetPointIds()->SetId(5, pointId + 7);
  pentagonalPrism->GetPointIds()->SetId(6, pointId + 8);
  pentagonalPrism->GetPointIds()->SetId(7, pointId + 20);
  pentagonalPrism->GetPointIds()->SetId(8, pointId + 19);
  pentagonalPrism->GetPointIds()->SetId(9, pointId + 18);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(pentagonalPrism->GetCellType(), pentagonalPrism->GetPointIds());

  pointId += 21;

  // 3D: hexagonal prism: a wedge with an hexagonal base.
  // Be careful, base face ordering is different from wedge...

  xOffset += 4.0;

  // hexagon
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.5, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // hexagon
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.5, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // hexagon
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.5, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18
  points->InsertNextPoint(xOffset + 2.5, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 3.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.5, yOffset + 1.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 1.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 22
  points->InsertNextPoint(xOffset + 2.5, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 3.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.5, yOffset + 1.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.5, yOffset + 1.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto hexagonalPrism = vtkSmartPointer<vtkHexagonalPrism>::New();
  hexagonalPrism->GetPointIds()->SetId(0, pointId);
  hexagonalPrism->GetPointIds()->SetId(1, pointId + 5);
  hexagonalPrism->GetPointIds()->SetId(2, pointId + 4);
  hexagonalPrism->GetPointIds()->SetId(3, pointId + 3);
  hexagonalPrism->GetPointIds()->SetId(4, pointId + 2);
  hexagonalPrism->GetPointIds()->SetId(5, pointId + 1);

  hexagonalPrism->GetPointIds()->SetId(6, pointId + 6);
  hexagonalPrism->GetPointIds()->SetId(7, pointId + 11);
  hexagonalPrism->GetPointIds()->SetId(8, pointId + 10);
  hexagonalPrism->GetPointIds()->SetId(9, pointId + 9);
  hexagonalPrism->GetPointIds()->SetId(10, pointId + 8);
  hexagonalPrism->GetPointIds()->SetId(11, pointId + 7);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(hexagonalPrism->GetCellType(), hexagonalPrism->GetPointIds());

  hexagonalPrism = vtkSmartPointer<vtkHexagonalPrism>::New();
  hexagonalPrism->GetPointIds()->SetId(0, pointId + 6);
  hexagonalPrism->GetPointIds()->SetId(1, pointId + 11);
  hexagonalPrism->GetPointIds()->SetId(2, pointId + 10);
  hexagonalPrism->GetPointIds()->SetId(3, pointId + 9);
  hexagonalPrism->GetPointIds()->SetId(4, pointId + 8);
  hexagonalPrism->GetPointIds()->SetId(5, pointId + 7);

  hexagonalPrism->GetPointIds()->SetId(6, pointId + 12);
  hexagonalPrism->GetPointIds()->SetId(7, pointId + 17);
  hexagonalPrism->GetPointIds()->SetId(8, pointId + 16);
  hexagonalPrism->GetPointIds()->SetId(9, pointId + 15);
  hexagonalPrism->GetPointIds()->SetId(10, pointId + 14);
  hexagonalPrism->GetPointIds()->SetId(11, pointId + 13);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(hexagonalPrism->GetCellType(), hexagonalPrism->GetPointIds());

  hexagonalPrism = vtkSmartPointer<vtkHexagonalPrism>::New();
  hexagonalPrism->GetPointIds()->SetId(0, pointId + 2);
  hexagonalPrism->GetPointIds()->SetId(1, pointId + 3);
  hexagonalPrism->GetPointIds()->SetId(2, pointId + 21);
  hexagonalPrism->GetPointIds()->SetId(3, pointId + 20);
  hexagonalPrism->GetPointIds()->SetId(4, pointId + 19);
  hexagonalPrism->GetPointIds()->SetId(5, pointId + 18);

  hexagonalPrism->GetPointIds()->SetId(6, pointId + 8);
  hexagonalPrism->GetPointIds()->SetId(7, pointId + 9);
  hexagonalPrism->GetPointIds()->SetId(8, pointId + 25);
  hexagonalPrism->GetPointIds()->SetId(9, pointId + 24);
  hexagonalPrism->GetPointIds()->SetId(10, pointId + 23);
  hexagonalPrism->GetPointIds()->SetId(11, pointId + 22);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(hexagonalPrism->GetCellType(), hexagonalPrism->GetPointIds());

  pointId += 26;

  // 3D: quadratic tetra: 2 tetra with one common face
  yOffset += 3.0;
  xOffset = 0.0;
  //  xOffset+=2.0;

  // corner points
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.3, -2.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on common face
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.6, 0.0); // y=0.5->0.6 (concave)
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.3, yOffset + 1.5, 0.0); // x=0.25->0.3 (convex)
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset - 0.2, yOffset + 1.0, 0.0); // x=0.0->-0.2 (convex)
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on other edges for the first tetra
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.3, 0.5); // y=0.25->0.3 (concave)
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.4, yOffset + 0.75, 0.5); // x=0.25->0.4 (convex)
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.25, 1.0); // z=0.5->1.0 (convex)
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on other edges for the second tetra

  points->InsertNextPoint(xOffset + 0.125, yOffset + 0.15, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.125, yOffset + 1.15, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.375, yOffset + 0.65, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticTetra = vtkSmartPointer<vtkQuadraticTetra>::New();
  quadraticTetra->GetPointIds()->SetId(0, pointId);
  quadraticTetra->GetPointIds()->SetId(1, pointId + 1);
  quadraticTetra->GetPointIds()->SetId(2, pointId + 2);
  quadraticTetra->GetPointIds()->SetId(3, pointId + 3);
  quadraticTetra->GetPointIds()->SetId(4, pointId + 5);
  quadraticTetra->GetPointIds()->SetId(5, pointId + 6);
  quadraticTetra->GetPointIds()->SetId(6, pointId + 7);
  quadraticTetra->GetPointIds()->SetId(7, pointId + 8);
  quadraticTetra->GetPointIds()->SetId(8, pointId + 9);
  quadraticTetra->GetPointIds()->SetId(9, pointId + 10);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticTetra->GetCellType(), quadraticTetra->GetPointIds());

  quadraticTetra = vtkSmartPointer<vtkQuadraticTetra>::New();
  quadraticTetra->GetPointIds()->SetId(0, pointId);
  quadraticTetra->GetPointIds()->SetId(1, pointId + 2);
  quadraticTetra->GetPointIds()->SetId(2, pointId + 1);
  quadraticTetra->GetPointIds()->SetId(3, pointId + 4);
  quadraticTetra->GetPointIds()->SetId(4, pointId + 7);
  quadraticTetra->GetPointIds()->SetId(5, pointId + 6);
  quadraticTetra->GetPointIds()->SetId(6, pointId + 5);
  quadraticTetra->GetPointIds()->SetId(7, pointId + 11);
  quadraticTetra->GetPointIds()->SetId(8, pointId + 12);
  quadraticTetra->GetPointIds()->SetId(9, pointId + 13);

  pointId += 14;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticTetra->GetCellType(), quadraticTetra->GetPointIds());

  // 3D: quadratic hexahedron: 2 with a common face
  xOffset += 2.0;

  // a face (back): 0-3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // a face (common): 4-7
  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // a face (front): 8-11
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the back face: 12-15
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the common face: 16-19
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points between the back and common face: 20-23
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the front face: 24-27
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points between the common face and the front face: 28-31
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticHexahedron = vtkSmartPointer<vtkQuadraticHexahedron>::New();
  quadraticHexahedron->GetPointIds()->SetId(0, pointId);
  quadraticHexahedron->GetPointIds()->SetId(1, pointId + 1);
  quadraticHexahedron->GetPointIds()->SetId(2, pointId + 2);
  quadraticHexahedron->GetPointIds()->SetId(3, pointId + 3);

  quadraticHexahedron->GetPointIds()->SetId(4, pointId + 4);
  quadraticHexahedron->GetPointIds()->SetId(5, pointId + 5);
  quadraticHexahedron->GetPointIds()->SetId(6, pointId + 6);
  quadraticHexahedron->GetPointIds()->SetId(7, pointId + 7);

  quadraticHexahedron->GetPointIds()->SetId(8, pointId + 12);
  quadraticHexahedron->GetPointIds()->SetId(9, pointId + 13);
  quadraticHexahedron->GetPointIds()->SetId(10, pointId + 14);
  quadraticHexahedron->GetPointIds()->SetId(11, pointId + 15);

  quadraticHexahedron->GetPointIds()->SetId(12, pointId + 16);
  quadraticHexahedron->GetPointIds()->SetId(13, pointId + 17);
  quadraticHexahedron->GetPointIds()->SetId(14, pointId + 18);
  quadraticHexahedron->GetPointIds()->SetId(15, pointId + 19);

  quadraticHexahedron->GetPointIds()->SetId(16, pointId + 20);
  quadraticHexahedron->GetPointIds()->SetId(17, pointId + 21);
  quadraticHexahedron->GetPointIds()->SetId(18, pointId + 22);
  quadraticHexahedron->GetPointIds()->SetId(19, pointId + 23);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticHexahedron->GetCellType(), quadraticHexahedron->GetPointIds());

  quadraticHexahedron = vtkSmartPointer<vtkQuadraticHexahedron>::New();
  quadraticHexahedron->GetPointIds()->SetId(0, pointId + 4);
  quadraticHexahedron->GetPointIds()->SetId(1, pointId + 5);
  quadraticHexahedron->GetPointIds()->SetId(2, pointId + 6);
  quadraticHexahedron->GetPointIds()->SetId(3, pointId + 7);

  quadraticHexahedron->GetPointIds()->SetId(4, pointId + 8);
  quadraticHexahedron->GetPointIds()->SetId(5, pointId + 9);
  quadraticHexahedron->GetPointIds()->SetId(6, pointId + 10);
  quadraticHexahedron->GetPointIds()->SetId(7, pointId + 11);

  quadraticHexahedron->GetPointIds()->SetId(8, pointId + 16);
  quadraticHexahedron->GetPointIds()->SetId(9, pointId + 17);
  quadraticHexahedron->GetPointIds()->SetId(10, pointId + 18);
  quadraticHexahedron->GetPointIds()->SetId(11, pointId + 19);

  quadraticHexahedron->GetPointIds()->SetId(12, pointId + 24);
  quadraticHexahedron->GetPointIds()->SetId(13, pointId + 25);
  quadraticHexahedron->GetPointIds()->SetId(14, pointId + 26);
  quadraticHexahedron->GetPointIds()->SetId(15, pointId + 27);

  quadraticHexahedron->GetPointIds()->SetId(16, pointId + 28);
  quadraticHexahedron->GetPointIds()->SetId(17, pointId + 29);
  quadraticHexahedron->GetPointIds()->SetId(18, pointId + 30);
  quadraticHexahedron->GetPointIds()->SetId(19, pointId + 31);

  pointId += 32;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticHexahedron->GetCellType(), quadraticHexahedron->GetPointIds());

  // 3D: quadratic wedge: 3 wedges, some share a quadratic quad face, some
  // share a quadratic triangle face
  xOffset += 2.0;

  // corner points
  // triangle face of the first wedge
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // common triangle face
  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 4
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 5
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // triangle face of the second wedge
  // 6
  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 8
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.9, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // remaining vertices of the third wedge
  // 9
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points

  // triangle face of the first wedge: id=11
  // 11
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 12
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 13
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // common triangle face: id=14
  // 14
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 15
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 16
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other edges of the first wedge: id=17
  // 17
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 19
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // triangle face of the second wedge: id=20
  // 20
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 21
  points->InsertNextPoint(xOffset + 0.7, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 22
  points->InsertNextPoint(xOffset + 0.3, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other edges of the second wedge: id=23
  // 23
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 24
  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 25
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.95, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // first triangle face of the third wedge: id=26
  // 1+9
  // 26
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 27
  // 2+9
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // second triangle face of the third wedge: id=28
  // 28
  // 4+10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 29
  // 5+10
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other edges of the third wedge: id=30
  // 30
  // 9+10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticWedge = vtkSmartPointer<vtkQuadraticWedge>::New();
  quadraticWedge->GetPointIds()->SetId(0, pointId);
  quadraticWedge->GetPointIds()->SetId(1, pointId + 1);
  quadraticWedge->GetPointIds()->SetId(2, pointId + 2);
  quadraticWedge->GetPointIds()->SetId(3, pointId + 3);
  quadraticWedge->GetPointIds()->SetId(4, pointId + 4);
  quadraticWedge->GetPointIds()->SetId(5, pointId + 5);

  quadraticWedge->GetPointIds()->SetId(6, pointId + 11);
  quadraticWedge->GetPointIds()->SetId(7, pointId + 12);
  quadraticWedge->GetPointIds()->SetId(8, pointId + 13);
  quadraticWedge->GetPointIds()->SetId(9, pointId + 14);
  quadraticWedge->GetPointIds()->SetId(10, pointId + 15);
  quadraticWedge->GetPointIds()->SetId(11, pointId + 16);

  quadraticWedge->GetPointIds()->SetId(12, pointId + 17);
  quadraticWedge->GetPointIds()->SetId(13, pointId + 18);
  quadraticWedge->GetPointIds()->SetId(14, pointId + 19);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticWedge->GetCellType(), quadraticWedge->GetPointIds());

  // this wedge shares a triangle face
  quadraticWedge = vtkSmartPointer<vtkQuadraticWedge>::New();
  quadraticWedge->GetPointIds()->SetId(0, pointId + 3);
  quadraticWedge->GetPointIds()->SetId(1, pointId + 4);
  quadraticWedge->GetPointIds()->SetId(2, pointId + 5);
  quadraticWedge->GetPointIds()->SetId(3, pointId + 6);
  quadraticWedge->GetPointIds()->SetId(4, pointId + 7);
  quadraticWedge->GetPointIds()->SetId(5, pointId + 8);

  quadraticWedge->GetPointIds()->SetId(6, pointId + 14);
  quadraticWedge->GetPointIds()->SetId(7, pointId + 15);
  quadraticWedge->GetPointIds()->SetId(8, pointId + 16);
  quadraticWedge->GetPointIds()->SetId(9, pointId + 20);
  quadraticWedge->GetPointIds()->SetId(10, pointId + 21);
  quadraticWedge->GetPointIds()->SetId(11, pointId + 22);

  quadraticWedge->GetPointIds()->SetId(12, pointId + 23);
  quadraticWedge->GetPointIds()->SetId(13, pointId + 24);
  quadraticWedge->GetPointIds()->SetId(14, pointId + 25);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticWedge->GetCellType(), quadraticWedge->GetPointIds());

  // this wedge shares a quad face
  quadraticWedge = vtkSmartPointer<vtkQuadraticWedge>::New();
  quadraticWedge->GetPointIds()->SetId(0, pointId + 2);
  quadraticWedge->GetPointIds()->SetId(1, pointId + 1);
  quadraticWedge->GetPointIds()->SetId(2, pointId + 9);
  quadraticWedge->GetPointIds()->SetId(3, pointId + 5);
  quadraticWedge->GetPointIds()->SetId(4, pointId + 4);
  quadraticWedge->GetPointIds()->SetId(5, pointId + 10);

  quadraticWedge->GetPointIds()->SetId(6, pointId + 12);
  quadraticWedge->GetPointIds()->SetId(7, pointId + 26);
  quadraticWedge->GetPointIds()->SetId(8, pointId + 27);
  quadraticWedge->GetPointIds()->SetId(9, pointId + 15);
  quadraticWedge->GetPointIds()->SetId(10, pointId + 28);
  quadraticWedge->GetPointIds()->SetId(11, pointId + 29);

  quadraticWedge->GetPointIds()->SetId(12, pointId + 19);
  quadraticWedge->GetPointIds()->SetId(13, pointId + 18);
  quadraticWedge->GetPointIds()->SetId(14, pointId + 30);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticWedge->GetCellType(), quadraticWedge->GetPointIds());

  pointId += 31;

  // 3D: quadratic pyramid: 3 pyramids, some share the base quad face,
  // some share a triangle face

  xOffset += 2.0;

  // quad face
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 2
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -1.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // top vertex
  // 4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // bottom vertex
  // 5
  points->InsertNextPoint(xOffset + 0.5, yOffset - 1.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other quad base
  // 6
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.9);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-edge-points

  // 8=(0+1)/2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 9=(1+2)/2
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10=(2+3)/2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, -1.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 11=(0+3)/2
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 12=(0+4)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, -0.15);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 13=(1+4)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, -0.25);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 14=(2+4)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, -0.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 15=(3+4)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, -0.85);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 16=(3+5)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset - 0.5, -0.85);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 17=(2+5)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset - 0.5, -0.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18=(1+5)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset - 0.5, -0.25);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 19=(0+5)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset - 0.5, -0.15);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 20=(1+6)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, -0.05);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 21=(6+7)/2
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 22=(2+7)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, -0.95);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 23=(4+6)/2
  points->InsertNextPoint(xOffset + 1.25, yOffset + 0.5, -0.3);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 24=(4+7)/2
  points->InsertNextPoint(xOffset + 1.25, yOffset + 0.5, -0.7);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticPyramid = vtkSmartPointer<vtkQuadraticPyramid>::New();
  quadraticPyramid->GetPointIds()->SetId(0, pointId);
  quadraticPyramid->GetPointIds()->SetId(1, pointId + 1);
  quadraticPyramid->GetPointIds()->SetId(2, pointId + 2);
  quadraticPyramid->GetPointIds()->SetId(3, pointId + 3);
  quadraticPyramid->GetPointIds()->SetId(4, pointId + 4);

  quadraticPyramid->GetPointIds()->SetId(5, pointId + 8);
  quadraticPyramid->GetPointIds()->SetId(6, pointId + 9);
  quadraticPyramid->GetPointIds()->SetId(7, pointId + 10);
  quadraticPyramid->GetPointIds()->SetId(8, pointId + 11);

  quadraticPyramid->GetPointIds()->SetId(9, pointId + 12);
  quadraticPyramid->GetPointIds()->SetId(10, pointId + 13);
  quadraticPyramid->GetPointIds()->SetId(11, pointId + 14);
  quadraticPyramid->GetPointIds()->SetId(12, pointId + 15);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticPyramid->GetCellType(), quadraticPyramid->GetPointIds());

  quadraticPyramid = vtkSmartPointer<vtkQuadraticPyramid>::New();
  quadraticPyramid->GetPointIds()->SetId(0, pointId + 3);
  quadraticPyramid->GetPointIds()->SetId(1, pointId + 2);
  quadraticPyramid->GetPointIds()->SetId(2, pointId + 1);
  quadraticPyramid->GetPointIds()->SetId(3, pointId);
  quadraticPyramid->GetPointIds()->SetId(4, pointId + 5);

  quadraticPyramid->GetPointIds()->SetId(5, pointId + 10);
  quadraticPyramid->GetPointIds()->SetId(6, pointId + 9);
  quadraticPyramid->GetPointIds()->SetId(7, pointId + 8);
  quadraticPyramid->GetPointIds()->SetId(8, pointId + 11);

  quadraticPyramid->GetPointIds()->SetId(9, pointId + 16);
  quadraticPyramid->GetPointIds()->SetId(10, pointId + 17);
  quadraticPyramid->GetPointIds()->SetId(11, pointId + 18);
  quadraticPyramid->GetPointIds()->SetId(12, pointId + 19);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticPyramid->GetCellType(), quadraticPyramid->GetPointIds());

  quadraticPyramid = vtkSmartPointer<vtkQuadraticPyramid>::New();
  quadraticPyramid->GetPointIds()->SetId(0, pointId + 1);
  quadraticPyramid->GetPointIds()->SetId(1, pointId + 6);
  quadraticPyramid->GetPointIds()->SetId(2, pointId + 7);
  quadraticPyramid->GetPointIds()->SetId(3, pointId + 2);
  quadraticPyramid->GetPointIds()->SetId(4, pointId + 4);

  quadraticPyramid->GetPointIds()->SetId(5, pointId + 20);
  quadraticPyramid->GetPointIds()->SetId(6, pointId + 21);
  quadraticPyramid->GetPointIds()->SetId(7, pointId + 22);
  quadraticPyramid->GetPointIds()->SetId(8, pointId + 9);

  quadraticPyramid->GetPointIds()->SetId(9, pointId + 13);
  quadraticPyramid->GetPointIds()->SetId(10, pointId + 23);
  quadraticPyramid->GetPointIds()->SetId(11, pointId + 24);
  quadraticPyramid->GetPointIds()->SetId(12, pointId + 14);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticPyramid->GetCellType(), quadraticPyramid->GetPointIds());

  pointId += 25;

  // 3D: triquadratic hexahedron: 2 with a common face
  xOffset += 2.0;

  // a face (back): 0-3
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 2
  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // a face (common): 4-7
  // 4
  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 5
  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 6
  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // a face (front): 8-11
  // 8
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 9
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 11
  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the back face: 12-15
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the common face: 16-19
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points between the back and common face: 20-23
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the front face: 24-27
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points between the common face and the front face: 28-31
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // face-centered points
  // 32=(0+1+4+5)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 33=(1+2+5+6)/4
  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 34=(2+3+6+7)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 35=(0+3+4+7)/4
  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 36=(0+1+2+3)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 37=(4+5+6+7)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other hexa
  // 38=(4+5+8+9)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 39=(5+6+9+10)/4
  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.0, 3.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 40=(6+7+10+11)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 41=(4+7+8+11)/4
  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.0, 3.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 42=(8+9+10+11)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto triQuadraticHexahedron = vtkSmartPointer<vtkTriQuadraticHexahedron>::New();
  triQuadraticHexahedron->GetPointIds()->SetId(0, pointId);
  triQuadraticHexahedron->GetPointIds()->SetId(1, pointId + 1);
  triQuadraticHexahedron->GetPointIds()->SetId(2, pointId + 2);
  triQuadraticHexahedron->GetPointIds()->SetId(3, pointId + 3);

  triQuadraticHexahedron->GetPointIds()->SetId(4, pointId + 4);
  triQuadraticHexahedron->GetPointIds()->SetId(5, pointId + 5);
  triQuadraticHexahedron->GetPointIds()->SetId(6, pointId + 6);
  triQuadraticHexahedron->GetPointIds()->SetId(7, pointId + 7);

  triQuadraticHexahedron->GetPointIds()->SetId(8, pointId + 12);
  triQuadraticHexahedron->GetPointIds()->SetId(9, pointId + 13);
  triQuadraticHexahedron->GetPointIds()->SetId(10, pointId + 14);
  triQuadraticHexahedron->GetPointIds()->SetId(11, pointId + 15);

  triQuadraticHexahedron->GetPointIds()->SetId(12, pointId + 16);
  triQuadraticHexahedron->GetPointIds()->SetId(13, pointId + 17);
  triQuadraticHexahedron->GetPointIds()->SetId(14, pointId + 18);
  triQuadraticHexahedron->GetPointIds()->SetId(15, pointId + 19);

  triQuadraticHexahedron->GetPointIds()->SetId(16, pointId + 20);
  triQuadraticHexahedron->GetPointIds()->SetId(17, pointId + 21);
  triQuadraticHexahedron->GetPointIds()->SetId(18, pointId + 22);
  triQuadraticHexahedron->GetPointIds()->SetId(19, pointId + 23);

  // before: 32,33,34,35,36,37
  triQuadraticHexahedron->GetPointIds()->SetId(20, pointId + 35);
  triQuadraticHexahedron->GetPointIds()->SetId(21, pointId + 33);
  triQuadraticHexahedron->GetPointIds()->SetId(22, pointId + 32);
  triQuadraticHexahedron->GetPointIds()->SetId(23, pointId + 34);
  triQuadraticHexahedron->GetPointIds()->SetId(24, pointId + 36);
  triQuadraticHexahedron->GetPointIds()->SetId(25, pointId + 37);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    triQuadraticHexahedron->GetCellType(), triQuadraticHexahedron->GetPointIds());

  triQuadraticHexahedron = vtkSmartPointer<vtkTriQuadraticHexahedron>::New();
  triQuadraticHexahedron->GetPointIds()->SetId(0, pointId + 4);
  triQuadraticHexahedron->GetPointIds()->SetId(1, pointId + 5);
  triQuadraticHexahedron->GetPointIds()->SetId(2, pointId + 6);
  triQuadraticHexahedron->GetPointIds()->SetId(3, pointId + 7);

  triQuadraticHexahedron->GetPointIds()->SetId(4, pointId + 8);
  triQuadraticHexahedron->GetPointIds()->SetId(5, pointId + 9);
  triQuadraticHexahedron->GetPointIds()->SetId(6, pointId + 10);
  triQuadraticHexahedron->GetPointIds()->SetId(7, pointId + 11);

  triQuadraticHexahedron->GetPointIds()->SetId(8, pointId + 16);
  triQuadraticHexahedron->GetPointIds()->SetId(9, pointId + 17);
  triQuadraticHexahedron->GetPointIds()->SetId(10, pointId + 18);
  triQuadraticHexahedron->GetPointIds()->SetId(11, pointId + 19);

  triQuadraticHexahedron->GetPointIds()->SetId(12, pointId + 24);
  triQuadraticHexahedron->GetPointIds()->SetId(13, pointId + 25);
  triQuadraticHexahedron->GetPointIds()->SetId(14, pointId + 26);
  triQuadraticHexahedron->GetPointIds()->SetId(15, pointId + 27);

  triQuadraticHexahedron->GetPointIds()->SetId(16, pointId + 28);
  triQuadraticHexahedron->GetPointIds()->SetId(17, pointId + 29);
  triQuadraticHexahedron->GetPointIds()->SetId(18, pointId + 30);
  triQuadraticHexahedron->GetPointIds()->SetId(19, pointId + 31);

  // before: 38,39,40,41,37,42
  triQuadraticHexahedron->GetPointIds()->SetId(20, pointId + 41);
  triQuadraticHexahedron->GetPointIds()->SetId(21, pointId + 39);
  triQuadraticHexahedron->GetPointIds()->SetId(22, pointId + 38);
  triQuadraticHexahedron->GetPointIds()->SetId(23, pointId + 40);
  triQuadraticHexahedron->GetPointIds()->SetId(24, pointId + 37);
  triQuadraticHexahedron->GetPointIds()->SetId(25, pointId + 42);

  pointId += 43;

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    triQuadraticHexahedron->GetCellType(), triQuadraticHexahedron->GetPointIds());

  // 3D: quadratic linear wedge: 3 wedges, some share a quadratic linear quad
  // face, some share a quadratic triangle face
  // NOTE: ordering is different from linear wedge or quadratic wedge
  // linear or quad: triangle 0-1-2 points outside, 3-4-5 points inside
  // here: 0-1-2 points inside, 3-4-5 points outside
  //
  xOffset += 2.0;

  // corner points
  // triangle face of the first wedge
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // common triangle face
  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 4
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 5
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // triangle face of the second wedge
  // 6
  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 8
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.9, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // remaining vertices of the third wedge
  // 9
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points

  // triangle face of the first wedge: id=11
  // 11
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 12
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 13
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // common triangle face: id=14
  // 14
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 15
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 16
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // triangle face of the second wedge: id=17
  // 17
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18
  points->InsertNextPoint(xOffset + 0.7, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 19
  points->InsertNextPoint(xOffset + 0.3, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // first triangle face of the third wedge: id=20
  // 1+9
  // 20
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 21
  // 2+9
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // second triangle face of the third wedge: id=22
  // 22
  // 4+10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 23
  // 5+10
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto quadraticLinearWedge = vtkSmartPointer<vtkQuadraticLinearWedge>::New();
  quadraticLinearWedge->GetPointIds()->SetId(0, pointId);
  quadraticLinearWedge->GetPointIds()->SetId(1, pointId + 1);
  quadraticLinearWedge->GetPointIds()->SetId(2, pointId + 2);
  quadraticLinearWedge->GetPointIds()->SetId(3, pointId + 3);
  quadraticLinearWedge->GetPointIds()->SetId(4, pointId + 4);
  quadraticLinearWedge->GetPointIds()->SetId(5, pointId + 5);

  quadraticLinearWedge->GetPointIds()->SetId(6, pointId + 11);
  quadraticLinearWedge->GetPointIds()->SetId(7, pointId + 12);
  quadraticLinearWedge->GetPointIds()->SetId(8, pointId + 13);
  quadraticLinearWedge->GetPointIds()->SetId(9, pointId + 14);
  quadraticLinearWedge->GetPointIds()->SetId(10, pointId + 15);
  quadraticLinearWedge->GetPointIds()->SetId(11, pointId + 16);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticLinearWedge->GetCellType(), quadraticLinearWedge->GetPointIds());

  // this wedge shares a triangle face
  quadraticLinearWedge = vtkSmartPointer<vtkQuadraticLinearWedge>::New();
  quadraticLinearWedge->GetPointIds()->SetId(0, pointId + 3);
  quadraticLinearWedge->GetPointIds()->SetId(1, pointId + 4);
  quadraticLinearWedge->GetPointIds()->SetId(2, pointId + 5);
  quadraticLinearWedge->GetPointIds()->SetId(3, pointId + 6);
  quadraticLinearWedge->GetPointIds()->SetId(4, pointId + 7);
  quadraticLinearWedge->GetPointIds()->SetId(5, pointId + 8);

  quadraticLinearWedge->GetPointIds()->SetId(6, pointId + 14);
  quadraticLinearWedge->GetPointIds()->SetId(7, pointId + 15);
  quadraticLinearWedge->GetPointIds()->SetId(8, pointId + 16);
  quadraticLinearWedge->GetPointIds()->SetId(9, pointId + 17);
  quadraticLinearWedge->GetPointIds()->SetId(10, pointId + 18);
  quadraticLinearWedge->GetPointIds()->SetId(11, pointId + 19);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticLinearWedge->GetCellType(), quadraticLinearWedge->GetPointIds());

  // this wedge shares a quad face
  quadraticLinearWedge = vtkSmartPointer<vtkQuadraticLinearWedge>::New();
  quadraticLinearWedge->GetPointIds()->SetId(0, pointId + 2);
  quadraticLinearWedge->GetPointIds()->SetId(1, pointId + 1);
  quadraticLinearWedge->GetPointIds()->SetId(2, pointId + 9);
  quadraticLinearWedge->GetPointIds()->SetId(3, pointId + 5);
  quadraticLinearWedge->GetPointIds()->SetId(4, pointId + 4);
  quadraticLinearWedge->GetPointIds()->SetId(5, pointId + 10);

  quadraticLinearWedge->GetPointIds()->SetId(6, pointId + 12);
  quadraticLinearWedge->GetPointIds()->SetId(7, pointId + 20);
  quadraticLinearWedge->GetPointIds()->SetId(8, pointId + 21);
  quadraticLinearWedge->GetPointIds()->SetId(9, pointId + 15);
  quadraticLinearWedge->GetPointIds()->SetId(10, pointId + 22);
  quadraticLinearWedge->GetPointIds()->SetId(11, pointId + 23);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(quadraticLinearWedge->GetCellType(), quadraticLinearWedge->GetPointIds());

  pointId += 24;

  // 3D: bi quadratic quadratic wedge: 3 wedges, some share a biquadratic
  // quad face, some share a quadratic triangle face
  xOffset += 2.0;

  // corner points
  // triangle face of the first wedge
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // common triangle face
  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 4
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 5
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // triangle face of the second wedge
  // 6
  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 8
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.9, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // remaining vertices of the third wedge
  // 9
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points

  // triangle face of the first wedge: id=11
  // 11
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 12
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 13
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // common triangle face: id=14
  // 14
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 15
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 16
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other edges of the first wedge: id=17
  // 17
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 19
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // triangle face of the second wedge: id=20
  // 20
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.1, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 21
  points->InsertNextPoint(xOffset + 0.7, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 22
  points->InsertNextPoint(xOffset + 0.3, yOffset + 0.5, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other edges of the second wedge: id=23
  // 23
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 24
  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 25
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.95, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // first triangle face of the third wedge: id=26
  // 1+9
  // 26
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 27
  // 2+9
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // second triangle face of the third wedge: id=28
  // 28
  // 4+10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 29
  // 5+10
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other edges of the third wedge: id=30
  // 30
  // 9+10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // face-centered points
  // 31=(0+1+3+4)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 32=(1+2+4+5)/4
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 33=(0+2+3+5)/4
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 34=(3+4+6+7)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.05, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 35=(4+5+7+8)/4
  points->InsertNextPoint(xOffset + 0.725, yOffset + 0.5, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 36=(3+5+6+8)/4
  points->InsertNextPoint(xOffset + 0.275, yOffset + 0.5, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 37=(1+4+9+10)/4
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.5, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 38=(2+5+9+10)/4
  points->InsertNextPoint(xOffset + 0.75, yOffset + 1.0, 0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto biQuadraticQuadraticWedge = vtkSmartPointer<vtkBiQuadraticQuadraticWedge>::New();
  biQuadraticQuadraticWedge->GetPointIds()->SetId(0, pointId);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(1, pointId + 1);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(2, pointId + 2);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(3, pointId + 3);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(4, pointId + 4);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(5, pointId + 5);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(6, pointId + 11);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(7, pointId + 12);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(8, pointId + 13);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(9, pointId + 14);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(10, pointId + 15);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(11, pointId + 16);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(12, pointId + 17);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(13, pointId + 18);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(14, pointId + 19);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(15, pointId + 31);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(16, pointId + 32);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(17, pointId + 33);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    biQuadraticQuadraticWedge->GetCellType(), biQuadraticQuadraticWedge->GetPointIds());

  // this wedge shares a triangle face
  biQuadraticQuadraticWedge = vtkSmartPointer<vtkBiQuadraticQuadraticWedge>::New();
  biQuadraticQuadraticWedge->GetPointIds()->SetId(0, pointId + 3);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(1, pointId + 4);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(2, pointId + 5);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(3, pointId + 6);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(4, pointId + 7);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(5, pointId + 8);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(6, pointId + 14);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(7, pointId + 15);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(8, pointId + 16);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(9, pointId + 20);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(10, pointId + 21);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(11, pointId + 22);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(12, pointId + 23);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(13, pointId + 24);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(14, pointId + 25);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(15, pointId + 34);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(16, pointId + 35);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(17, pointId + 36);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    biQuadraticQuadraticWedge->GetCellType(), biQuadraticQuadraticWedge->GetPointIds());

  // this wedge shares a quad face
  biQuadraticQuadraticWedge = vtkSmartPointer<vtkBiQuadraticQuadraticWedge>::New();
  biQuadraticQuadraticWedge->GetPointIds()->SetId(0, pointId + 2);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(1, pointId + 1);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(2, pointId + 9);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(3, pointId + 5);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(4, pointId + 4);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(5, pointId + 10);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(6, pointId + 12);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(7, pointId + 26);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(8, pointId + 27);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(9, pointId + 15);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(10, pointId + 28);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(11, pointId + 29);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(12, pointId + 19);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(13, pointId + 18);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(14, pointId + 30);

  biQuadraticQuadraticWedge->GetPointIds()->SetId(15, pointId + 32);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(16, pointId + 37);
  biQuadraticQuadraticWedge->GetPointIds()->SetId(17, pointId + 38);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    biQuadraticQuadraticWedge->GetCellType(), biQuadraticQuadraticWedge->GetPointIds());

  pointId += 39;

  // 3D: triquadratic pyramid: 3 pyramids, some share the base quad face,
  // some share a triangle face
  xOffset += 2.0;

  // quad face
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 2
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -1.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -1.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // top vertex
  // 4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // bottom vertex
  // 5
  points->InsertNextPoint(xOffset + 0.5, yOffset - 1.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other quad base
  // 6
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.9);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-edge-points

  // 8=(0+1)/2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 9=(1+2)/2
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10=(2+3)/2
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, -1.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 11=(0+3)/2
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 12=(0+4)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, -0.15);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 13=(1+4)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, -0.25);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 14=(2+4)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset + 0.5, -0.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 15=(3+4)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset + 0.5, -0.85);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 16=(3+5)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset - 0.5, -0.85);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 17=(2+5)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset - 0.5, -0.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 18=(1+5)/2
  points->InsertNextPoint(xOffset + 0.75, yOffset - 0.5, -0.25);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 19=(0+5)/2
  points->InsertNextPoint(xOffset + 0.25, yOffset - 0.5, -0.15);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 20=(1+6)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, -0.05);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 21=(6+7)/2
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 22=(2+7)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, -0.95);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 23=(4+6)/2
  points->InsertNextPoint(xOffset + 1.25, yOffset + 0.5, -0.3);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 24=(4+7)/2
  points->InsertNextPoint(xOffset + 1.25, yOffset + 0.5, -0.7);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-face-points

  // 25=(0+1+2+3)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 26=(0+1+4)/3
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0 / 3.0, -0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 27=(1+2+4)/3
  points->InsertNextPoint(xOffset + 5.0 / 6.0, yOffset + 1.0 / 3.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 28=(2+3+4)/3
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.0 / 3.0, -0.9);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 29=(2+3+4)/3
  points->InsertNextPoint(xOffset + 1.0 / 6.0, yOffset + 1.0 / 3.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 30=(3+2+5)/3
  points->InsertNextPoint(xOffset + 0.5, yOffset + -1.0 / 3.0, -0.9);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 31=(2+1+5)/3
  points->InsertNextPoint(xOffset + 5.0 / 6.0, yOffset + -1.0 / 3.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 32=(1+0+5)/3
  points->InsertNextPoint(xOffset + 0.5, yOffset + -1.0 / 3.0, -0.1);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 33=(0+3+5)/3
  points->InsertNextPoint(xOffset + 1.0 / 6.0, yOffset + -1.0 / 3.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 34=(1+6+7+2)/4
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 35=(1+6+4)/3
  points->InsertNextPoint(xOffset + 7.0 / 6.0, yOffset + 1.0 / 3.0, -0.2);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 36=(6+7+4)/3
  points->InsertNextPoint(xOffset + 1.5, yOffset + 1.0 / 3.0, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 37=(7+2+4)/3
  points->InsertNextPoint(xOffset + 7.0 / 6.0, yOffset + 1.0 / 3.0, -0.8);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // volumetric centroids

  // 38=(0+1+2+3+4)/5
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.2, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 39=(3+2+1+0+5)/5
  points->InsertNextPoint(xOffset + 0.5, yOffset - 0.2, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 40=(1+6+7+2+4)/5
  points->InsertNextPoint(xOffset + 1.3, yOffset + 0.2, -0.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto triQuadraticPyramid = vtkSmartPointer<vtkTriQuadraticPyramid>::New();
  triQuadraticPyramid->GetPointIds()->SetId(0, pointId);
  triQuadraticPyramid->GetPointIds()->SetId(1, pointId + 1);
  triQuadraticPyramid->GetPointIds()->SetId(2, pointId + 2);
  triQuadraticPyramid->GetPointIds()->SetId(3, pointId + 3);
  triQuadraticPyramid->GetPointIds()->SetId(4, pointId + 4);

  triQuadraticPyramid->GetPointIds()->SetId(5, pointId + 8);
  triQuadraticPyramid->GetPointIds()->SetId(6, pointId + 9);
  triQuadraticPyramid->GetPointIds()->SetId(7, pointId + 10);
  triQuadraticPyramid->GetPointIds()->SetId(8, pointId + 11);

  triQuadraticPyramid->GetPointIds()->SetId(9, pointId + 12);
  triQuadraticPyramid->GetPointIds()->SetId(10, pointId + 13);
  triQuadraticPyramid->GetPointIds()->SetId(11, pointId + 14);
  triQuadraticPyramid->GetPointIds()->SetId(12, pointId + 15);

  triQuadraticPyramid->GetPointIds()->SetId(13, pointId + 25);
  triQuadraticPyramid->GetPointIds()->SetId(14, pointId + 26);
  triQuadraticPyramid->GetPointIds()->SetId(15, pointId + 27);
  triQuadraticPyramid->GetPointIds()->SetId(16, pointId + 28);
  triQuadraticPyramid->GetPointIds()->SetId(17, pointId + 29);

  triQuadraticPyramid->GetPointIds()->SetId(18, pointId + 38);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(triQuadraticPyramid->GetCellType(), triQuadraticPyramid->GetPointIds());

  triQuadraticPyramid = vtkSmartPointer<vtkTriQuadraticPyramid>::New();
  triQuadraticPyramid->GetPointIds()->SetId(0, pointId + 3);
  triQuadraticPyramid->GetPointIds()->SetId(1, pointId + 2);
  triQuadraticPyramid->GetPointIds()->SetId(2, pointId + 1);
  triQuadraticPyramid->GetPointIds()->SetId(3, pointId);
  triQuadraticPyramid->GetPointIds()->SetId(4, pointId + 5);

  triQuadraticPyramid->GetPointIds()->SetId(5, pointId + 10);
  triQuadraticPyramid->GetPointIds()->SetId(6, pointId + 9);
  triQuadraticPyramid->GetPointIds()->SetId(7, pointId + 8);
  triQuadraticPyramid->GetPointIds()->SetId(8, pointId + 11);

  triQuadraticPyramid->GetPointIds()->SetId(9, pointId + 16);
  triQuadraticPyramid->GetPointIds()->SetId(10, pointId + 17);
  triQuadraticPyramid->GetPointIds()->SetId(11, pointId + 18);
  triQuadraticPyramid->GetPointIds()->SetId(12, pointId + 19);

  triQuadraticPyramid->GetPointIds()->SetId(13, pointId + 25);
  triQuadraticPyramid->GetPointIds()->SetId(14, pointId + 30);
  triQuadraticPyramid->GetPointIds()->SetId(15, pointId + 31);
  triQuadraticPyramid->GetPointIds()->SetId(16, pointId + 32);
  triQuadraticPyramid->GetPointIds()->SetId(17, pointId + 33);

  triQuadraticPyramid->GetPointIds()->SetId(18, pointId + 39);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(triQuadraticPyramid->GetCellType(), triQuadraticPyramid->GetPointIds());

  triQuadraticPyramid = vtkSmartPointer<vtkTriQuadraticPyramid>::New();
  triQuadraticPyramid->GetPointIds()->SetId(0, pointId + 1);
  triQuadraticPyramid->GetPointIds()->SetId(1, pointId + 6);
  triQuadraticPyramid->GetPointIds()->SetId(2, pointId + 7);
  triQuadraticPyramid->GetPointIds()->SetId(3, pointId + 2);
  triQuadraticPyramid->GetPointIds()->SetId(4, pointId + 4);

  triQuadraticPyramid->GetPointIds()->SetId(5, pointId + 20);
  triQuadraticPyramid->GetPointIds()->SetId(6, pointId + 21);
  triQuadraticPyramid->GetPointIds()->SetId(7, pointId + 22);
  triQuadraticPyramid->GetPointIds()->SetId(8, pointId + 9);

  triQuadraticPyramid->GetPointIds()->SetId(9, pointId + 13);
  triQuadraticPyramid->GetPointIds()->SetId(10, pointId + 23);
  triQuadraticPyramid->GetPointIds()->SetId(11, pointId + 24);
  triQuadraticPyramid->GetPointIds()->SetId(12, pointId + 14);

  triQuadraticPyramid->GetPointIds()->SetId(13, pointId + 34);
  triQuadraticPyramid->GetPointIds()->SetId(14, pointId + 35);
  triQuadraticPyramid->GetPointIds()->SetId(15, pointId + 36);
  triQuadraticPyramid->GetPointIds()->SetId(16, pointId + 37);
  triQuadraticPyramid->GetPointIds()->SetId(17, pointId + 27);

  triQuadraticPyramid->GetPointIds()->SetId(18, pointId + 40);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(triQuadraticPyramid->GetCellType(), triQuadraticPyramid->GetPointIds());

  pointId += 41;

  // 3D: biquadraticquadratic hexahedron: 3 with a some with a common face
  // with no center point, and other with a face with a center point.
  xOffset += 2.0;

  // a face (back): 0-3
  // 0
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 1
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 2
  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 3
  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // a face (common): 4-7
  // 4
  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 5
  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 6
  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 7
  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // a face (front): 8-11
  // 8
  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 9
  points->InsertNextPoint(xOffset + 1.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 10
  points->InsertNextPoint(xOffset + 1.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 11
  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the back face: 12-15
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the common face: 16-19
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points between the back and common face: 20-23
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points on the front face: 24-27
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 1.0, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.5, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 1.0, 4.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // mid-points between the common face and the front face: 28-31
  points->InsertNextPoint(xOffset + 0.05, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // face-centered points
  // 32=(0+1+4+5)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.05, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 33=(1+2+5+6)/4
  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 34=(2+3+6+7)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.95, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 35=(0+3+4+7)/4
  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // other hexa
  // 36=(4+5+8+9)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 0.05, 3.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 37=(5+6+9+10)/4
  points->InsertNextPoint(xOffset + 0.95, yOffset + 1.0, 3.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 38=(6+7+10+11)/4
  points->InsertNextPoint(xOffset + 0.5, yOffset + 1.95, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 39=(4+7+8+11)/4
  points->InsertNextPoint(xOffset + 0.05, yOffset + 1.0, 3.75);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // Third hexahedron
  // 40
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 41
  points->InsertNextPoint(xOffset + 2.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 42
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 43
  points->InsertNextPoint(xOffset + 2.0, yOffset + 2.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // face-edge: 44,45,46
  // 44=(1+40)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 45=(40+41)/2
  points->InsertNextPoint(xOffset + 2.0, yOffset + 1.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 46=(1+40)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // face-edge: 47,48,49
  // 47=(5+42)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 48=(42+43)/2
  points->InsertNextPoint(xOffset + 2.0, yOffset + 1.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 49=(6+43)/2
  points->InsertNextPoint(xOffset + 1.5, yOffset + 2.0, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // cross-face-edge: 50,51
  // 50=(42+40)/2
  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // 51=(41+43)/2
  points->InsertNextPoint(xOffset + 2.0, yOffset + 2.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  // center: 52,53,54
  // 52=(40+41+42+43)/4
  points->InsertNextPoint(xOffset + 2.0, yOffset + 1.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 53=(1+5+40+42)/4
  points->InsertNextPoint(xOffset + 1.5, yOffset + 0.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;
  // 54=(2+6+41+43)/4
  points->InsertNextPoint(xOffset + 1.5, yOffset + 2.0, 1.5);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto biQuadraticQuadraticHexahedron = vtkSmartPointer<vtkBiQuadraticQuadraticHexahedron>::New();
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(0, pointId);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(1, pointId + 1);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(2, pointId + 2);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(3, pointId + 3);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(4, pointId + 4);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(5, pointId + 5);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(6, pointId + 6);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(7, pointId + 7);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(8, pointId + 12);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(9, pointId + 13);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(10, pointId + 14);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(11, pointId + 15);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(12, pointId + 16);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(13, pointId + 17);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(14, pointId + 18);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(15, pointId + 19);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(16, pointId + 20);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(17, pointId + 21);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(18, pointId + 22);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(19, pointId + 23);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(20, pointId + 35);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(21, pointId + 33);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(22, pointId + 32);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(23, pointId + 34);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    biQuadraticQuadraticHexahedron->GetCellType(), biQuadraticQuadraticHexahedron->GetPointIds());

  biQuadraticQuadraticHexahedron = vtkSmartPointer<vtkBiQuadraticQuadraticHexahedron>::New();
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(0, pointId + 4);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(1, pointId + 5);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(2, pointId + 6);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(3, pointId + 7);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(4, pointId + 8);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(5, pointId + 9);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(6, pointId + 10);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(7, pointId + 11);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(8, pointId + 16);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(9, pointId + 17);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(10, pointId + 18);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(11, pointId + 19);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(12, pointId + 24);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(13, pointId + 25);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(14, pointId + 26);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(15, pointId + 27);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(16, pointId + 28);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(17, pointId + 29);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(18, pointId + 30);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(19, pointId + 31);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(20, pointId + 39);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(21, pointId + 37);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(22, pointId + 36);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(23, pointId + 38);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    biQuadraticQuadraticHexahedron->GetCellType(), biQuadraticQuadraticHexahedron->GetPointIds());

  biQuadraticQuadraticHexahedron = vtkSmartPointer<vtkBiQuadraticQuadraticHexahedron>::New();
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(0, pointId + 1);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(1, pointId + 40);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(2, pointId + 41);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(3, pointId + 2);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(4, pointId + 5);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(5, pointId + 42);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(6, pointId + 43);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(7, pointId + 6);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(8, pointId + 44);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(9, pointId + 45);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(10, pointId + 46);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(11, pointId + 13);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(12, pointId + 47);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(13, pointId + 48);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(14, pointId + 49);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(15, pointId + 17);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(16, pointId + 21);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(17, pointId + 50);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(18, pointId + 51);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(19, pointId + 22);

  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(20, pointId + 33);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(21, pointId + 52);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(22, pointId + 53);
  biQuadraticQuadraticHexahedron->GetPointIds()->SetId(23, pointId + 54);

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(
    biQuadraticQuadraticHexahedron->GetCellType(), biQuadraticQuadraticHexahedron->GetPointIds());

  pointId += 55;

  // 3D: polyhedron: 2 hexahedra with one common face
  xOffset += 3.0;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 0.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 0.1, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.9, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.1, yOffset + 1.9, 3.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 0.0, 4.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 2.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  points->InsertNextPoint(xOffset + 0.0, yOffset + 2.0, 5.0);
  scalars->InsertNextValue(scalar);
  scalar += scalarStep;

  auto polyhedron = vtkSmartPointer<vtkPolyhedron>::New();
  vtkIdType faces[6][4] = {
    { 0, 4, 7, 3 },
    { 1, 2, 6, 5 },
    { 0, 1, 5, 4 },
    { 3, 7, 6, 2 },
    { 0, 3, 2, 1 },
    { 4, 5, 6, 7 },
  };
  polyhedron->GetPointIds()->SetNumberOfIds(6 * 5 + 1);
  vtkIdType fptr = 0;
  polyhedron->GetPointIds()->SetId(fptr++, 6);
  for (int f = 0; f < 6; f++)
  {
    polyhedron->GetPointIds()->SetId(fptr++, 4);
    for (int p = 0; p < 4; p++)
    {
      polyhedron->GetPointIds()->SetId(fptr++, pointId + faces[f][p]);
    }
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polyhedron->GetCellType(), polyhedron->GetPointIds());

  polyhedron->GetPointIds()->SetNumberOfIds(6 * 5 + 1);
  fptr = 0;
  polyhedron->GetPointIds()->SetId(fptr++, 6);
  for (int f = 0; f < 6; f++)
  {
    polyhedron->GetPointIds()->SetId(fptr++, 4);
    for (int p = 0; p < 4; p++)
    {
      polyhedron->GetPointIds()->SetId(fptr++, 4 + pointId + faces[f][p]);
    }
  }

  cellIds->InsertNextValue(cellId);
  ++cellId;
  grid->InsertNextCell(polyhedron->GetCellType(), polyhedron->GetPointIds());

  pointId += 12;

  grid->SetPoints(points);
  grid->GetPointData()->SetScalars(scalars);
  grid->GetCellData()->SetScalars(cellIds);

  vtkNew<vtkCellTypes> uniqueCells;
  grid->GetCellTypes(uniqueCells);
  for (vtkIdType j = 0; j < uniqueCells->GetNumberOfTypes(); ++j)
  {
    unsigned char cellType = uniqueCells->GetCellType(j);
    std::cout << vtkCellTypes::GetClassNameFromTypeId(cellType) << ": "
              << static_cast<int>(cellType) << std::endl;
  }

#endif

  // Create the filter
  auto geom = vtkSmartPointer<vtkUnstructuredGridGeometryFilter>::New();
#ifdef READ_FILE
  geom->SetInputConnection(0, reader->GetOutputPort(0));
#else
  geom->SetInputData(grid);
#endif
  geom->Update(); // So that we can call GetRange() on the scalars

  // Check that all the cells are not 3D.
  // TODO
#ifdef USE_SHRINK
  auto shrink = vtkSmartPointer<vtkShrinkFilter>::New();
  shrink->SetShrinkFactor(0.5);
  shrink->SetInputConnection(0, geom->GetOutputPort(0));
#endif
#ifdef FAST_GEOMETRY
  auto linearGeom = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
#else
  auto linearGeom = vtkSmartPointer<vtkGeometryFilter>::New();
#endif
#ifdef USE_SHRINK
  linearGeom->SetInputConnection(0, shrink->GetOutputPort(0));
#else
  linearGeom->SetInputConnection(0, geom->GetOutputPort(0));
#endif
  linearGeom->Update(); // So that we can call GetRange() on the scalars

  // This creates a blue to red lut.
  auto lut = vtkSmartPointer<vtkLookupTable>::New();
  lut->SetHueRange(0.667, 0.0);

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetLookupTable(lut);
  mapper->SetInputConnection(0, linearGeom->GetOutputPort(0));

  if (linearGeom->GetOutput()->GetPointData() != nullptr)
  {
    if (linearGeom->GetOutput()->GetPointData()->GetScalars() != nullptr)
    {
      mapper->SetScalarRange(linearGeom->GetOutput()->GetPointData()->GetScalars()->GetRange());
    }
  }

  auto actor = vtkSmartPointer<vtkActor>::New();
//  cout<<"prop="<<actor->GetProperty()->GetBackfaceCulling()<<endl;
#ifdef USE_CULLING
  actor->GetProperty()->SetBackfaceCulling(1);
#endif
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

#ifdef WRITE_RESULT
  // Save the result of the filter in a file
  auto writer3d = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer3d->SetInputConnection(0, geom->GetOutputPort(0));
  writer3d->SetFileName("surface3d.vtu");
  writer3d->SetDataModeToAscii();
  writer3d->Write();
#endif // #ifdef WRITE_RESULT

  // Standard testing code.
  renderer->SetBackground(0.5, 0.5, 0.5);
  renWin->SetSize(300, 300);
  //  vtkCamera *cam=renderer->GetActiveCamera();
  //  renderer->ResetCamera();
  //  cam->Azimuth(180);

  renWin->Render();
  int retVal = vtkRegressionTestImage(renWin);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    iren->Start();
  }

  return !retVal;
}
