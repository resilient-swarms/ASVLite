/*==================================================================

  Program:   Visualization Toolkit
  Module:    TestHyperTreeGridBinaryHyperbolicParaboloidMaterial.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

===================================================================*/
// .SECTION Thanks
// This test was written by Philippe Pebay, Kitware 2012
// This test was revised by Philippe Pebay,2016
// This work was supported by Commissariat a l'Energie Atomique (CEA/DIF)

#include "vtkCamera.h"
#include "vtkCellData.h"
#include "vtkColorTransferFunction.h"
#include "vtkHyperTreeGrid.h"
#include "vtkHyperTreeGridGeometry.h"
#include "vtkHyperTreeGridSource.h"
#include "vtkNew.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkProperty2D.h"
#include "vtkQuadric.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkScalarBarActor.h"
#include "vtkTextProperty.h"

int TestHyperTreeGridBinaryHyperbolicParaboloidMaterial(int argc, char* argv[])
{
  // Hyper tree grid
  vtkNew<vtkHyperTreeGridSource> htGrid;
  vtkHyperTreeGrid* htg = vtkHyperTreeGrid::SafeDownCast(htGrid->GetOutput());
  htg->GetCellData()->SetScalars(htg->GetCellData()->GetArray("Depth"));
  htGrid->SetMaxDepth(6);
  htGrid->SetDimensions(9, 9, 9); // GridCell 8, 8, 8
  htGrid->SetGridScale(1., .5, .75);
  htGrid->SetBranchFactor(2);
  htGrid->UseDescriptorOff();
  htGrid->UseMaskOn();
  vtkNew<vtkQuadric> quadric;
  quadric->SetCoefficients(4., -16., 0., 0., 0., 0., -32., 64., 16., -48.);
  htGrid->SetQuadric(quadric);

  // Geometry
  vtkNew<vtkHyperTreeGridGeometry> geometry;
  geometry->SetInputConnection(htGrid->GetOutputPort());
  geometry->Update();
  vtkPolyData* pd = geometry->GetPolyDataOutput();
  pd->GetCellData()->SetActiveScalars("Quadric");

  //  Color transfer function
  vtkNew<vtkColorTransferFunction> colorFunction;
  colorFunction->AddRGBSegment(-90., 0., .4, 1., 0., 1., .4, 0.);

  // Mappers
  vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();
  vtkNew<vtkPolyDataMapper> mapper1;
  mapper1->SetInputConnection(geometry->GetOutputPort());
  mapper1->SetScalarRange(pd->GetCellData()->GetArray("Depth")->GetRange());
  mapper1->UseLookupTableScalarRangeOn();
  mapper1->SetLookupTable(colorFunction);

  // Actors
  vtkNew<vtkActor> actor1;
  actor1->SetMapper(mapper1);

  // Camera
  double bd[6];
  pd->GetBounds(bd);
  vtkNew<vtkCamera> camera;
  camera->SetClippingRange(1., 100.);
  camera->SetViewUp(0., 0., 1.);
  camera->SetFocalPoint(pd->GetCenter());
  camera->SetPosition(2.3 * bd[1], -1.4 * bd[3], .6 * bd[5]);

  // Scalar bar
  vtkNew<vtkScalarBarActor> scalarBar;
  scalarBar->SetLookupTable(colorFunction);
  scalarBar->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
  scalarBar->GetPositionCoordinate()->SetValue(.05, .3);
  scalarBar->SetTitle("Quadric");
  scalarBar->SetNumberOfLabels(4);
  scalarBar->SetWidth(0.15);
  scalarBar->SetHeight(0.4);
  scalarBar->SetTextPad(4);
  scalarBar->SetMaximumWidthInPixels(60);
  scalarBar->SetMaximumHeightInPixels(200);
  scalarBar->SetTextPositionToPrecedeScalarBar();
  scalarBar->GetTitleTextProperty()->SetColor(.4, .4, .4);
  scalarBar->GetLabelTextProperty()->SetColor(.4, .4, .4);
  scalarBar->SetDrawFrame(1);
  scalarBar->GetFrameProperty()->SetColor(.4, .4, .4);
  scalarBar->SetDrawBackground(1);
  scalarBar->GetBackgroundProperty()->SetColor(1., 1., 1.);

  // Renderer
  vtkNew<vtkRenderer> renderer;
  renderer->SetActiveCamera(camera);
  renderer->SetBackground(1., 1., 1.);
  renderer->AddActor(actor1);
  renderer->AddActor(scalarBar);

  // Render window
  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(renderer);
  renWin->SetSize(400, 400);
  renWin->SetMultiSamples(0);

  // Interactor
  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  // Render and test
  renWin->Render();

  int retVal = vtkRegressionTestImage(renWin);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    iren->Start();
  }

  return !retVal;
}
