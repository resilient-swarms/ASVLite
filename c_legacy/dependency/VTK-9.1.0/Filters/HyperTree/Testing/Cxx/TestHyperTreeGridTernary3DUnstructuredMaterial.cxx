/*==================================================================

  Program:   Visualization Toolkit
  Module:    TestHyperTreeGridTernary3DUnstructuredMaterial.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

===================================================================*/
// .SECTION Thanks
// This test was written by Philippe Pebay, Kitware 2012
// This test was revised by Philippe Pebay, 2016
// This work was supported by Commissariat a l'Energie Atomique (CEA/DIF)

#include "vtkCamera.h"
#include "vtkCellData.h"
#include "vtkDataSetMapper.h"
#include "vtkHyperTreeGrid.h"
#include "vtkHyperTreeGridSource.h"
#include "vtkHyperTreeGridToUnstructuredGrid.h"
#include "vtkNew.h"
#include "vtkOutlineFilter.h"
#include "vtkProperty.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkShrinkFilter.h"
#include "vtkUnstructuredGrid.h"

int TestHyperTreeGridTernary3DUnstructuredMaterial(int argc, char* argv[])
{
  // Hyper tree grid
  vtkNew<vtkHyperTreeGridSource> htGrid;
  htGrid->SetMaxDepth(5);
  htGrid->SetDimensions(4, 4, 3); // GridCell 3, 3, 2
  htGrid->SetGridScale(1.5, 1., .7);
  htGrid->SetBranchFactor(3);
  htGrid->UseMaskOn();
  htGrid->SetDescriptor(
    "RRR .R. .RR ..R ..R .R.|R.......................... ........................... "
    "........................... .............R............. ....RR.RR........R......... "
    ".....RRRR.....R.RR......... ........................... ........................... "
    "...........................|........................... ........................... "
    "........................... ...RR.RR.......RR.......... ........................... "
    "RR......................... ........................... ........................... "
    "........................... ........................... ........................... "
    "........................... ........................... "
    "............RRR............|........................... ........................... "
    ".......RR.................. ........................... ........................... "
    "........................... ........................... ........................... "
    "........................... ........................... "
    "...........................|........................... ...........................");
  htGrid->SetMask(
    "111 011 011 111 011 110|111111111111111111111111111 111111111111111111111111111 "
    "000000000100110111111111111 111111111111111111111111111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111111111111111111 111111111111111111111111111 "
    "000110011100000100100010100|000001011011111111111111111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111001111111101111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111111111111111111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111111111111111111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111111111111111111 "
    "111111111111111111111111111|000000000111100100111100100 000000000111001001111001001 "
    "000000111100100111111111111 000000111001001111111111111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111111111111111111 111111111111111111111111111 "
    "111111111111111111111111111 111111111111111111111111111 "
    "110110110100111110111000000|111111111111111111111111111  11111111111111111111111111");
  htGrid->Update();
  vtkHyperTreeGrid* htg = vtkHyperTreeGrid::SafeDownCast(htGrid->GetOutput());
  htg->GetCellData()->SetScalars(htg->GetCellData()->GetArray("Depth"));

  // Outline
  vtkNew<vtkOutlineFilter> outline;
  outline->SetInputConnection(htGrid->GetOutputPort());

  // Hyper tree grid to unstructured grid filter
  vtkNew<vtkHyperTreeGridToUnstructuredGrid> htg2ug;
  htg2ug->SetInputConnection(htGrid->GetOutputPort());
  htg2ug->Update();
  vtkUnstructuredGrid* pd = htg2ug->GetUnstructuredGridOutput();

  // Shrink filter
  vtkNew<vtkShrinkFilter> shrink;
  shrink->SetInputConnection(htg2ug->GetOutputPort());
  shrink->SetShrinkFactor(.8);

  // Mappers
  vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();
  vtkNew<vtkDataSetMapper> mapper1;
  mapper1->SetInputConnection(shrink->GetOutputPort());
  mapper1->SetScalarRange(pd->GetCellData()->GetArray("Depth")->GetRange());
  vtkNew<vtkDataSetMapper> mapper2;
  mapper2->SetInputConnection(htg2ug->GetOutputPort());
  mapper2->ScalarVisibilityOff();

  // Actors
  vtkNew<vtkActor> actor1;
  actor1->SetMapper(mapper1);
  vtkNew<vtkActor> actor2;
  actor2->SetMapper(mapper2);
  actor2->GetProperty()->SetRepresentationToWireframe();
  actor2->GetProperty()->SetColor(.7, .7, .7);

  // Camera
  vtkHyperTreeGrid* ht = htGrid->GetHyperTreeGridOutput();
  double bd[6];
  ht->GetBounds(bd);
  vtkNew<vtkCamera> camera;
  camera->SetClippingRange(1., 100.);
  camera->SetFocalPoint(ht->GetCenter());
  camera->SetPosition(-.8 * bd[1], 2.1 * bd[3], -4.8 * bd[5]);

  // Renderer
  vtkNew<vtkRenderer> renderer;
  renderer->SetActiveCamera(camera);
  renderer->SetBackground(1., 1., 1.);
  renderer->AddActor(actor1);
  renderer->AddActor(actor2);

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

  int retVal = vtkRegressionTestImageThreshold(renWin, 120);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    iren->Start();
  }

  return !retVal;
}
