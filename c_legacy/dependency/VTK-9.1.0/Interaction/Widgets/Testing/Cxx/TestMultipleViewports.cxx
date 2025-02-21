/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestBorderWidget.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
//
// This example tests laying out widgets in multiple viewports.

// First include the required header files for the VTK classes we are using.
#include "vtkActor.h"
#include "vtkBorderRepresentation.h"
#include "vtkBorderWidget.h"
#include "vtkCommand.h"
#include "vtkHandleWidget.h"
#include "vtkNew.h"
#include "vtkPlaneSource.h"
#include "vtkPointHandleRepresentation2D.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

int TestMultipleViewports(int, char*[])
{
  // Create the RenderWindow, Renderers
  //
  vtkNew<vtkRenderer> ren0;
  vtkNew<vtkRenderer> ren1;
  vtkNew<vtkRenderWindow> renWin;

  ren0->SetBackground(0, 0, 0);
  ren0->SetViewport(0, 0, 0.5, 1);
  ren1->SetBackground(0.1, 0.1, 0.1);
  ren1->SetViewport(0.5, 0, 1, 1);

  renWin->AddRenderer(ren0);
  renWin->AddRenderer(ren1);

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  vtkNew<vtkPlaneSource> plane;
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(plane->GetOutputPort());
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  ren1->AddActor(planeActor);

  iren->Initialize();
  renWin->SetSize(300, 150);
  renWin->Render();

  // Create widgets in different viewports. Note that SetCurrentRenderer()
  // should be set to prevent the automated detection of renderer which
  // screws things up with multiple renderers.
  vtkNew<vtkBorderWidget> borderWidget;
  borderWidget->SetInteractor(iren);
  borderWidget->SetCurrentRenderer(ren0);
  vtkNew<vtkBorderRepresentation> borderRep;
  borderRep->GetPositionCoordinate()->SetValue(0.1, 0.5);
  borderRep->GetPosition2Coordinate()->SetValue(0.4, 0.1);
  borderRep->SetShowBorderToOn();
  borderWidget->SetRepresentation(borderRep);
  borderWidget->On();

  vtkNew<vtkHandleWidget> handleWidget;
  handleWidget->SetCurrentRenderer(ren1);
  handleWidget->SetInteractor(iren);
  vtkNew<vtkPointHandleRepresentation2D> handleRep;
  handleRep->SetWorldPosition(plane->GetOrigin());
  handleWidget->SetRepresentation(handleRep);
  handleWidget->On();

  // Remove the observers so we can go interactive. Without this the "-I"
  // testing option fails.
  iren->Start();

  return EXIT_SUCCESS;
}
