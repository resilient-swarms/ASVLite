/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestEllipseArcSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include <vtkActor.h>
#include <vtkEllipseArcSource.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int TestEllipseArcSource(int vtkNotUsed(argc), char* vtkNotUsed(argv)[])
{
  vtkNew<vtkEllipseArcSource> source;
  source->SetCenter(0.0, 0.0, 0.0);
  source->SetRatio(0.25);
  source->SetNormal(0., 0., 1.);
  source->SetMajorRadiusVector(10, 0., 0.);
  source->SetStartAngle(20);
  source->SetSegmentAngle(250);
  source->SetResolution(80);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(source->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3);

  renderWindow->SetMultiSamples(0);
  renderWindow->Render();
  renderWindowInteractor->Start();
  return EXIT_SUCCESS;
}
