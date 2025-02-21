/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkNew.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkTestUtilities.h"

#include "vtkRenderWindowInteractor.h"

#include "vtkOpenGLRenderWindow.h"

//------------------------------------------------------------------------------
int TestSphereVertex(int argc, char* argv[])
{
  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(0.0, 0.0, 0.0);
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetSize(300, 300);
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renderWindow);

  vtkNew<vtkSphereSource> sphere;
  sphere->SetThetaResolution(16);
  sphere->SetPhiResolution(16);
  sphere->SetEndTheta(270.0);

  {
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphere->GetOutputPort());
    vtkNew<vtkActor> actor;
    renderer->AddActor(actor);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetDiffuseColor(0.4, 1.0, 1.0);
    vtkNew<vtkProperty> backProp;
    backProp->SetDiffuseColor(0.4, 0.65, 0.8);
    actor->SetBackfaceProperty(backProp);

    actor->GetProperty()->EdgeVisibilityOn();
    actor->GetProperty()->SetEdgeColor(1.0, 1.0, 1.0);
    actor->GetProperty()->SetLineWidth(7.0);
    actor->GetProperty()->RenderLinesAsTubesOn();

    actor->GetProperty()->VertexVisibilityOn();
    actor->GetProperty()->SetVertexColor(1.0, 0.5, 1.0);
    actor->GetProperty()->SetPointSize(14.0);
    actor->GetProperty()->RenderPointsAsSpheresOn();
  }

  renderWindow->SetMultiSamples(0);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Elevation(-45);
  renderer->GetActiveCamera()->OrthogonalizeViewUp();
  renderer->GetActiveCamera()->Zoom(1.5);
  renderer->ResetCameraClippingRange();
  renderWindow->Render();

  int retVal = vtkRegressionTestImageThreshold(renderWindow, 0.5);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    iren->Start();
  }

  return !retVal;
}
