/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestVTKMClipWithImplicitFunction.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkmClip.h"

#include "vtkActor.h"
#include "vtkDataArray.h"
#include "vtkDataSetSurfaceFilter.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRTAnalyticSource.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphere.h"

int TestVTKMClipWithImplicitFunction(int argc, char* argv[])
{
  vtkNew<vtkRTAnalyticSource> wavelet;
  wavelet->SetWholeExtent(-8, 8, -8, 8, -8, 8);
  wavelet->SetCenter(0, 0, 0);

  vtkNew<vtkSphere> sphere;
  sphere->SetCenter(0, 0, 0);
  sphere->SetRadius(10);
  vtkNew<vtkmClip> clip;
  clip->ForceVTKmOn();
  clip->SetInputConnection(wavelet->GetOutputPort());
  clip->SetClipFunction(sphere);

  vtkNew<vtkDataSetSurfaceFilter> surface;
  surface->SetInputConnection(clip->GetOutputPort());

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(surface->GetOutputPort());
  mapper->SetScalarRange(37, 150);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(actor);
  renderer->ResetCamera();

  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);
  iren->Initialize();

  renWin->Render();
  int retVal = vtkRegressionTestImage(renWin);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    iren->Start();
  }

  return !retVal;
}
