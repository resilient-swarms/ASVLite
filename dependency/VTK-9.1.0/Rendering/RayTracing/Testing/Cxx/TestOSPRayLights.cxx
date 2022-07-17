/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestOSPRayLights.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// This test verifies that lighting works as expected with ospray.
// When advanced materials are exposed in ospray, it will also validate
// refractions and reflections
//
// The command line arguments are:
// -I        => run in interactive mode; unless this is used, the program will
//              not allow interaction and exit
//              In interactive mode it responds to the keys listed
//              vtkOSPRayTestInteractor.h

#include "vtkTestUtilities.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkLight.h"
#include "vtkOSPRayPass.h"
#include "vtkOSPRayRendererNode.h"
#include "vtkOpenGLRenderer.h"
#include "vtkPLYReader.h"
#include "vtkPlaneSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkProperty.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"

#include "vtkOSPRayTestInteractor.h"

int TestOSPRayLights(int argc, char* argv[])
{
  vtkSmartPointer<vtkRenderWindowInteractor> iren =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  iren->SetRenderWindow(renWin);
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AutomaticLightCreationOff();
  renWin->AddRenderer(renderer);

  const char* fileName = vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/bunny.ply");
  vtkSmartPointer<vtkPLYReader> polysource = vtkSmartPointer<vtkPLYReader>::New();
  polysource->SetFileName(fileName);

  // measure so we can place things sensibly
  polysource->Update();
  double bds[6];
  polysource->GetOutput()->GetBounds(bds);
  double x0 = bds[0] * 2;
  double x1 = bds[1] * 2;
  double y0 = bds[2];
  double y1 = bds[3] * 2;
  double z0 = bds[4];
  double z1 = bds[5] * 4;

  // TODO: ospray acts strangely without these such that Diff and Spec are not 0..255 instead of
  // 0..1
  vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputConnection(polysource->GetOutputPort());

  const double DIFF = 0.7;
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(normals->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetDiffuse(DIFF);
  actor->GetProperty()->SetSpecularColor(1, 1, 1);
  actor->GetProperty()->SetSpecular(0.9);
  actor->GetProperty()->SetSpecularPower(500);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkPlaneSource> backwall = vtkSmartPointer<vtkPlaneSource>::New();
  backwall->SetOrigin(x0, y0, z0);
  backwall->SetPoint1(x1, y0, z0);
  backwall->SetPoint2(x0, y1, z0);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(backwall->GetOutputPort());
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetDiffuse(DIFF);
  actor->GetProperty()->SetSpecular(0);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkPlaneSource> floor = vtkSmartPointer<vtkPlaneSource>::New();
  floor->SetOrigin(x0, y0, z0);
  floor->SetPoint1(x0, y0, z1);
  floor->SetPoint2(x1, y0, z0);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(floor->GetOutputPort());
  actor = vtkSmartPointer<vtkActor>::New();
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetDiffuse(DIFF);
  actor->GetProperty()->SetSpecular(0);
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkPlaneSource> left = vtkSmartPointer<vtkPlaneSource>::New();
  left->SetOrigin(x0, y0, z0);
  left->SetPoint1(x0, y1, z0);
  left->SetPoint2(x0, y0, z1);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(left->GetOutputPort());
  actor = vtkSmartPointer<vtkActor>::New();
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetDiffuse(DIFF);
  actor->GetProperty()->SetSpecular(0);
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkSphereSource> magnifier = vtkSmartPointer<vtkSphereSource>::New();
  magnifier->SetCenter(x0 + (x1 - x0) * 0.6, y0 + (y1 - y0) * 0.2, z0 + (z1 - z0) * 0.7);
  magnifier->SetRadius((x1 - x0) * 0.05);
  magnifier->SetPhiResolution(30);
  magnifier->SetThetaResolution(30);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(magnifier->GetOutputPort());
  actor = vtkSmartPointer<vtkActor>::New();
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetInterpolationToPBR();
  actor->GetProperty()->SetMetallic(0.0);
  actor->GetProperty()->SetRoughness(0.1);
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkSphereSource> discoball = vtkSmartPointer<vtkSphereSource>::New();
  discoball->SetCenter(x0 + (x1 - x0) * 0.5, y0 + (y1 - y0) * 0.85, z0 + (z1 - z0) * 0.5);
  discoball->SetRadius((x1 - x0) * 0.1);
  discoball->SetPhiResolution(30);
  discoball->SetThetaResolution(30);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(discoball->GetOutputPort());
  actor = vtkSmartPointer<vtkActor>::New();
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetMetallic(1.0);
  actor->GetProperty()->SetRoughness(0.1);
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

#define INTENS 0.2

  vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
  // blue light casting shadows from infinity toward bottom left back corner
  light->PositionalOff();
  light->SetPosition(x0 + (x1 - x0) * 1, y0 + (y1 - y0) * 1, z0 + (z1 + z0) * 1);
  light->SetFocalPoint(x0, y0, z0);
  light->SetLightTypeToSceneLight();
  light->SetColor(0.0, 0.0, 1.0);
  light->SetIntensity(INTENS);
  light->SwitchOn();
  renderer->AddLight(light);

  light = vtkSmartPointer<vtkLight>::New();
  // red light casting shadows from top to bottom
  light->PositionalOn();
  double t = 1.8; // adjust t to see effect of positional
  light->SetPosition(x0 + (x1 - x0) * 0.5, y0 + (y1 - y0) * t, z0 + (z1 - z0) * 0.5);
  light->SetFocalPoint(x0 + (x1 - x0) * 0.5, y0 + (y1 - y0) * 0, z0 + (z1 - z0) * 0.5);
  light->SetLightTypeToSceneLight();
  light->SetColor(1.0, 0.0, 0.0);
  light->SetIntensity(INTENS);
  light->SwitchOn();
  renderer->AddLight(light);

  light = vtkSmartPointer<vtkLight>::New();
  // green light following camera
  light->PositionalOn();
  light->SetLightTypeToHeadlight();
  light->SetColor(0.0, 1.0, 0.0);
  light->SetIntensity(INTENS);
  light->SwitchOn();
  renderer->AddLight(light);

  renderer->SetBackground(0.0, 0.0, 0.0);
  renWin->SetSize(400, 400);

  vtkSmartPointer<vtkOSPRayPass> ospray = vtkSmartPointer<vtkOSPRayPass>::New();
  renderer->SetPass(ospray);
  vtkOSPRayRendererNode::SetRendererType("OSPRay pathtracer", renderer);

  for (int i = 0; i < argc; ++i)
  {
    if (!strcmp(argv[i], "--OptiX"))
    {
      vtkOSPRayRendererNode::SetRendererType("optix pathtracer", renderer);
      break;
    }
  }

  // increase image quality from default (otherwise subsampling artifacts)
  renWin->Render();
  renderer->UseShadowsOn();
  vtkOSPRayRendererNode::SetMaxFrames(0, renderer);
  vtkOSPRayRendererNode::SetSamplesPerPixel(20, renderer);

  vtkSmartPointer<vtkOSPRayTestInteractor> style = vtkSmartPointer<vtkOSPRayTestInteractor>::New();
  style->SetPipelineControlPoints(renderer, ospray, nullptr);
  iren->SetInteractorStyle(style);
  style->SetCurrentRenderer(renderer);

  iren->Start();

  return 0;
}
