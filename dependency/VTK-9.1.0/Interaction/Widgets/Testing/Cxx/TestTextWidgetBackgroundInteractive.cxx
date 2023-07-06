/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestTextWidgetBackgroundInteractive.cxx

=========================================================================*/
//
// This example tests the vtkTextWidget.

// First include the required header files for the VTK classes we are using.
#include "vtkActor.h"
#include "vtkBorderRepresentation.h"
#include "vtkBorderWidget.h"
#include "vtkCommand.h"
#include "vtkInteractorEventRecorder.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkTextRepresentation.h"
#include "vtkTextWidget.h"

int TestTextWidgetBackgroundInteractive(int, char*[])
{
  // Create the RenderWindow, Renderer and both Actors
  //
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  // Create a test pipeline
  //
  vtkNew<vtkSphereSource> ss;
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(ss->GetOutputPort());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  // Create the widget and its representation
  // Default Border Widget
  vtkNew<vtkBorderRepresentation> rep;
  rep->ProportionalResizeOn();
  rep->SetShowBorderToOn();
  rep->SetPolygonColor(0.0, 1.0, 0.0);
  rep->SetPolygonOpacity(0.2);

  vtkNew<vtkBorderWidget> widget;
  widget->SetInteractor(interactor);
  widget->SetRepresentation(rep);
  widget->SelectableOff();

  // Top Left: Default Widget
  vtkNew<vtkTextRepresentation> tlRep;
  tlRep->ProportionalResizeOff();
  tlRep->SetShowBorderToOn();
  tlRep->SetPosition(0.05, 0.75);
  tlRep->SetPosition2(0.3, 0.2);
  tlRep->SetPolygonColor(1.0, 0.0, 0.0);
  tlRep->SetPolygonOpacity(0.5);
  tlRep->SetCornerRadiusStrength(0.5);

  vtkNew<vtkTextWidget> tlWidget;
  tlWidget->SetInteractor(interactor);
  tlWidget->SetRepresentation(tlRep);

  // Top Right: Always On
  vtkNew<vtkTextRepresentation> trRep;
  trRep->ProportionalResizeOff();
  trRep->SetShowBorderToOn();
  trRep->SetPosition(0.65, 0.75);
  trRep->SetPosition2(0.3, 0.2);
  trRep->SetPolygonColor(0.0, 1.0, 0.0);

  vtkNew<vtkTextWidget> trWidget;
  trWidget->SetInteractor(interactor);
  trWidget->SetRepresentation(trRep);

  // Bottom Right: Auto + Always Border
  vtkNew<vtkTextRepresentation> brRep;
  brRep->ProportionalResizeOff();
  brRep->SetShowBorderToActive();
  brRep->SetPosition(0.65, 0.05);
  brRep->SetPosition2(0.3, 0.2);
  brRep->SetPolygonColor(1.0, 0.0, 1.0);
  brRep->SetPolygonOpacity(0.3);
  brRep->EnforceNormalizedViewportBoundsOn();
  brRep->SetMinimumNormalizedViewportSize(0.3, 0.2);

  vtkNew<vtkTextWidget> brWidget;
  brWidget->SetInteractor(interactor);
  brWidget->SetRepresentation(brRep);
  brWidget->SelectableOff();

  // Add the actors to the renderer, set the background and size
  //
  renderer->AddActor(actor);
  renderer->SetBackground(0.1, 0.2, 0.4);
  renderWindow->SetSize(300, 300);

  // render the image
  //
  interactor->Initialize();
  renderWindow->Render();
  widget->On();
  tlWidget->On();
  trWidget->On();
  brWidget->On();

  interactor->Start();

  return EXIT_SUCCESS;
}
