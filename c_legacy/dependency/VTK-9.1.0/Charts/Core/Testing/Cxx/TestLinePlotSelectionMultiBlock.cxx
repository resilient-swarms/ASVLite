/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestLinePlotSelectionMultiBlock.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkAnnotationLink.h"
#include "vtkChartXY.h"
#include "vtkContextMouseEvent.h"
#include "vtkContextScene.h"
#include "vtkContextView.h"
#include "vtkFloatArray.h"
#include "vtkNew.h"
#include "vtkPlot.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkTable.h"

//------------------------------------------------------------------------------
int TestLinePlotSelectionMultiBlock(int, char*[])
{
  // Set up a 2D scene, add an XY chart to it
  vtkNew<vtkContextView> view;
  view->GetRenderWindow()->SetSize(400, 300);
  vtkNew<vtkChartXY> chart;
  view->GetScene()->AddItem(chart);
  vtkNew<vtkAnnotationLink> link;
  chart->SetAnnotationLink(link);
  chart->SetActionToButton(vtkChart::SELECT_POLYGON, vtkContextMouseEvent::LEFT_BUTTON);
  chart->SetSelectionMethod(vtkChart::SELECTION_ROWS);

  // Create a table with some points in it...
  vtkNew<vtkTable> table;
  vtkNew<vtkFloatArray> arrX;
  arrX->SetName("X Axis");
  table->AddColumn(arrX);
  vtkNew<vtkFloatArray> arrC;
  arrC->SetName("Cosine");
  table->AddColumn(arrC);
  vtkNew<vtkFloatArray> arrS;
  arrS->SetName("Sine");
  table->AddColumn(arrS);
  vtkNew<vtkFloatArray> arrS2;
  arrS2->SetName("Sine2");
  table->AddColumn(arrS2);
  // Test charting with a few more points...
  int numPoints = 69;
  float inc = 7.5 / (numPoints - 1);
  table->SetNumberOfRows(numPoints);
  for (int i = 0; i < numPoints; ++i)
  {
    table->SetValue(i, 0, i * inc);
    table->SetValue(i, 1, cos(i * inc) + 0.0);
    table->SetValue(i, 2, sin(i * inc) + 0.0);
    table->SetValue(i, 3, sin(i * inc) + 0.5);
  }

  // Add multiple line plots, setting the colors etc
  vtkPlot* line = chart->AddPlot(vtkChart::LINE, 0);
  line->SetInputData(table, 0, 1);
  line->SetColor(0, 255, 0, 255);
  line->SetWidth(1.0);
  line = chart->AddPlot(vtkChart::LINE, 1);
  line->SetInputData(table, 0, 2);
  line->SetColor(255, 0, 0, 255);
  line->SetWidth(5.0);
  line = chart->AddPlot(vtkChart::LINE, 2);
  line->SetInputData(table, 0, 3);
  line->SetColor(0, 0, 255, 255);
  line->SetWidth(4.0);

  view->Update();
  view->Render();

  // Inject some mouse events to perform selection.
  chart->SetSelectionMode(vtkContextScene::SELECTION_ADDITION);
  vtkContextMouseEvent event;
  event.SetInteractor(view->GetInteractor());
  event.SetPos(vtkVector2f(80, 50));
  event.SetButton(vtkContextMouseEvent::RIGHT_BUTTON);
  chart->MouseButtonPressEvent(event);
  event.SetPos(vtkVector2f(200, 200));
  chart->MouseButtonReleaseEvent(event);

  // Polygon now.
  event.SetPos(vtkVector2f(260, 50));
  event.SetButton(vtkContextMouseEvent::LEFT_BUTTON);
  chart->MouseButtonPressEvent(event);
  event.SetPos(vtkVector2f(220, 250));
  chart->MouseMoveEvent(event);
  event.SetPos(vtkVector2f(350, 90));
  chart->MouseButtonReleaseEvent(event);

  // Finally render the scene and compare the image to a reference image
  view->GetRenderWindow()->SetMultiSamples(0);
  view->GetInteractor()->Initialize();
  view->GetInteractor()->Start();
  return EXIT_SUCCESS;
}
