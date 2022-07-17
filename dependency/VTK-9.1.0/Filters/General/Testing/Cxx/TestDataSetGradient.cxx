/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestDataSetGradient.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include <vtkDataSetGradient.h>
#include <vtkSmartPointer.h>

#include <vtkArrowSource.h>
#include <vtkGlyph3D.h>
#include <vtkMaskPoints.h>

#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkGenericCell.h>
#include <vtkPointData.h>

#include <vtkUnstructuredGrid.h>
#include <vtkUnstructuredGridReader.h>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include "vtkTestUtilities.h"

int TestDataSetGradient(int argc, char* argv[])
{
  char* fileName = vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/hexa.vtk");

  // Read the data
  vtkSmartPointer<vtkUnstructuredGridReader> reader =
    vtkSmartPointer<vtkUnstructuredGridReader>::New();
  reader->SetFileName(fileName);
  delete[] fileName;

  // This class computes the gradient for each cell
  vtkSmartPointer<vtkDataSetGradient> gradient = vtkSmartPointer<vtkDataSetGradient>::New();
  gradient->SetInputConnection(reader->GetOutputPort());
  gradient->SetInputArrayToProcess(0, 0, 0, 0, "scalars");
  gradient->Update();

  // Create a polydata
  //  Points at the parametric center of each cell
  //  PointData contains the gradient
  vtkDoubleArray* gradientAtCenters =
    vtkDoubleArray::SafeDownCast(gradient->GetOutput()->GetCellData()->GetArray("gradient"));

  vtkSmartPointer<vtkDoubleArray> gradients = vtkSmartPointer<vtkDoubleArray>::New();
  gradients->ShallowCopy(gradientAtCenters);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(gradient->GetOutput()->GetNumberOfCells());

  vtkSmartPointer<vtkGenericCell> aCell = vtkSmartPointer<vtkGenericCell>::New();
  for (vtkIdType cellId = 0; cellId < gradient->GetOutput()->GetNumberOfCells(); ++cellId)
  {
    gradient->GetOutput()->GetCell(cellId, aCell);
    reader->GetOutput()->GetCell(cellId, aCell);

    double pcenter[3], center[3];
    aCell->GetParametricCenter(pcenter);
    std::vector<double> cweights(aCell->GetNumberOfPoints());
    int pSubId = 0;
    aCell->EvaluateLocation(pSubId, pcenter, center, &(*cweights.begin()));
    points->SetPoint(cellId, center);
  }
  polyData->SetPoints(points);
  polyData->GetPointData()->SetVectors(gradientAtCenters);

  // Select a small percentage of the gradients
  // Use 10% of the points
  int onRatio =
    reader->GetOutput()->GetNumberOfPoints() / (reader->GetOutput()->GetNumberOfPoints() * .1);

  vtkSmartPointer<vtkMaskPoints> maskPoints = vtkSmartPointer<vtkMaskPoints>::New();
  maskPoints->SetInputData(polyData);
  maskPoints->RandomModeOff();
  maskPoints->SetOnRatio(onRatio);

  // Create the Glyphs for the gradient
  vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();

  double scaleFactor = .005;
  vtkSmartPointer<vtkGlyph3D> vectorGradientGlyph = vtkSmartPointer<vtkGlyph3D>::New();
  vectorGradientGlyph->SetSourceConnection(arrowSource->GetOutputPort());
  vectorGradientGlyph->SetInputConnection(maskPoints->GetOutputPort());
  vectorGradientGlyph->SetScaleModeToScaleByVector();
  vectorGradientGlyph->SetVectorModeToUseVector();
  vectorGradientGlyph->SetScaleFactor(scaleFactor);

  vtkSmartPointer<vtkPolyDataMapper> vectorGradientMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  vectorGradientMapper->SetInputConnection(vectorGradientGlyph->GetOutputPort());
  vectorGradientMapper->ScalarVisibilityOff();

  vtkSmartPointer<vtkActor> vectorGradientActor = vtkSmartPointer<vtkActor>::New();
  vectorGradientActor->SetMapper(vectorGradientMapper);
  vectorGradientActor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(.5, .5, .5);

  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene
  renderer->AddActor(vectorGradientActor);

  renderer->ResetCamera();
  renderer->GetActiveCamera()->Azimuth(120);
  renderer->GetActiveCamera()->Elevation(30);
  renderer->GetActiveCamera()->Dolly(1.0);
  renderer->ResetCameraClippingRange();

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
