/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestThreshold.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkDataObject.h"
#include "vtkFloatArray.h"
#include "vtkImageData.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkRTAnalyticSource.h"
#include "vtkSmartPointer.h"
#include "vtkThreshold.h"
#include "vtkUnstructuredGrid.h"

int TestThreshold(int, char*[])
{
  //---------------------------------------------------
  // Test using different thresholding methods
  //---------------------------------------------------
  vtkNew<vtkRTAnalyticSource> source;
  vtkNew<vtkThreshold> filter;
  filter->SetInputConnection(source->GetOutputPort());

  double L = 100.0;
  double U = 200.0;
  filter->SetThresholdFunction(vtkThreshold::THRESHOLD_BETWEEN);
  filter->SetLowerThreshold(L);
  filter->SetUpperThreshold(U);
  filter->SetAllScalars(0);
  filter->Update();
  int n1 = filter->GetOutput()->GetNumberOfCells();

  filter->UseContinuousCellRangeOn();
  filter->Update();
  int n2 = filter->GetOutput()->GetNumberOfCells();

  // we are using a large query range,
  // whether to use continuous range or not should not matter
  if (n1 != n2)
  {
    return EXIT_FAILURE;
  }

  filter->UseContinuousCellRangeOff();
  filter->SetUpperThreshold(L);
  filter->Update();
  // since we are not using continuous cell range
  // no cell points should fall in the empty interval
  if (filter->GetOutput()->GetNumberOfCells() > 0)
  {
    return EXIT_FAILURE;
  }
  filter->UseContinuousCellRangeOn();
  filter->Update();
  if (filter->GetOutput()->GetNumberOfCells() == 0)
  {
    return EXIT_FAILURE;
  }

  // Get the total number of cells
  int totalCellCount = source->GetOutput()->GetNumberOfCells();
  int thresholdedCellCount = filter->GetOutput()->GetNumberOfCells();

  // Now invert the threshold and test the number of cells
  filter->InvertOn();
  filter->Update();
  int invertedCellCount = filter->GetOutput()->GetNumberOfCells();
  if (invertedCellCount + thresholdedCellCount != totalCellCount)
  {
    std::cerr << "Cell count and inverted cell count inconsistent" << std::endl;
    return EXIT_FAILURE;
  }

  // Revert attributes to default values
  filter->AllScalarsOn();
  filter->InvertOff();
  filter->UseContinuousCellRangeOff();

  // Check the number of cells after thresholding below
  filter->SetThresholdFunction(vtkThreshold::THRESHOLD_LOWER);
  filter->SetLowerThreshold(L);
  filter->Update();
  if (filter->GetOutput()->GetNumberOfCells() != 132)
  {
    std::cerr << "Unexpected cell count after thresholding below" << std::endl;
    return EXIT_FAILURE;
  }

  // Check the number of cells after thresholding above
  filter->SetThresholdFunction(vtkThreshold::THRESHOLD_UPPER);
  filter->SetUpperThreshold(U);
  filter->Update();
  if (filter->GetOutput()->GetNumberOfCells() != 780)
  {
    std::cerr << "Unexpected cell count after thresholding above" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
