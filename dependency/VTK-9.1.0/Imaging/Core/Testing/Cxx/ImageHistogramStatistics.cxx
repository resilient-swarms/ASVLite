/*=========================================================================

  Program:   Visualization Toolkit
  Module:    ImageHistogramStatistics.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// Test the vtkImageHistogramStatistics class
//
// The command line arguments are:
// -I        => run in interactive mode

#include "vtkFloatingPointExceptions.h"
#include "vtkImageAccumulate.h"
#include "vtkImageCast.h"
#include "vtkImageData.h"
#include "vtkImageHistogramStatistics.h"
#include "vtkMath.h"
#include "vtkNew.h"
#include "vtkPNGReader.h"

#include "vtkTestUtilities.h"

#include <cmath>

int ImageHistogramStatistics(int argc, char* argv[])
{
  vtkNew<vtkPNGReader> reader;

  char* fname = vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/fullhead15.png");

  reader->SetFileName(fname);
  delete[] fname;

  // Use float data to get the most code coverage
  vtkNew<vtkImageCast> imageCast;
  imageCast->SetOutputScalarTypeToFloat();
  imageCast->SetInputConnection(reader->GetOutputPort());

  double minValTest = 0;
  double maxValTest = 3714;
  double meanValTest = 635.8066572717137;
  double medianTest = 190.9279926756695;
  double stdevTest = 660.9126299774935;
  double tol = 1e-6;

  vtkNew<vtkImageHistogramStatistics> statistics;
  statistics->SetInputConnection(imageCast->GetOutputPort());
  statistics->GenerateHistogramImageOff();
  statistics->Update();

  double minVal = statistics->GetMinimum();
  double maxVal = statistics->GetMaximum();
  double meanVal = statistics->GetMean();
  double median = statistics->GetMedian();
  double stdev = statistics->GetStandardDeviation();

  // uncomment to test vtkImageAccumulate instead
  /*
    vtkNew<vtkImageAccumulate> accumulate;
    accumulate->SetInputConnection(reader->GetOutputPort());
    accumulate->Update();

    double minVal = accumulate->GetMin()[0];
    double maxVal = accumulate->GetMax()[0];
    double meanVal = accumulate->GetMean()[0];
    double median = medianTest;
    double stdev = accumulate->GetStandardDeviation()[0];
  */

  bool retVal = true;

  if (fabs((minVal - minValTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "minVal " << minVal << " should be " << minValTest << endl;
    retVal = false;
  }
  if (fabs((maxVal - maxValTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "maxVal " << maxVal << " should be " << maxValTest << endl;
    retVal = false;
  }
  if (fabs((meanVal - meanValTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "meanVal " << meanVal << " should be " << meanValTest << endl;
    retVal = false;
  }
  if (fabs((median - medianTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "median " << median << " should be " << medianTest << endl;
    retVal = false;
  }
  if (fabs((stdev - stdevTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "stdev " << stdev << " should be " << stdevTest << endl;
    retVal = false;
  }

  // Make sure histogram computation does not crash if image has NaN pixel

  // Clone the input image
  vtkNew<vtkImageData> imageDataWitNaN;
  imageDataWitNaN->DeepCopy(imageCast->GetOutput());
  double rangeOriginal[2];
  imageDataWitNaN->GetScalarRange(rangeOriginal);

  // Set pixel value at (1,1,0) position to NaN
  imageDataWitNaN->SetScalarComponentFromDouble(1, 1, 0, 0, vtkMath::Nan());

  // Verify that scalar range is still computed correctly

  // GetScalarRange() would crash due to invalid floating-point operation, therefore we
  // need to disable floating-point exceptions here.
  vtkFloatingPointExceptions::Disable();

  double rangeWithNaN[2];
  imageDataWitNaN->GetScalarRange(rangeWithNaN);

  // range original[0] is 0 but overall range is around 3600
  // so do not divide by range original[0] here
  if (fabs(rangeOriginal[0] - rangeWithNaN[0]) > tol)
  {
    cout.precision(16);
    cout << "rangeWithNaN[0] " << rangeWithNaN[0] << " should be " << rangeOriginal[0] << endl;
    retVal = false;
  }
  if (fabs((rangeOriginal[1] - rangeWithNaN[1]) / rangeOriginal[1]) > tol)
  {
    cout.precision(16);
    cout << "rangeWithNaN[1] " << rangeWithNaN[1] << " should be " << rangeOriginal[1] << endl;
    retVal = false;
  }

  // Verify that the filter does not crash
  statistics->SetInputData(imageDataWitNaN);
  statistics->Update();

  // Verify that the results are still the same (one pixel should not cause perceivable difference)
  // (exact same tests as above)
  if (fabs((minVal - minValTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "minVal " << minVal << " should be " << minValTest << endl;
    retVal = false;
  }
  if (fabs((maxVal - maxValTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "maxVal " << maxVal << " should be " << maxValTest << endl;
    retVal = false;
  }
  if (fabs((meanVal - meanValTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "meanVal " << meanVal << " should be " << meanValTest << endl;
    retVal = false;
  }
  if (fabs((median - medianTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "median " << median << " should be " << medianTest << endl;
    retVal = false;
  }
  if (fabs((stdev - stdevTest) / maxValTest) > tol)
  {
    cout.precision(16);
    cout << "stdev " << stdev << " should be " << stdevTest << endl;
    retVal = false;
  }

  return !retVal;
}
