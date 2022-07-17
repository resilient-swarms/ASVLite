//=============================================================================
//
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//
//  Copyright 2012 Sandia Corporation.
//  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
//  the U.S. Government retains certain rights in this software.
//
//=============================================================================
/**
 * @class   vtkmAverageToPoints
 * @brief   Accelerated cell to point interpolation filter.
 *
 * vtkmAverageToPoints is a filter that transforms cell data (i.e., data
 * specified per cell) into point data (i.e., data specified at cell
 * points). The method of transformation is based on averaging the data
 * values of all cells using a particular point. This filter will also
 * pass through any existing point and cell arrays.
 *
 */
#ifndef vtkmAverageToPoints_h
#define vtkmAverageToPoints_h

#include "vtkAcceleratorsVTKmFiltersModule.h" //required for correct implementation
#include "vtkDataSetAlgorithm.h"

class VTKACCELERATORSVTKMFILTERS_EXPORT vtkmAverageToPoints : public vtkDataSetAlgorithm
{
public:
  vtkTypeMacro(vtkmAverageToPoints, vtkDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  static vtkmAverageToPoints* New();

protected:
  vtkmAverageToPoints();
  ~vtkmAverageToPoints() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkmAverageToPoints(const vtkmAverageToPoints&) = delete;
  void operator=(const vtkmAverageToPoints&) = delete;
};

#endif // vtkmAverageToPoints_h
