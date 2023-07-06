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
 * @brief   Accelerated point to cell interpolation filter.
 *
 * vtkmAverageToPoints is a filter that transforms point data (i.e., data
 * specified at cell points) into cell data (i.e., data specified per cell).
 * The method of transformation is based on averaging the data
 * values of all points used by particular cell. This filter will also
 * pass through any existing point and cell arrays.
 *
 */

#ifndef vtkmAverageToCells_h
#define vtkmAverageToCells_h

#include "vtkAcceleratorsVTKmFiltersModule.h" //required for correct implementation
#include "vtkDataSetAlgorithm.h"

class VTKACCELERATORSVTKMFILTERS_EXPORT vtkmAverageToCells : public vtkDataSetAlgorithm
{
public:
  vtkTypeMacro(vtkmAverageToCells, vtkDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  static vtkmAverageToCells* New();

protected:
  vtkmAverageToCells();
  ~vtkmAverageToCells() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkmAverageToCells(const vtkmAverageToCells&) = delete;
  void operator=(const vtkmAverageToCells&) = delete;
};

#endif // vtkmAverageToCells_h
