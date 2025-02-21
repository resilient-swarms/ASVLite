/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPWindBladeReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkPWindBladeReader
 * @brief   class for reading WindBlade data files
 *
 * vtkPWindBladeReader is a source object that reads WindBlade files
 * which are block binary files with tags before and after each block
 * giving the number of bytes within the block.  The number of data
 * variables dumped varies.  There are 3 output ports with the first
 * being a structured grid with irregular spacing in the Z dimension.
 * The second is an unstructured grid only read on on process 0 and
 * used to represent the blade.  The third is also a structured grid
 * with irregular spacing on the Z dimension.  Only the first and
 * second output ports have time dependent data.
 * Parallel version of vtkWindBladeReader.h
 */

#ifndef vtkPWindBladeReader_h
#define vtkPWindBladeReader_h

#include "vtkIOMPIParallelModule.h" // For export macro
#include "vtkWindBladeReader.h"

class PWindBladeReaderInternal;

class VTKIOMPIPARALLEL_EXPORT vtkPWindBladeReader : public vtkWindBladeReader
{
public:
  static vtkPWindBladeReader* New();
  vtkTypeMacro(vtkPWindBladeReader, vtkWindBladeReader);

  void PrintSelf(ostream& os, vtkIndent indent) override;

protected:
  vtkPWindBladeReader();
  ~vtkPWindBladeReader() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  void CalculatePressure(int pressure, int prespre, int tempg, int density) override;
  void CalculateVorticity(int vort, int uvw, int density) override;
  void LoadVariableData(int var) override;
  bool ReadGlobalData() override;
  bool FindVariableOffsets() override;
  void CreateZTopography(float* zValues) override;
  void SetupBladeData() override;
  void LoadBladeData(int timeStep) override;

private:
  PWindBladeReaderInternal* PInternal;

  vtkPWindBladeReader(const vtkPWindBladeReader&) = delete;
  void operator=(const vtkPWindBladeReader&) = delete;
};

#endif
