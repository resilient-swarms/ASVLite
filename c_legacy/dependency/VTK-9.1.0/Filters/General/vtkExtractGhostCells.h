/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkExtractGhostCells.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class vtkExtractGhostCells
 * @brief Extracts ghost cells from the input and untag them (they become visible).
 *
 * This filter takes a `vtkDataSet` as input, removes any non-ghost cell,
 * and renames the ghost cell array in the output to what `OutputGhostArrayName` is set to
 * so it is no longer treated as a ghost type array.
 * By default, `OutputGhostArrayName` is set to `GhostType`.
 */

#ifndef vtkExtractGhostCells_h
#define vtkExtractGhostCells_h

#include "vtkFiltersGeneralModule.h" // for export macros
#include "vtkUnstructuredGridAlgorithm.h"

class VTKFILTERSGENERAL_EXPORT vtkExtractGhostCells : public vtkUnstructuredGridAlgorithm
{
public:
  static vtkExtractGhostCells* New();
  vtkTypeMacro(vtkExtractGhostCells, vtkUnstructuredGridAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Set / Get the name of the ghost cell array in the output.
   */
  vtkSetStringMacro(OutputGhostArrayName);
  vtkGetStringMacro(OutputGhostArrayName);
  ///@}

protected:
  vtkExtractGhostCells();
  ~vtkExtractGhostCells() override;

  int FillInputPortInformation(int port, vtkInformation* info) override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  char* OutputGhostArrayName;

private:
  vtkExtractGhostCells(const vtkExtractGhostCells&) = delete;
  void operator=(const vtkExtractGhostCells&) = delete;
};

#endif
