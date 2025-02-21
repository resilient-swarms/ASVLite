/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPConvertToMultiBlockDataSet.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class vtkPConvertToMultiBlockDataSet
 * @brief parallel version of vtkConvertToMultiBlockDataSet
 *
 * vtkPConvertToMultiBlockDataSet is the MPI-aware version of
 * vtkConvertToMultiBlockDataSet.
 *
 * The extra work this filter does it to ensure that each `vtkPartitionedDataSet` instance
 * in the input when replaced by a `vtkMultiPieceDataSet in the output,
 * `vtkMultiPieceDataSet` has piece counts across ranks such the output
 * multiblock structure is identical on all ranks. `vtkPartitionedDataSet` /
 * `vtkPartitionedDataSetCollection` doesn't have this requirement and hence the
 * number of partitions in a `vtkPartitionedDataSet` in the input may not be
 * identical on all ranks. Hence, this extra check is needed.
 */

#ifndef vtkPConvertToMultiBlockDataSet_h
#define vtkPConvertToMultiBlockDataSet_h

#include "vtkConvertToMultiBlockDataSet.h"
#include "vtkFiltersParallelModule.h" // For export macro

class vtkMultiProcessController;
class VTKFILTERSPARALLEL_EXPORT vtkPConvertToMultiBlockDataSet
  : public vtkConvertToMultiBlockDataSet
{
public:
  static vtkPConvertToMultiBlockDataSet* New();
  vtkTypeMacro(vtkPConvertToMultiBlockDataSet, vtkConvertToMultiBlockDataSet);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Get/Set the controller to use. By default, initialized to
   * `vtkMultiProcessController::GetGlobalController` in the constructor.
   */
  void SetController(vtkMultiProcessController*);
  vtkGetObjectMacro(Controller, vtkMultiProcessController);
  ///@}

protected:
  vtkPConvertToMultiBlockDataSet();
  ~vtkPConvertToMultiBlockDataSet() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkPConvertToMultiBlockDataSet(const vtkPConvertToMultiBlockDataSet&) = delete;
  void operator=(const vtkPConvertToMultiBlockDataSet&) = delete;
  vtkMultiProcessController* Controller;
};

#endif
