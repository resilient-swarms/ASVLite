/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCompositeDataProbeFilter.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkCompositeDataProbeFilter
 * @brief   subclass of vtkProbeFilter which supports
 * composite datasets in the input.
 *
 * vtkCompositeDataProbeFilter supports probing into multi-group datasets.
 * It sequentially probes through each concrete dataset within the composite
 * probing at only those locations at which there were no hits when probing
 * earlier datasets. For Hierarchical datasets, this traversal through leaf
 * datasets is done in reverse order of levels i.e. highest level first.
 * To keep the ability of using locators with a composite input, we use a map that
 * maps a dataset belonging to the composite input to its FindCell strategy.
 *
 * When dealing with composite datasets, partial arrays are common i.e.
 * data-arrays that are not available in all of the blocks. By default, this
 * filter only passes those point and cell data-arrays that are available in all
 * the blocks i.e. partial array are removed.
 * When PassPartialArrays is turned on, this behavior is changed to take a
 * union of all arrays present thus partial arrays are passed as well. However,
 * for composite dataset input, this filter still produces a non-composite
 * output. For all those locations in a block of where a particular data array
 * is missing, this filter uses vtkMath::Nan() for double and float arrays,
 * while 0 for all other types of arrays i.e int, char etc.
 */

#ifndef vtkCompositeDataProbeFilter_h
#define vtkCompositeDataProbeFilter_h

#include "vtkFiltersCoreModule.h" // For export macro
#include "vtkProbeFilter.h"

#include <map> // For std::map

class vtkCompositeDataSet;
class vtkDataSet;
class vtkFindCellStrategy;
template <typename T>
class vtkSmartPointer;

class VTKFILTERSCORE_EXPORT vtkCompositeDataProbeFilter : public vtkProbeFilter
{
public:
  static vtkCompositeDataProbeFilter* New();
  vtkTypeMacro(vtkCompositeDataProbeFilter, vtkProbeFilter);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * When dealing with composite datasets, partial arrays are common i.e.
   * data-arrays that are not available in all of the blocks. By default, this
   * filter only passes those point and cell data-arrays that are available in
   * all the blocks i.e. partial array are removed.  When PassPartialArrays is
   * turned on, this behavior is changed to take a union of all arrays present
   * thus partial arrays are passed as well. However, for composite dataset
   * input, this filter still produces a non-composite output. For all those
   * locations in a block of where a particular data array is missing, this
   * filter uses vtkMath::Nan() for double and float arrays, while 0 for all
   * other types of arrays i.e int, char etc.
   */
  vtkSetMacro(PassPartialArrays, bool);
  vtkGetMacro(PassPartialArrays, bool);
  vtkBooleanMacro(PassPartialArrays, bool);
  ///@}

  /**
   * Set the structure mapping a dataset belonging to the composite input to
   * its FindCell strategy. If a leaf is not a key of the provided map then no
   * strategy will be used for this leaf.
   */
  void SetFindCellStrategyMap(
    const std::map<vtkDataSet*, vtkSmartPointer<vtkFindCellStrategy>>& map);

protected:
  vtkCompositeDataProbeFilter();
  ~vtkCompositeDataProbeFilter() override;

  /**
   * Change input information to accept composite datasets as the input which
   * is probed into.
   */
  int FillInputPortInformation(int port, vtkInformation* info) override;

  /**
   * Builds the field list using the composite dataset source.
   */
  int BuildFieldList(vtkCompositeDataSet* source);

  /**
   * Initializes output and various arrays which keep track for probing status.
   */
  void InitializeOutputArrays(vtkPointData* outPD, vtkIdType numPts) override;

  /**
   * Handle composite input.
   */
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  /**
   * Create a default executive.
   */
  vtkExecutive* CreateDefaultExecutive() override;

  bool PassPartialArrays;

private:
  vtkCompositeDataProbeFilter(const vtkCompositeDataProbeFilter&) = delete;
  void operator=(const vtkCompositeDataProbeFilter&) = delete;

  std::map<vtkDataSet*, vtkSmartPointer<vtkFindCellStrategy>> StrategyMap;
};

#endif
