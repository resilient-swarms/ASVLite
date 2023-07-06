/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPartitionedDataSetCollectionSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class vtkPartitionedDataSetCollectionSource
 * @brief a source that produces a vtkPartitionedDataSetCollection.
 *
 * vtkPartitionedDataSetCollection generates a vtkPartitionedDataSetCollection
 * for testing purposes. It uses vtkParametricFunctionSource internally to
 * generate different types of surfaces for each partitioned dataset in the
 * collection. Each partitioned dataset is split among ranks in an even fashion.
 * Thus the number of partitions per rank for a partitioned dataset are always
 * different.
 */

#ifndef vtkPartitionedDataSetCollectionSource_h
#define vtkPartitionedDataSetCollectionSource_h

#include "vtkFiltersSourcesModule.h" // For export macro
#include "vtkPartitionedDataSetCollectionAlgorithm.h"

class VTKFILTERSSOURCES_EXPORT vtkPartitionedDataSetCollectionSource
  : public vtkPartitionedDataSetCollectionAlgorithm
{
public:
  static vtkPartitionedDataSetCollectionSource* New();
  vtkTypeMacro(vtkPartitionedDataSetCollectionSource, vtkPartitionedDataSetCollectionAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Get/Set the number of partitioned datasets in the collection.
   */
  vtkSetClampMacro(NumberOfShapes, int, 0, 12);
  vtkGetMacro(NumberOfShapes, int);
  ///@}

protected:
  vtkPartitionedDataSetCollectionSource();
  ~vtkPartitionedDataSetCollectionSource() override;

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  int RequestData(
    vtkInformation* info, vtkInformationVector** input, vtkInformationVector* output) override;

private:
  vtkPartitionedDataSetCollectionSource(const vtkPartitionedDataSetCollectionSource&) = delete;
  void operator=(const vtkPartitionedDataSetCollectionSource&) = delete;

  int NumberOfShapes;
};

#endif
