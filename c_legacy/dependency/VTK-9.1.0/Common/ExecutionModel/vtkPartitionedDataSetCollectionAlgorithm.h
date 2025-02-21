/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPartitionedDataSetCollectionCollectionAlgorithm.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class vtkPartitionedDataSetCollectionAlgorithm
 * @brief Superclass for algorithms that produce vtkPartitionedDataSetCollectionAlgorithm
 *
 * vtkPartitionedDataSetCollectionAlgorithm is intended as a superclass for algorithms that
 * produce a vtkPartitionedDataSetCollection.
 */

#ifndef vtkPartitionedDataSetCollectionAlgorithm_h
#define vtkPartitionedDataSetCollectionAlgorithm_h

#include "vtkAlgorithm.h"
#include "vtkCommonExecutionModelModule.h" // For export macro

class vtkPartitionedDataSetCollection;

class VTKCOMMONEXECUTIONMODEL_EXPORT vtkPartitionedDataSetCollectionAlgorithm : public vtkAlgorithm
{
public:
  vtkTypeMacro(vtkPartitionedDataSetCollectionAlgorithm, vtkAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Get the output data object for the specified output port.
   */
  vtkPartitionedDataSetCollection* GetOutput();
  vtkPartitionedDataSetCollection* GetOutput(int);
  ///@}

  vtkTypeBool ProcessRequest(
    vtkInformation* request, vtkInformationVector** inInfo, vtkInformationVector* outInfo) override;

protected:
  vtkPartitionedDataSetCollectionAlgorithm();
  ~vtkPartitionedDataSetCollectionAlgorithm() override;

  ///@{
  /**
   * Methods for subclasses to override to handle different pipeline requests.
   */
  virtual int RequestDataObject(vtkInformation*, vtkInformationVector**, vtkInformationVector*)
  {
    return 1;
  }
  virtual int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*)
  {
    return 1;
  }
  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*)
  {
    return 1;
  }
  virtual int RequestUpdateExtent(vtkInformation*, vtkInformationVector**, vtkInformationVector*)
  {
    return 1;
  }
  ///@}

  int FillOutputPortInformation(int port, vtkInformation* info) override;
  int FillInputPortInformation(int port, vtkInformation* info) override;

private:
  vtkPartitionedDataSetCollectionAlgorithm(
    const vtkPartitionedDataSetCollectionAlgorithm&) = delete;
  void operator=(const vtkPartitionedDataSetCollectionAlgorithm&) = delete;
};

#endif
