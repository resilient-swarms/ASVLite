/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMultiBlockFromTimeSeriesFilter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// Hide VTK_DEPRECATED_IN_9_1_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkMultiBlockFromTimeSeriesFilter.h"

#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"

vtkStandardNewMacro(vtkMultiBlockFromTimeSeriesFilter);

vtkMultiBlockFromTimeSeriesFilter::vtkMultiBlockFromTimeSeriesFilter()
{
  this->UpdateTimeIndex = 0;
}

vtkMultiBlockFromTimeSeriesFilter::~vtkMultiBlockFromTimeSeriesFilter() = default;

int vtkMultiBlockFromTimeSeriesFilter::FillInputPortInformation(int, vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkDataObject");
  return 1;
}

int vtkMultiBlockFromTimeSeriesFilter::RequestInformation(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inInfo, vtkInformationVector* outInfoVec)
{
  this->UpdateTimeIndex = 0;
  vtkInformation* info = inInfo[0]->GetInformationObject(0);
  int len = info->Length(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  double* timeSteps = info->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  this->TimeSteps.resize(len);
  std::copy(timeSteps, timeSteps + len, this->TimeSteps.begin());
  this->TempDataset = vtkSmartPointer<vtkMultiBlockDataSet>::New();
  this->TempDataset->SetNumberOfBlocks(len);

  vtkInformation* outInfo = outInfoVec->GetInformationObject(0);
  outInfo->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  outInfo->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());

  return 1;
}

int vtkMultiBlockFromTimeSeriesFilter::RequestUpdateExtent(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inInfo, vtkInformationVector* vtkNotUsed(outInfo))
{
  if (this->TimeSteps.size() > static_cast<size_t>(this->UpdateTimeIndex))
  {
    vtkInformation* info = inInfo[0]->GetInformationObject(0);
    info->Set(
      vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(), this->TimeSteps[this->UpdateTimeIndex]);
  }
  return 1;
}

int vtkMultiBlockFromTimeSeriesFilter::RequestData(
  vtkInformation* request, vtkInformationVector** inInfo, vtkInformationVector* outInfo)
{
  vtkInformation* info = inInfo[0]->GetInformationObject(0);
  vtkDataObject* data = vtkDataObject::GetData(info);
  vtkSmartPointer<vtkDataObject> clone = vtkSmartPointer<vtkDataObject>::Take(data->NewInstance());
  clone->ShallowCopy(data);
  this->TempDataset->SetBlock(this->UpdateTimeIndex, clone);
  if (this->UpdateTimeIndex < static_cast<vtkTypeInt64>(this->TimeSteps.size()) - 1)
  {
    this->UpdateTimeIndex++;
    request->Set(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING(), 1);
  }
  else
  {
    vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::GetData(outInfo);
    output->ShallowCopy(this->TempDataset);
    for (unsigned i = 0; i < this->TempDataset->GetNumberOfBlocks(); ++i)
    {
      this->TempDataset->SetBlock(i, nullptr);
    }
    request->Remove(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING());
  }
  return 1;
}

void vtkMultiBlockFromTimeSeriesFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
