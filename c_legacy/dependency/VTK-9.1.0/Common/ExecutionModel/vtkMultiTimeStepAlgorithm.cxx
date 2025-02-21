/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkMultiTimeStepAlgorithm.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// Hide VTK_DEPRECATED_IN_9_1_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkMultiTimeStepAlgorithm.h"

#include "vtkCommand.h"
#include "vtkCompositeDataPipeline.h"
#include "vtkDataSet.h"
#include "vtkInformation.h"
#include "vtkInformationDoubleVectorKey.h"
#include "vtkInformationKey.h"
#include "vtkInformationVector.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"

vtkStandardNewMacro(vtkMultiTimeStepAlgorithm);

vtkInformationKeyMacro(vtkMultiTimeStepAlgorithm, UPDATE_TIME_STEPS, DoubleVector);

//------------------------------------------------------------------------------
// Instantiate object so that cell data is not passed to output.
vtkMultiTimeStepAlgorithm::vtkMultiTimeStepAlgorithm()
{
  this->RequestUpdateIndex = 0;
  this->SetNumberOfInputPorts(1);
  this->CacheData = false;
  this->NumberOfCacheEntries = 1;
}

//------------------------------------------------------------------------------
bool vtkMultiTimeStepAlgorithm::IsInCache(double time, size_t& idx)
{
  std::vector<TimeCache>::iterator it = this->Cache.begin();
  for (idx = 0; it != this->Cache.end(); ++it, ++idx)
  {
    if (time == it->TimeValue)
    {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
vtkTypeBool vtkMultiTimeStepAlgorithm::ProcessRequest(
  vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // create the output
  if (request->Has(vtkDemandDrivenPipeline::REQUEST_DATA_OBJECT()))
  {
    return this->RequestDataObject(request, inputVector, outputVector);
  }

  // set update extent
  if (request->Has(vtkCompositeDataPipeline::REQUEST_UPDATE_EXTENT()))
  {
    int retVal(1);
    vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
    if (this->RequestUpdateIndex == 0)
    {
      retVal = this->RequestUpdateExtent(request, inputVector, outputVector);

      double* upTimes = inInfo->Get(UPDATE_TIME_STEPS());
      int numUpTimes = inInfo->Length(UPDATE_TIME_STEPS());
      this->UpdateTimeSteps.clear();
      for (int i = 0; i < numUpTimes; i++)
      {
        this->UpdateTimeSteps.push_back(upTimes[i]);
      }
      inInfo->Remove(UPDATE_TIME_STEPS());
    }

    size_t nTimeSteps = this->UpdateTimeSteps.size();
    if (nTimeSteps > 0)
    {
      bool inCache = true;
      for (size_t i = 0; i < nTimeSteps; i++)
      {
        size_t idx;
        if (!this->IsInCache(this->UpdateTimeSteps[i], idx))
        {
          inCache = false;
          break;
        }
      }
      if (!inCache)
      {
        inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(),
          this->UpdateTimeSteps[this->RequestUpdateIndex]);
      }
      else
      {
        // Ask for any time step. This should not update unless something else changed.
        inInfo->Remove(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
      }
    }
    return retVal;
  }

  // generate the data
  if (request->Has(vtkCompositeDataPipeline::REQUEST_DATA()))
  {
    int retVal = 1;
    vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
    auto inData = vtk::MakeSmartPointer(inInfo->Get(vtkDataObject::DATA_OBJECT()));

    if (this->UpdateTimeSteps.empty())
    {
      vtkErrorMacro("No temporal data has been requested. ");
      return 0;
    }

    size_t idx;
    if (!this->IsInCache(this->UpdateTimeSteps[this->RequestUpdateIndex], idx))
    {
      auto inDataCopy = vtk::TakeSmartPointer(inData->NewInstance());
      inDataCopy->ShallowCopy(inData);
      this->Cache.emplace_back(this->UpdateTimeSteps[this->RequestUpdateIndex], inDataCopy);
    }

    this->RequestUpdateIndex++;

    const size_t nTimeSteps = this->UpdateTimeSteps.size();
    if (this->RequestUpdateIndex == static_cast<int>(nTimeSteps)) // all the time steps are here
    {
      // try calling the newer / recommended API first.
      std::vector<vtkSmartPointer<vtkDataObject>> inputs(nTimeSteps);
      for (size_t cc = 0; cc < nTimeSteps; ++cc)
      {
        if (this->IsInCache(this->UpdateTimeSteps[cc], idx))
        {
          inputs[cc] = this->Cache[idx].Data;
        }
        else
        {
          // This should never happen
          vtkErrorMacro("exceptional condition reached! Please report.");
          return 0;
        }
      }

      retVal = this->Execute(request, inputs, outputVector);
      if (retVal == -1)
      {
        vtkWarningMacro("Using legacy `RequestData`. That will not work for all input data-types. "
                        "Please update code to override `Execute` instead.");
        vtkNew<vtkMultiBlockDataSet> mb;
        for (size_t i = 0; i < nTimeSteps; i++)
        {
          if (this->IsInCache(this->UpdateTimeSteps[i], idx))
          {
            mb->SetBlock(static_cast<unsigned int>(i), this->Cache[idx].Data);
          }
        }

        // change the input to the multiblock data and let child class to do the work
        // make sure to set the input back to what it was to not break anything upstream
        inInfo->Set(vtkDataObject::DATA_OBJECT(), mb);
        retVal = this->RequestData(request, inputVector, outputVector);
        inInfo->Set(vtkDataObject::DATA_OBJECT(), inData);
      }

      this->UpdateTimeSteps.clear();
      this->RequestUpdateIndex = 0;
      if (!this->CacheData)
      {
        // No caching, remove all
        this->Cache.clear();
      }
      else
      {
        // Caching, erase ones outside the cache
        // Note that this is a first in first out implementation
        size_t cacheSize = this->Cache.size();
        if (cacheSize > this->NumberOfCacheEntries)
        {
          size_t nToErase = cacheSize - this->NumberOfCacheEntries;
          this->Cache.erase(this->Cache.begin(), this->Cache.begin() + nToErase);
        }
      }
      request->Remove(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING());
    }
    else
    {
      request->Set(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING(), 1);
    }

    return retVal;
  }

  // execute information
  if (request->Has(vtkDemandDrivenPipeline::REQUEST_INFORMATION()))
  {
    // Upstream changed, clear the cache.
    this->Cache.clear();
    return this->RequestInformation(request, inputVector, outputVector);
  }

  return this->Superclass::ProcessRequest(request, inputVector, outputVector);
}

//------------------------------------------------------------------------------
void vtkMultiTimeStepAlgorithm::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
