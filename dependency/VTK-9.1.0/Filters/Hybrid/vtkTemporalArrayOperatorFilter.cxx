/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTemporalArrayOperatorFilter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

    This software is distributed WITHOUT ANY WARRANTY; without even
    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
    PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkTemporalArrayOperatorFilter.h"

#include "vtkArrayDispatch.h"
#include "vtkCellData.h"
#include "vtkDataArray.h"
#include "vtkDataArrayRange.h"
#include "vtkDataObjectTreeIterator.h"
#include "vtkDataSet.h"
#include "vtkGraph.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkTable.h"

#include <algorithm>
#include <functional>

vtkStandardNewMacro(vtkTemporalArrayOperatorFilter);

//------------------------------------------------------------------------------
vtkTemporalArrayOperatorFilter::vtkTemporalArrayOperatorFilter()
{
  this->Operator = OperatorType::ADD;
  this->NumberTimeSteps = 0;
  this->FirstTimeStepIndex = 0;
  this->SecondTimeStepIndex = 0;
  this->OutputArrayNameSuffix = nullptr;

  // Set the default input data array that the algorithm will process (point scalars)
  this->SetInputArrayToProcess(
    0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);

  this->SetNumberOfOutputPorts(1);
}

//------------------------------------------------------------------------------
vtkTemporalArrayOperatorFilter::~vtkTemporalArrayOperatorFilter()
{
  this->SetOutputArrayNameSuffix(nullptr);
}

//------------------------------------------------------------------------------
void vtkTemporalArrayOperatorFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Operator: " << this->Operator << endl;
  os << indent << "First time step: " << this->FirstTimeStepIndex << endl;
  os << indent << "Second time step: " << this->SecondTimeStepIndex << endl;
  os << indent << "Output array name suffix: "
     << (this->OutputArrayNameSuffix ? this->OutputArrayNameSuffix : "") << endl;
  os << indent << "Field association: "
     << vtkDataObject::GetAssociationTypeAsString(this->GetInputArrayAssociation()) << endl;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::FillInputPortInformation(
  int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkDataObject");
  return 1;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::FillOutputPortInformation(
  int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkDataObject");
  return 1;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::RequestDataObject(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputInfoVector, vtkInformationVector* outputInfoVector)
{
  vtkDataObject* inputObj = vtkDataObject::GetData(inputInfoVector[0]);
  if (inputObj != nullptr)
  {
    vtkDataObject* outputObj = vtkDataObject::GetData(outputInfoVector);
    if (!outputObj || !outputObj->IsA(inputObj->GetClassName()))
    {
      vtkDataObject* newOutputObj = inputObj->NewInstance();
      vtkInformation* outputInfo = outputInfoVector->GetInformationObject(0);
      outputInfo->Set(vtkDataObject::DATA_OBJECT(), newOutputObj);
      newOutputObj->Delete();
    }
    return 1;
  }
  return 0;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::RequestInformation(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputInfoVector, vtkInformationVector* vtkNotUsed(outputInfoVector))
{
  // Get input and output information objects
  vtkInformation* inputInfo = inputInfoVector[0]->GetInformationObject(0);

  // Check for presence more than one time step
  if (inputInfo->Has(vtkStreamingDemandDrivenPipeline::TIME_STEPS()))
  {
    // Find time on input
    this->NumberTimeSteps = inputInfo->Length(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    if (this->NumberTimeSteps < 2)
    {
      vtkErrorMacro(<< "Not enough numbers of time steps: " << this->NumberTimeSteps);
      return 0;
    }
  }
  else
  {
    vtkErrorMacro(<< "No time steps in input data.");
    return 0;
  }

  return 1;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::RequestUpdateExtent(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputInfoVector, vtkInformationVector* outputInfoVector)
{
  if (this->FirstTimeStepIndex < 0 || this->SecondTimeStepIndex < 0 ||
    this->FirstTimeStepIndex >= this->NumberTimeSteps ||
    this->SecondTimeStepIndex >= this->NumberTimeSteps)
  {
    vtkErrorMacro(<< "Specified timesteps (" << this->FirstTimeStepIndex << " and "
                  << this->SecondTimeStepIndex
                  << "are outside the range of"
                     " available time steps ("
                  << this->NumberTimeSteps << ")");
    return 0;
  }

  if (this->FirstTimeStepIndex == this->SecondTimeStepIndex)
  {
    vtkWarningMacro(<< "First and second time steps are the same.");
  }

  vtkInformation* outputInfo = outputInfoVector->GetInformationObject(0);
  // Find the required input time steps and request them
  if (outputInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    vtkInformation* inputInfo = inputInfoVector[0]->GetInformationObject(0);
    // Get the available input times
    double* inputTime = inputInfo->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    if (inputTime)
    {
      // Request the two time steps upstream
      double inputUpdateTimes[2] = { inputTime[this->FirstTimeStepIndex],
        inputTime[this->SecondTimeStepIndex] };

      inputInfo->Set(vtkMultiTimeStepAlgorithm::UPDATE_TIME_STEPS(), inputUpdateTimes, 2);
    }
  }
  return 1;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::Execute(vtkInformation*,
  const std::vector<vtkSmartPointer<vtkDataObject>>& inputs, vtkInformationVector* outputVector)
{
  if (inputs.size() != 2)
  {
    vtkErrorMacro(<< "The number of time blocks is incorrect.");
    return 0;
  }

  auto& data0 = inputs[0];
  auto& data1 = inputs[1];
  if (!data0 || !data1)
  {
    vtkErrorMacro(<< "Unable to retrieve data objects.");
    return 0;
  }

  vtkSmartPointer<vtkDataObject> newOutData;
  newOutData.TakeReference(this->Process(data0, data1));

  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkDataObject* outData = vtkDataObject::GetData(outInfo);
  outData->ShallowCopy(newOutData);

  return newOutData != nullptr ? 1 : 0;
}

//------------------------------------------------------------------------------
int vtkTemporalArrayOperatorFilter::GetInputArrayAssociation()
{
  vtkInformation* inputArrayInfo =
    this->GetInformation()->Get(INPUT_ARRAYS_TO_PROCESS())->GetInformationObject(0);
  return inputArrayInfo->Get(vtkDataObject::FIELD_ASSOCIATION());
}

//------------------------------------------------------------------------------
vtkDataObject* vtkTemporalArrayOperatorFilter::Process(
  vtkDataObject* inputData0, vtkDataObject* inputData1)
{
  if (inputData0->IsA("vtkCompositeDataSet"))
  {
    // We suppose input data are same type and have same structure (they should!)
    vtkCompositeDataSet* compositeDataSet0 = vtkCompositeDataSet::SafeDownCast(inputData0);
    vtkCompositeDataSet* compositeDataSet1 = vtkCompositeDataSet::SafeDownCast(inputData1);

    vtkCompositeDataSet* outputCompositeDataSet = compositeDataSet0->NewInstance();
    outputCompositeDataSet->ShallowCopy(inputData0);

    vtkSmartPointer<vtkCompositeDataIterator> iter;
    iter.TakeReference(compositeDataSet0->NewIterator());
    for (iter->InitTraversal(); !iter->IsDoneWithTraversal(); iter->GoToNextItem())
    {
      vtkDataObject* dataObj0 = iter->GetCurrentDataObject();
      vtkDataObject* dataObj1 = compositeDataSet1->GetDataSet(iter);
      if (!dataObj0 || !dataObj1)
      {
        vtkWarningMacro("The composite datasets have different structure.");
        continue;
      }

      vtkSmartPointer<vtkDataObject> resultDataObj;
      resultDataObj.TakeReference(this->ProcessDataObject(dataObj0, dataObj1));
      if (!resultDataObj)
      {
        return nullptr;
      }
      outputCompositeDataSet->SetDataSet(iter, resultDataObj);
    }
    return outputCompositeDataSet;
  }

  return this->ProcessDataObject(inputData0, inputData1);
}

//------------------------------------------------------------------------------
vtkDataObject* vtkTemporalArrayOperatorFilter::ProcessDataObject(
  vtkDataObject* inputData0, vtkDataObject* inputData1)
{
  vtkDataArray* inputArray0 = this->GetInputArrayToProcess(0, inputData0);
  vtkDataArray* inputArray1 = this->GetInputArrayToProcess(0, inputData1);
  if (!inputArray0 || !inputArray1)
  {
    vtkErrorMacro(<< "Unable to retrieve data arrays to process.");
    return nullptr;
  }

  if (inputArray0->GetDataType() != inputArray1->GetDataType())
  {
    vtkErrorMacro(<< "Array type in each time step are different.");
    return nullptr;
  }

  if (strcmp(inputArray0->GetName(), inputArray1->GetName()) != 0)
  {
    vtkErrorMacro(<< "Array name in each time step are different.");
    return nullptr;
  }

  if (inputArray0->GetNumberOfComponents() != inputArray1->GetNumberOfComponents())
  {
    vtkErrorMacro(<< "The number of components of the array in each time "
                     "step are different.");
    return nullptr;
  }

  if (inputArray0->GetNumberOfTuples() != inputArray1->GetNumberOfTuples())
  {
    vtkErrorMacro(<< "The number of tuples of the array in each time step"
                     "are different.");
    return nullptr;
  }

  // Copy input structure into output
  vtkDataObject* outputDataObject = inputData0->NewInstance();
  outputDataObject->ShallowCopy(inputData1);

  vtkDataSet* outputDataSet = vtkDataSet::SafeDownCast(outputDataObject);
  vtkGraph* outputGraph = vtkGraph::SafeDownCast(outputDataObject);
  vtkTable* outputTable = vtkTable::SafeDownCast(outputDataObject);

  vtkSmartPointer<vtkDataArray> outputArray;
  outputArray.TakeReference(this->ProcessDataArray(inputArray0, inputArray1));

  switch (this->GetInputArrayAssociation())
  {
    case vtkDataObject::FIELD_ASSOCIATION_CELLS:
      if (!outputDataSet)
      {
        vtkErrorMacro(<< "Bad input association for input data object.");
        return nullptr;
      }
      outputDataSet->GetCellData()->AddArray(outputArray);
      break;
    case vtkDataObject::FIELD_ASSOCIATION_NONE:
      outputDataObject->GetFieldData()->AddArray(outputArray);
      break;
    case vtkDataObject::FIELD_ASSOCIATION_VERTICES:
      if (!outputGraph)
      {
        vtkErrorMacro(<< "Bad input association for input data object.");
        return nullptr;
      }
      outputGraph->GetVertexData()->AddArray(outputArray);
      break;
    case vtkDataObject::FIELD_ASSOCIATION_EDGES:
      if (!outputGraph)
      {
        vtkErrorMacro(<< "Bad input association for input data object.");
        return nullptr;
      }
      outputGraph->GetEdgeData()->AddArray(outputArray);
      break;
    case vtkDataObject::FIELD_ASSOCIATION_ROWS:
      if (!outputTable)
      {
        vtkErrorMacro(<< "Bad input association for input data object.");
        return nullptr;
      }
      outputTable->GetRowData()->AddArray(outputArray);
      break;
    case vtkDataObject::FIELD_ASSOCIATION_POINTS:
    default:
      if (!outputDataSet)
      {
        vtkErrorMacro(<< "Bad input association for input data object.");
        return nullptr;
      }
      outputDataSet->GetPointData()->AddArray(outputArray);
      break;
  }

  return outputDataObject;
}

//------------------------------------------------------------------------------
struct TemporalDataOperatorWorker
{
  TemporalDataOperatorWorker(int op)
    : Operator(op)
  {
  }

  template <typename Array1T, typename Array2T, typename Array3T>
  void operator()(Array1T* src1, Array2T* src2, Array3T* dst)
  {
    using T = vtk::GetAPIType<Array3T>;

    VTK_ASSUME(src1->GetNumberOfComponents() == dst->GetNumberOfComponents());
    VTK_ASSUME(src2->GetNumberOfComponents() == dst->GetNumberOfComponents());

    const auto srcRange1 = vtk::DataArrayValueRange(src1);
    const auto srcRange2 = vtk::DataArrayValueRange(src2);
    auto dstRange = vtk::DataArrayValueRange(dst);

    switch (this->Operator)
    {
      case vtkTemporalArrayOperatorFilter::ADD:
        std::transform(srcRange1.cbegin(), srcRange1.cend(), srcRange2.cbegin(), dstRange.begin(),
          std::plus<T>{});
        break;
      case vtkTemporalArrayOperatorFilter::SUB:
        std::transform(srcRange1.cbegin(), srcRange1.cend(), srcRange2.cbegin(), dstRange.begin(),
          std::minus<T>{});
        break;
      case vtkTemporalArrayOperatorFilter::MUL:
        std::transform(srcRange1.cbegin(), srcRange1.cend(), srcRange2.cbegin(), dstRange.begin(),
          std::multiplies<T>{});
        break;
      case vtkTemporalArrayOperatorFilter::DIV:
        std::transform(srcRange1.cbegin(), srcRange1.cend(), srcRange2.cbegin(), dstRange.begin(),
          std::divides<T>{});
        break;
      default:
        std::copy(srcRange1.cbegin(), srcRange1.cend(), dstRange.begin());
        break;
    }
  }

  int Operator;
};

//------------------------------------------------------------------------------
vtkDataArray* vtkTemporalArrayOperatorFilter::ProcessDataArray(
  vtkDataArray* inputArray0, vtkDataArray* inputArray1)
{
  vtkAbstractArray* outputArray = vtkAbstractArray::CreateArray(inputArray0->GetDataType());
  vtkDataArray* outputDataArray = vtkDataArray::SafeDownCast(outputArray);

  outputDataArray->SetNumberOfComponents(inputArray0->GetNumberOfComponents());
  outputDataArray->SetNumberOfTuples(inputArray0->GetNumberOfTuples());
  outputDataArray->CopyComponentNames(inputArray0);

  std::string s = inputArray0->GetName() ? inputArray0->GetName() : "input_array";
  if (this->OutputArrayNameSuffix && strlen(this->OutputArrayNameSuffix) != 0)
  {
    s += this->OutputArrayNameSuffix;
  }
  else
  {
    switch (this->Operator)
    {
      case SUB:
        s += "_sub";
        break;
      case MUL:
        s += "_mul";
        break;
      case DIV:
        s += "_div";
        break;
      case ADD:
      default:
        s += "_add";
        break;
    }
  }
  outputDataArray->SetName(s.c_str());

  // Let's perform the operation on the array
  TemporalDataOperatorWorker worker(this->Operator);

  if (!vtkArrayDispatch::Dispatch3SameValueType::Execute(
        inputArray0, inputArray1, outputDataArray, worker))
  {
    worker(inputArray0, inputArray1, outputDataArray); // vtkDataArray fallback
  }

  return outputDataArray;
}
