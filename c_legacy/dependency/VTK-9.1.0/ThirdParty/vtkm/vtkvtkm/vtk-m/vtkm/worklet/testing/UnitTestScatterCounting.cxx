//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/worklet/ScatterCounting.h>

#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/DeviceAdapterAlgorithm.h>

#include <vtkm/worklet/DispatcherMapField.h>
#include <vtkm/worklet/WorkletMapField.h>

#include <vtkm/cont/testing/Testing.h>

#include <vector>

namespace
{

struct TestScatterArrays
{
  vtkm::cont::ArrayHandle<vtkm::IdComponent> CountArray;
  vtkm::cont::ArrayHandle<vtkm::Id> InputToOutputMap;
  vtkm::cont::ArrayHandle<vtkm::Id> OutputToInputMap;
  vtkm::cont::ArrayHandle<vtkm::IdComponent> VisitArray;
};

TestScatterArrays MakeScatterArraysShort()
{
  TestScatterArrays arrays;
  arrays.CountArray = vtkm::cont::make_ArrayHandle<vtkm::IdComponent>(
    { 1, 2, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 });
  arrays.InputToOutputMap = vtkm::cont::make_ArrayHandle<vtkm::Id>(
    { 0, 1, 3, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6 });
  arrays.OutputToInputMap = vtkm::cont::make_ArrayHandle<vtkm::Id>({ 0, 1, 1, 4, 6, 14 });
  arrays.VisitArray = vtkm::cont::make_ArrayHandle<vtkm::IdComponent>({ 0, 0, 1, 0, 0, 0 });

  return arrays;
}

TestScatterArrays MakeScatterArraysLong()
{
  TestScatterArrays arrays;
  arrays.CountArray = vtkm::cont::make_ArrayHandle<vtkm::IdComponent>({ 0, 1, 2, 3, 4, 5 });
  arrays.InputToOutputMap = vtkm::cont::make_ArrayHandle<vtkm::Id>({ 0, 0, 1, 3, 6, 10 });
  arrays.OutputToInputMap =
    vtkm::cont::make_ArrayHandle<vtkm::Id>({ 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5 });
  arrays.VisitArray = vtkm::cont::make_ArrayHandle<vtkm::IdComponent>(
    { 0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 3, 4 });

  return arrays;
}

TestScatterArrays MakeScatterArraysZero()
{
  TestScatterArrays arrays;
  arrays.CountArray = vtkm::cont::make_ArrayHandle<vtkm::IdComponent>({ 0, 0, 0, 0, 0, 0 });
  arrays.InputToOutputMap = vtkm::cont::make_ArrayHandle<vtkm::Id>({ 0, 0, 0, 0, 0, 0 });
  arrays.OutputToInputMap.Allocate(0);
  arrays.VisitArray.Allocate(0);

  return arrays;
}

struct TestScatterCountingWorklet : public vtkm::worklet::WorkletMapField
{
  using ControlSignature = void(FieldIn inputIndices,
                                FieldOut copyIndices,
                                FieldOut recordVisit,
                                FieldOut recordWorkId);
  using ExecutionSignature = void(_1, _2, _3, _4, VisitIndex, WorkIndex);

  using ScatterType = vtkm::worklet::ScatterCounting;

  template <typename CountArrayType>
  VTKM_CONT static ScatterType MakeScatter(const CountArrayType& countArray)
  {
    return ScatterType(countArray);
  }

  VTKM_EXEC
  void operator()(vtkm::Id inputIndex,
                  vtkm::Id& indexCopy,
                  vtkm::IdComponent& writeVisit,
                  vtkm::Float32& captureWorkId,
                  vtkm::IdComponent visitIndex,
                  vtkm::Id workId) const
  {
    indexCopy = inputIndex;
    writeVisit = visitIndex;
    captureWorkId = TestValue(workId, vtkm::Float32());
  }
};

template <typename T>
void CompareArrays(vtkm::cont::ArrayHandle<T> array1, vtkm::cont::ArrayHandle<T> array2)
{
  using PortalType = typename vtkm::cont::ArrayHandle<T>::ReadPortalType;
  PortalType portal1 = array1.ReadPortal();
  PortalType portal2 = array2.ReadPortal();

  VTKM_TEST_ASSERT(portal1.GetNumberOfValues() == portal2.GetNumberOfValues(),
                   "Arrays are not the same length.");

  for (vtkm::Id index = 0; index < portal1.GetNumberOfValues(); index++)
  {
    T value1 = portal1.Get(index);
    T value2 = portal2.Get(index);
    VTKM_TEST_ASSERT(value1 == value2, "Array values not equal.");
  }
}

// This unit test makes sure the ScatterCounting generates the correct map
// and visit arrays.
void TestScatterArrayGeneration(const TestScatterArrays& arrays)
{
  std::cout << "  Testing array generation" << std::endl;

  vtkm::worklet::ScatterCounting scatter(
    arrays.CountArray, vtkm::cont::DeviceAdapterTagAny(), true);

  vtkm::Id inputSize = arrays.CountArray.GetNumberOfValues();

  std::cout << "    Checking input to output map." << std::endl;
  CompareArrays(arrays.InputToOutputMap, scatter.GetInputToOutputMap());

  std::cout << "    Checking output to input map." << std::endl;
  CompareArrays(arrays.OutputToInputMap, scatter.GetOutputToInputMap(inputSize));

  std::cout << "    Checking visit array." << std::endl;
  CompareArrays(arrays.VisitArray, scatter.GetVisitArray(inputSize));
}

// This is more of an integration test that makes sure the scatter works with a
// worklet invocation.
void TestScatterWorklet(const TestScatterArrays& arrays)
{
  std::cout << "  Testing scatter counting in a worklet." << std::endl;

  vtkm::worklet::DispatcherMapField<TestScatterCountingWorklet> dispatcher(
    TestScatterCountingWorklet::MakeScatter(arrays.CountArray));

  vtkm::Id inputSize = arrays.CountArray.GetNumberOfValues();
  vtkm::cont::ArrayHandleIndex inputIndices(inputSize);
  vtkm::cont::ArrayHandle<vtkm::Id> outputToInputMapCopy;
  vtkm::cont::ArrayHandle<vtkm::IdComponent> visitCopy;
  vtkm::cont::ArrayHandle<vtkm::Float32> captureWorkId;

  std::cout << "    Invoke worklet" << std::endl;
  dispatcher.Invoke(inputIndices, outputToInputMapCopy, visitCopy, captureWorkId);

  std::cout << "    Check output to input map." << std::endl;
  CompareArrays(outputToInputMapCopy, arrays.OutputToInputMap);
  std::cout << "    Check visit." << std::endl;
  CompareArrays(visitCopy, arrays.VisitArray);
  std::cout << "    Check work id." << std::endl;
  CheckPortal(captureWorkId.ReadPortal());
}

void TestScatterCountingWithArrays(const TestScatterArrays& arrays)
{
  TestScatterArrayGeneration(arrays);
  TestScatterWorklet(arrays);
}

void TestScatterCounting()
{
  std::cout << "Testing arrays with output smaller than input." << std::endl;
  TestScatterCountingWithArrays(MakeScatterArraysShort());

  std::cout << "Testing arrays with output larger than input." << std::endl;
  TestScatterCountingWithArrays(MakeScatterArraysLong());

  std::cout << "Testing arrays with zero output." << std::endl;
  TestScatterCountingWithArrays(MakeScatterArraysZero());
}

} // anonymous namespace

int UnitTestScatterCounting(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(TestScatterCounting, argc, argv);
}
