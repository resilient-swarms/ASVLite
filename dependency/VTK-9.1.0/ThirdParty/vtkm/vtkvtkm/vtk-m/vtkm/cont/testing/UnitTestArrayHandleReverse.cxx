//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/cont/ArrayHandleReverse.h>

#include <vtkm/cont/ArrayHandleIndex.h>
#include <vtkm/cont/serial/DeviceAdapterSerial.h>
#include <vtkm/cont/testing/Testing.h>

namespace UnitTestArrayHandleReverseNamespace
{

const vtkm::Id ARRAY_SIZE = 10;

void TestArrayHandleReverseRead()
{
  vtkm::cont::ArrayHandleIndex array(ARRAY_SIZE);
  VTKM_TEST_ASSERT(array.GetNumberOfValues() == ARRAY_SIZE, "Bad size.");

  auto portal = array.ReadPortal();
  for (vtkm::Id index = 0; index < ARRAY_SIZE; index++)
  {
    VTKM_TEST_ASSERT(portal.Get(index) == index, "Index array has unexpected value.");
  }

  vtkm::cont::ArrayHandleReverse<vtkm::cont::ArrayHandleIndex> reverse =
    vtkm::cont::make_ArrayHandleReverse(array);

  auto reversedPortal = reverse.ReadPortal();
  for (vtkm::Id index = 0; index < ARRAY_SIZE; index++)
  {
    VTKM_TEST_ASSERT(reversedPortal.Get(index) == portal.Get(9 - index),
                     "ArrayHandleReverse does not reverse array");
  }
}

void TestArrayHandleReverseWrite()
{
  std::vector<vtkm::Id> ids(ARRAY_SIZE, 0);
  vtkm::cont::ArrayHandle<vtkm::Id> handle = vtkm::cont::make_ArrayHandle(ids, vtkm::CopyFlag::Off);

  vtkm::cont::ArrayHandleReverse<vtkm::cont::ArrayHandle<vtkm::Id>> reverse =
    vtkm::cont::make_ArrayHandleReverse(handle);

  for (vtkm::Id index = 0; index < ARRAY_SIZE; index++)
  {
    reverse.WritePortal().Set(index, index);
  }

  auto portal = handle.ReadPortal();
  for (vtkm::Id index = 0; index < ARRAY_SIZE; index++)
  {
    VTKM_TEST_ASSERT(portal.Get(index) == (9 - index), "ArrayHandleReverse does not reverse array");
  }
}

void TestArrayHandleReverseScanInclusiveByKey()
{
  vtkm::cont::ArrayHandle<vtkm::Id> values =
    vtkm::cont::make_ArrayHandle<vtkm::Id>({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 });
  vtkm::cont::ArrayHandle<vtkm::Id> keys =
    vtkm::cont::make_ArrayHandle<vtkm::Id>({ 0, 0, 0, 0, 1, 1, 2, 3, 3, 4 });

  vtkm::cont::ArrayHandle<vtkm::Id> output;
  vtkm::cont::ArrayHandleReverse<vtkm::cont::ArrayHandle<vtkm::Id>> reversed =
    vtkm::cont::make_ArrayHandleReverse(output);

  using Algorithm = vtkm::cont::DeviceAdapterAlgorithm<vtkm::cont::DeviceAdapterTagSerial>;
  Algorithm::ScanInclusiveByKey(keys, values, reversed);

  vtkm::cont::ArrayHandleReverse<vtkm::cont::ArrayHandle<vtkm::Id>> expected_reversed =
    vtkm::cont::make_ArrayHandleReverse(
      vtkm::cont::make_ArrayHandle<vtkm::Id>({ 0, 1, 3, 6, 4, 9, 6, 7, 15, 9 }));
  auto outputPortal = output.ReadPortal();
  auto reversePortal = expected_reversed.ReadPortal();
  for (int i = 0; i < 10; i++)
  {
    VTKM_TEST_ASSERT(outputPortal.Get(i) == reversePortal.Get(i),
                     "ArrayHandleReverse as output of ScanInclusiveByKey");
  }
  std::cout << std::endl;
}

void TestArrayHandleReverse()
{
  TestArrayHandleReverseRead();
  TestArrayHandleReverseWrite();
  TestArrayHandleReverseScanInclusiveByKey();
}

}; // namespace UnitTestArrayHandleReverseNamespace

int UnitTestArrayHandleReverse(int argc, char* argv[])
{
  using namespace UnitTestArrayHandleReverseNamespace;
  return vtkm::cont::testing::Testing::Run(TestArrayHandleReverse, argc, argv);
}
