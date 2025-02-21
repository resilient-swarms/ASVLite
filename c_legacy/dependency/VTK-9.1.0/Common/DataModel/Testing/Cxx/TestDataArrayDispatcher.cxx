/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestDispatchers.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME Test Dispatchers
// .SECTION Description
// Tests vtkDataArrayDispatcher

// Hide VTK_DEPRECATED_IN_9_0_0() warnings for this file.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkDataArrayDispatcher.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"

// classes we will be using in the test
#include "vtkDoubleArray.h"
#include "vtkIntArray.h"

#include <algorithm>
#include <stdexcept>
namespace
{

void test_expression(bool valid, const std::string& msg)
{
  if (!valid)
  {
    throw std::runtime_error(msg);
  }
}

template <typename T, typename U>
inline T* as(U* u)
{
  return dynamic_cast<T*>(u);
}

// returns the length of an array
struct lengthCheckFunctor
{
  template <typename T>
  vtkIdType operator()(const vtkDataArrayDispatcherPointer<T>& array) const
  {
    return array.NumberOfComponents * array.NumberOfTuples;
  }
};

// accumulates the length of all arrays used with this functor
struct storeLengthFunctor
{
  vtkIdType length;
  storeLengthFunctor()
    : length(0)
  {
  }

  template <typename T>
  void operator()(vtkDataArrayDispatcherPointer<T> array)
  {
    length += array.NumberOfComponents * array.NumberOfTuples;
  }
};

// modifies an array to be sorted, only works with arrays
// that have one component
struct sortArray
{
  template <typename T>
  void operator()(vtkDataArrayDispatcherPointer<T> array) const
  {
    std::sort(array.RawPointer, array.RawPointer + array.NumberOfTuples);
  }
};

bool TestDataArrayDispatchStatefull()
{
  storeLengthFunctor functor;
  vtkDataArrayDispatcher<storeLengthFunctor> dispatcher(functor);

  // verify the dispatching
  vtkNew<vtkDoubleArray> doubleArray;
  vtkNew<vtkIntArray> intArray;

  doubleArray->SetNumberOfComponents(4);
  doubleArray->SetNumberOfTuples(10);
  intArray->SetNumberOfTuples(13);

  const vtkIdType doubleSize =
    doubleArray->GetNumberOfComponents() * doubleArray->GetNumberOfTuples();
  const vtkIdType intSize = intArray->GetNumberOfComponents() * intArray->GetNumberOfTuples();

  dispatcher.Go(as<vtkDataArray>(doubleArray.GetPointer()));
  test_expression(
    functor.length == doubleSize, "double array dispatch failed with statefull functor");

  dispatcher.Go(intArray.GetPointer());
  test_expression(
    functor.length == intSize + doubleSize, "int array dispatch failed with statefull functor");

  return true;
}

bool TestDataArrayDispatchStateless()
{
  vtkDataArrayDispatcher<lengthCheckFunctor, int> dispatcher;

  // verify the dispatching
  vtkNew<vtkDoubleArray> doubleArray;
  vtkNew<vtkIntArray> intArray;

  doubleArray->SetNumberOfComponents(4);
  doubleArray->SetNumberOfTuples(10);
  intArray->SetNumberOfTuples(13);

  const vtkIdType doubleSize =
    doubleArray->GetNumberOfComponents() * doubleArray->GetNumberOfTuples();
  const vtkIdType intSize = intArray->GetNumberOfComponents() * intArray->GetNumberOfTuples();

  vtkIdType result = dispatcher.Go(doubleArray.GetPointer());
  test_expression(result == doubleSize, "double array dispatch failed with stateless functor");
  result = dispatcher.Go(intArray.GetPointer());
  test_expression(result == intSize, "int array dispatch failed with stateless functor");
  return true;
}

bool TestDataArrayDispatchSort()
{
  vtkDataArrayDispatcher<sortArray> dispatcher;

  // verify the dispatching
  vtkNew<vtkDoubleArray> doubleArray;

  const int doubleSize = 10;
  doubleArray->SetNumberOfTuples(doubleSize);

  for (int i = 0; i < doubleSize; i++)
  {
    doubleArray->SetValue(i, doubleSize - i);
  }

  dispatcher.Go(doubleArray.GetPointer());

  for (int i = 0; i < doubleSize; i++)
  {
    test_expression(doubleArray->GetValue(i) == i + 1, "sort functor failed");
  }

  return true;
}

}

int TestDataArrayDispatcher(int /*argc*/, char* /*argv*/[])
{

  bool passed = TestDataArrayDispatchStatefull();
  passed &= TestDataArrayDispatchStateless();
  passed &= TestDataArrayDispatchSort();
  return passed ? 0 : 1;
}
