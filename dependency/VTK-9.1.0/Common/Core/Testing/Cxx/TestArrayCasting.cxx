/*=========================================================================

  Program:   Visualization Toolkit
  Module:    ArrayCasting.cxx

-------------------------------------------------------------------------
  Copyright 2008 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
  the U.S. Government retains certain rights in this software.
-------------------------------------------------------------------------

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include <vtkDenseArray.h>
#include <vtkSmartPointer.h>
#include <vtkSparseArray.h>
#include <vtkTryDowncast.h>

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

#define VTK_CREATE(type, name) vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

#define test_expression(expression)                                                                \
  {                                                                                                \
    if (!(expression))                                                                             \
    {                                                                                              \
      std::ostringstream buffer;                                                                   \
      buffer << "Expression failed at line " << __LINE__ << ": " << #expression;                   \
      throw std::runtime_error(buffer.str());                                                      \
    }                                                                                              \
  }

class DowncastTest
{
public:
  DowncastTest(int& count)
    : Count(count)
  {
  }

  template <typename T>
  void operator()(T* vtkNotUsed(array)) const
  {
    ++Count;
  }

  int& Count;

private:
  DowncastTest& operator=(const DowncastTest&);
};

template <template <typename> class TargetT, typename TypesT>
void SuccessTest(vtkObject* source, int line)
{
  int count = 0;
  if (!vtkTryDowncast<TargetT, TypesT>(source, DowncastTest(count)))
  {
    std::ostringstream buffer;
    buffer << "Expression failed at line " << line;
    throw std::runtime_error(buffer.str());
  }

  if (count != 1)
  {
    std::ostringstream buffer;
    buffer << "Functor was called " << count << " times at line " << line;
    throw std::runtime_error(buffer.str());
  }
}

template <template <typename> class TargetT, typename TypesT>
void FailTest(vtkObject* source, int line)
{
  int count = 0;
  if (vtkTryDowncast<TargetT, TypesT>(source, DowncastTest(count)))
  {
    std::ostringstream buffer;
    buffer << "Expression failed at line " << line;
    throw std::runtime_error(buffer.str());
  }

  if (count != 0)
  {
    std::ostringstream buffer;
    buffer << "Functor was called " << count << " times at line " << line;
    throw std::runtime_error(buffer.str());
  }
}

/*
// This functor increments array values in-place using a parameter passed via the algorithm (instead
of a parameter
// stored in the functor).  It can work with any numeric array type.
struct IncrementValues
{
  template<typename T>
  void operator()(T* array, int amount) const
  {
    for(vtkIdType n = 0; n != array->GetNonNullSize(); ++n)
      array->SetValueN(n, array->GetValueN(n) + amount);
  }
};

// This functor converts strings in-place to a form suitable for case-insensitive comparison.  It's
an example of
// how you can write generic code while still specializing functionality on a case-by-case basis,
since
// in this situation we want to use some special functionality provided by vtkUnicodeString.
struct FoldCase
{
  template<typename ValueT>
  void operator()(vtkTypedArray<ValueT>* array) const
  {
    for(vtkIdType n = 0; n != array->GetNonNullSize(); ++n)
      {
      ValueT value = array->GetValueN(n);
      boost::algorithm::to_lower(value);
      array->SetValueN(n, value);
      }
  }

  void operator()(vtkTypedArray<vtkUnicodeString>* array) const
  {
    for(vtkIdType n = 0; n != array->GetNonNullSize(); ++n)
      array->SetValueN(n, array->GetValueN(n).fold_case());
  }
};

// This functor efficiently creates a transposed array.  It's one example of how you can create an
output array
// with the same type as an input array.
struct Transpose
{
  Transpose(vtkSmartPointer<vtkArray>& result_matrix) : ResultMatrix(result_matrix) {}

  template<typename ValueT>
  void operator()(vtkDenseArray<ValueT>* input) const
  {
    if(input->GetDimensions() != 2 || input->GetExtents()[0] != input->GetExtents()[1])
      throw std::runtime_error("A square matrix is required.");

    vtkDenseArray<ValueT>* output = vtkDenseArray<ValueT>::SafeDownCast(input->DeepCopy());
    for(vtkIdType i = 0; i != input->GetExtents()[0]; ++i)
      {
      for(vtkIdType j = i + 1; j != input->GetExtents()[1]; ++j)
        {
        output->SetValue(i, j, input->GetValue(j, i));
        output->SetValue(j, i, input->GetValue(i, j));
        }
      }

    this->ResultMatrix = output;
  }

  vtkSmartPointer<vtkArray>& ResultMatrix;
};
*/

//
//
// Here are some examples of how the algorithm might be called.
//
//

int TestArrayCasting(int vtkNotUsed(argc), char* vtkNotUsed(argv)[])
{
  try
  {
    VTK_CREATE(vtkDenseArray<int>, dense_int);
    VTK_CREATE(vtkDenseArray<double>, dense_double);
    VTK_CREATE(vtkDenseArray<vtkStdString>, dense_string);
    VTK_CREATE(vtkSparseArray<int>, sparse_int);
    VTK_CREATE(vtkSparseArray<double>, sparse_double);
    VTK_CREATE(vtkSparseArray<vtkStdString>, sparse_string);

    SuccessTest<vtkTypedArray, vtkIntegerTypes>(dense_int, __LINE__);
    FailTest<vtkTypedArray, vtkIntegerTypes>(dense_double, __LINE__);
    FailTest<vtkTypedArray, vtkIntegerTypes>(dense_string, __LINE__);
    SuccessTest<vtkTypedArray, vtkIntegerTypes>(sparse_int, __LINE__);
    FailTest<vtkTypedArray, vtkIntegerTypes>(sparse_double, __LINE__);
    FailTest<vtkTypedArray, vtkIntegerTypes>(sparse_string, __LINE__);

    FailTest<vtkTypedArray, vtkFloatingPointTypes>(dense_int, __LINE__);
    SuccessTest<vtkTypedArray, vtkFloatingPointTypes>(dense_double, __LINE__);
    FailTest<vtkTypedArray, vtkFloatingPointTypes>(dense_string, __LINE__);
    FailTest<vtkTypedArray, vtkFloatingPointTypes>(sparse_int, __LINE__);
    SuccessTest<vtkTypedArray, vtkFloatingPointTypes>(sparse_double, __LINE__);
    FailTest<vtkTypedArray, vtkFloatingPointTypes>(sparse_string, __LINE__);

    SuccessTest<vtkTypedArray, vtkNumericTypes>(dense_int, __LINE__);
    SuccessTest<vtkTypedArray, vtkNumericTypes>(dense_double, __LINE__);
    FailTest<vtkTypedArray, vtkNumericTypes>(dense_string, __LINE__);
    SuccessTest<vtkTypedArray, vtkNumericTypes>(sparse_int, __LINE__);
    SuccessTest<vtkTypedArray, vtkNumericTypes>(sparse_double, __LINE__);
    FailTest<vtkTypedArray, vtkNumericTypes>(sparse_string, __LINE__);

    FailTest<vtkTypedArray, vtkStringTypes>(dense_int, __LINE__);
    FailTest<vtkTypedArray, vtkStringTypes>(dense_double, __LINE__);
    SuccessTest<vtkTypedArray, vtkStringTypes>(dense_string, __LINE__);
    FailTest<vtkTypedArray, vtkStringTypes>(sparse_int, __LINE__);
    FailTest<vtkTypedArray, vtkStringTypes>(sparse_double, __LINE__);
    SuccessTest<vtkTypedArray, vtkStringTypes>(sparse_string, __LINE__);

    SuccessTest<vtkTypedArray, vtkAllTypes>(dense_int, __LINE__);
    SuccessTest<vtkTypedArray, vtkAllTypes>(dense_double, __LINE__);
    SuccessTest<vtkTypedArray, vtkAllTypes>(dense_string, __LINE__);
    SuccessTest<vtkTypedArray, vtkAllTypes>(sparse_int, __LINE__);
    SuccessTest<vtkTypedArray, vtkAllTypes>(sparse_double, __LINE__);
    SuccessTest<vtkTypedArray, vtkAllTypes>(sparse_string, __LINE__);

    SuccessTest<vtkDenseArray, vtkAllTypes>(dense_int, __LINE__);
    FailTest<vtkDenseArray, vtkAllTypes>(sparse_int, __LINE__);
    FailTest<vtkSparseArray, vtkAllTypes>(dense_int, __LINE__);
    SuccessTest<vtkSparseArray, vtkAllTypes>(sparse_int, __LINE__);

    return 0;
  }
  catch (std::exception& e)
  {
    cerr << e.what() << endl;
    return 1;
  }
}
