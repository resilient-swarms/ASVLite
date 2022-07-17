/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkBitArrayIterator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkBitArrayIterator.h"

#include "vtkBitArray.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkBitArrayIterator);
vtkCxxSetObjectMacro(vtkBitArrayIterator, Array, vtkBitArray);
//------------------------------------------------------------------------------
vtkBitArrayIterator::vtkBitArrayIterator()
{
  this->Array = nullptr;
  this->Tuple = nullptr;
  this->TupleSize = 0;
}

//------------------------------------------------------------------------------
vtkBitArrayIterator::~vtkBitArrayIterator()
{
  this->SetArray(nullptr);
  delete[] this->Tuple;
}

//------------------------------------------------------------------------------
void vtkBitArrayIterator::Initialize(vtkAbstractArray* a)
{
  vtkBitArray* b = vtkArrayDownCast<vtkBitArray>(a);
  if (!b && a)
  {
    vtkErrorMacro("vtkBitArrayIterator can iterate only over vtkBitArray.");
    return;
  }
  this->SetArray(b);
}

//------------------------------------------------------------------------------
vtkAbstractArray* vtkBitArrayIterator::GetArray()
{
  return this->Array;
}

//------------------------------------------------------------------------------
int* vtkBitArrayIterator::GetTuple(vtkIdType id)
{
  if (!this->Array)
  {
    return nullptr;
  }

  vtkIdType numComps = this->Array->GetNumberOfComponents();
  if (this->TupleSize < numComps)
  {
    this->TupleSize = static_cast<int>(numComps);
    delete[] this->Tuple;
    this->Tuple = new int[this->TupleSize];
  }
  vtkIdType loc = id * numComps;
  for (int j = 0; j < numComps; j++)
  {
    this->Tuple[j] = this->Array->GetValue(loc + j);
  }
  return this->Tuple;
}

//------------------------------------------------------------------------------
int vtkBitArrayIterator::GetValue(vtkIdType id)
{
  if (this->Array)
  {
    return this->Array->GetValue(id);
  }
  vtkErrorMacro("Array Iterator not initialized.");
  return 0;
}

//------------------------------------------------------------------------------
void vtkBitArrayIterator::SetValue(vtkIdType id, int value)
{
  if (this->Array)
  {
    this->Array->SetValue(id, value);
  }
}

//------------------------------------------------------------------------------
vtkIdType vtkBitArrayIterator::GetNumberOfTuples() const
{
  if (this->Array)
  {
    return this->Array->GetNumberOfTuples();
  }
  return 0;
}
//------------------------------------------------------------------------------
vtkIdType vtkBitArrayIterator::GetNumberOfValues() const
{
  if (this->Array)
  {
    return this->Array->GetNumberOfTuples() * this->Array->GetNumberOfComponents();
  }
  return 0;
}
//------------------------------------------------------------------------------
int vtkBitArrayIterator::GetNumberOfComponents() const
{
  if (this->Array)
  {
    return this->Array->GetNumberOfComponents();
  }
  return 0;
}

//------------------------------------------------------------------------------
int vtkBitArrayIterator::GetDataType() const
{
  if (this->Array)
  {
    return this->Array->GetDataType();
  }
  return 0;
}
//------------------------------------------------------------------------------
int vtkBitArrayIterator::GetDataTypeSize() const
{
  if (this->Array)
  {
    return this->Array->GetDataTypeSize();
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkBitArrayIterator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
