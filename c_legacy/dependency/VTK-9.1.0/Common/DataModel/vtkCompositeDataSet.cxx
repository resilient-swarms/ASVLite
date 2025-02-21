/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCompositeDataSet.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkCompositeDataSet.h"

#include "vtkBoundingBox.h"
#include "vtkCompositeDataSetRange.h"
#include "vtkDataSet.h"
#include "vtkInformation.h"
#include "vtkInformationIntegerKey.h"
#include "vtkInformationStringKey.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"

#include <numeric>

vtkInformationKeyMacro(vtkCompositeDataSet, NAME, String);
vtkInformationKeyMacro(vtkCompositeDataSet, CURRENT_PROCESS_CAN_LOAD_BLOCK, Integer);

//------------------------------------------------------------------------------
vtkCompositeDataSet::vtkCompositeDataSet() = default;

//------------------------------------------------------------------------------
vtkCompositeDataSet::~vtkCompositeDataSet() = default;

//------------------------------------------------------------------------------
vtkCompositeDataSet* vtkCompositeDataSet::GetData(vtkInformation* info)
{
  return info ? vtkCompositeDataSet::SafeDownCast(info->Get(DATA_OBJECT())) : nullptr;
}

//------------------------------------------------------------------------------
vtkCompositeDataSet* vtkCompositeDataSet::GetData(vtkInformationVector* v, int i)
{
  return vtkCompositeDataSet::GetData(v->GetInformationObject(i));
}

//------------------------------------------------------------------------------
void vtkCompositeDataSet::CopyStructure(vtkCompositeDataSet* input)
{
  if (input != this)
  {
    // copy data-information and other common stuff by calling
    // superclass' ShallowCopy.
    this->Superclass::ShallowCopy(input);
  }
}

//------------------------------------------------------------------------------
void vtkCompositeDataSet::ShallowCopy(vtkDataObject* src)
{
  if (src == this)
  {
    return;
  }

  this->Superclass::ShallowCopy(src);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkCompositeDataSet::DeepCopy(vtkDataObject* src)
{
  if (src == this)
  {
    return;
  }

  this->Superclass::DeepCopy(src);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkCompositeDataSet::Initialize()
{
  this->Superclass::Initialize();
}

//------------------------------------------------------------------------------
unsigned long vtkCompositeDataSet::GetActualMemorySize()
{
  using Opts = vtk::CompositeDataSetOptions;
  unsigned long memSize = 0;
  // Note: This loop cannot be turned into `std::accumulate` because
  // vtkCompositeDataSetRange has an issue when the `end` iterator is copied
  // into the algorithm on MSVC. See #18150.
  for (vtkDataObject* block : vtk::Range(this, Opts::SkipEmptyNodes))
  {
    assert(vtkCompositeDataSet::SafeDownCast(block) == nullptr && block != nullptr);
    memSize += block->GetActualMemorySize();
  }
  return memSize;
}

//------------------------------------------------------------------------------
vtkIdType vtkCompositeDataSet::GetNumberOfPoints()
{
  return this->GetNumberOfElements(vtkDataSet::POINT);
}

//------------------------------------------------------------------------------
vtkIdType vtkCompositeDataSet::GetNumberOfCells()
{
  return this->GetNumberOfElements(vtkDataSet::CELL);
}

//------------------------------------------------------------------------------
vtkIdType vtkCompositeDataSet::GetNumberOfElements(int type)
{
  using Opts = vtk::CompositeDataSetOptions;
  vtkIdType numElements = 0;
  // Note: This loop cannot be turned into `std::accumulate` because
  // vtkCompositeDataSetRange has an issue when the `end` iterator is copied
  // into the algorithm on MSVC. See #18150.
  for (vtkDataObject* block : vtk::Range(this, Opts::SkipEmptyNodes))
  {
    assert(vtkCompositeDataSet::SafeDownCast(block) == nullptr && block != nullptr);
    numElements += block->GetNumberOfElements(type);
  };

  // Call superclass to ensure we don't miss field data tuples.
  return numElements += this->Superclass::GetNumberOfElements(type);
}

//------------------------------------------------------------------------------
void vtkCompositeDataSet::GetBounds(double bounds[6])
{
  using Opts = vtk::CompositeDataSetOptions;
  double bds[6];
  vtkBoundingBox bbox;
  for (vtkDataObject* dobj : vtk::Range(this, Opts::SkipEmptyNodes))
  {
    if (auto ds = vtkDataSet::SafeDownCast(dobj))
    {
      ds->GetBounds(bds);
      bbox.AddBounds(bds);
    }
  }
  bbox.GetBounds(bounds);
}

//------------------------------------------------------------------------------
void vtkCompositeDataSet::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
