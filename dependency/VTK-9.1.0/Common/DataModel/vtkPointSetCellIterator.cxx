/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPointSetCellIterator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkPointSetCellIterator.h"

#include "vtkIdList.h"
#include "vtkObjectFactory.h"
#include "vtkPointSet.h"
#include "vtkPoints.h"

vtkStandardNewMacro(vtkPointSetCellIterator);

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "PointSet: " << this->PointSet << endl;
}

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::SetPointSet(vtkPointSet* ds)
{
  this->PointSet = ds;
  this->PointSetPoints = ds ? ds->GetPoints() : nullptr;
  this->CellId = 0;
  if (this->PointSetPoints)
  {
    this->Points->SetDataType(this->PointSetPoints->GetDataType());
  }
}

//------------------------------------------------------------------------------
bool vtkPointSetCellIterator::IsDoneWithTraversal()
{
  return this->PointSet == nullptr || this->CellId >= this->PointSet->GetNumberOfCells();
}

//------------------------------------------------------------------------------
vtkIdType vtkPointSetCellIterator::GetCellId()
{
  return this->CellId;
}

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::IncrementToNextCell()
{
  ++this->CellId;
}

//------------------------------------------------------------------------------
vtkPointSetCellIterator::vtkPointSetCellIterator()
  : PointSet(nullptr)
  , PointSetPoints(nullptr)
  , CellId(0)
{
}

//------------------------------------------------------------------------------
vtkPointSetCellIterator::~vtkPointSetCellIterator() = default;

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::ResetToFirstCell()
{
  this->CellId = 0;
}

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::FetchCellType()
{
  this->CellType = this->PointSet->GetCellType(this->CellId);
}

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::FetchPointIds()
{
  this->PointSet->GetCellPoints(this->CellId, this->PointIds);
}

//------------------------------------------------------------------------------
void vtkPointSetCellIterator::FetchPoints()
{
  vtkIdList* pointIds = this->GetPointIds();
  this->PointSetPoints->GetPoints(pointIds, this->Points);
}
