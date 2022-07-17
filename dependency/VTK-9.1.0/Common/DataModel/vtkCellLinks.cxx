/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCellLinks.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkCellLinks.h"

#include "vtkCellArray.h"
#include "vtkDataSet.h"
#include "vtkGenericCell.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkSMPTools.h"

#include <vector>

vtkStandardNewMacro(vtkCellLinks);

//------------------------------------------------------------------------------
vtkCellLinks::~vtkCellLinks()
{
  this->Type = vtkAbstractCellLinks::CELL_LINKS;
  this->Initialize();
}

//------------------------------------------------------------------------------
void vtkCellLinks::Initialize()
{
  if (this->Array != nullptr)
  {
    for (vtkIdType i = 0; i <= this->MaxId; i++)
    {
      delete[] this->Array[i].cells;
    }

    delete[] this->Array;
    this->Array = nullptr;
  }
  this->NumberOfPoints = 0;
  this->NumberOfCells = 0;
}

//------------------------------------------------------------------------------
void vtkCellLinks::Allocate(vtkIdType sz, vtkIdType ext)
{
  static vtkCellLinks::Link linkInit = { 0, nullptr };

  this->Size = sz;
  delete[] this->Array;
  this->Array = new vtkCellLinks::Link[sz];
  this->Extend = ext;
  this->MaxId = -1;

  for (vtkIdType i = 0; i < sz; i++)
  {
    this->Array[i] = linkInit;
  }
}

//------------------------------------------------------------------------------
// Allocate memory for the list of lists of cell ids.
void vtkCellLinks::AllocateLinks(vtkIdType n)
{
  for (vtkIdType i = 0; i < n; i++)
  {
    this->Array[i].cells = new vtkIdType[this->Array[i].ncells];
  }
}

//------------------------------------------------------------------------------
// Reclaim any unused memory.
void vtkCellLinks::Squeeze()
{
  this->Resize(this->MaxId + 1);
}

//------------------------------------------------------------------------------
void vtkCellLinks::Reset()
{
  this->MaxId = -1;
}

//------------------------------------------------------------------------------
//
// Private function does "reallocate"
//
vtkCellLinks::Link* vtkCellLinks::Resize(vtkIdType sz)
{
  vtkIdType i;
  vtkCellLinks::Link* newArray;
  vtkIdType newSize;
  vtkCellLinks::Link linkInit = { 0, nullptr };

  if (sz >= this->Size)
  {
    newSize = this->Size + sz;
  }
  else
  {
    newSize = sz;
  }

  newArray = new vtkCellLinks::Link[newSize];

  for (i = 0; i < sz && i < this->Size; i++)
  {
    newArray[i] = this->Array[i];
  }

  for (i = this->Size; i < newSize; i++)
  {
    newArray[i] = linkInit;
  }

  this->Size = newSize;
  delete[] this->Array;
  this->Array = newArray;

  return this->Array;
}

//------------------------------------------------------------------------------
// Build the link list array.
void vtkCellLinks::BuildLinks(vtkDataSet* data)
{
  vtkIdType numPts = this->NumberOfPoints = data->GetNumberOfPoints();
  vtkIdType numCells = this->NumberOfCells = data->GetNumberOfCells();
  int j;
  vtkIdType cellId;

  // If this method is called outside of dataset (e.g.,
  // vtkPolyData::BuildLinks()) then will have to perform initial link
  // allocation.
  if (this->Array == nullptr)
  {
    this->Allocate(numPts);
  }

  // fill out lists with number of references to cells
  std::vector<vtkIdType> linkLoc(numPts, 0);

  // Use fast path if polydata
  if (data->GetDataObjectType() == VTK_POLY_DATA)
  {
    vtkIdType npts;
    const vtkIdType* pts;

    vtkPolyData* pdata = static_cast<vtkPolyData*>(data);
    // traverse data to determine number of uses of each point
    for (cellId = 0; cellId < numCells; cellId++)
    {
      pdata->GetCellPoints(cellId, npts, pts);
      for (j = 0; j < npts; j++)
      {
        this->IncrementLinkCount(pts[j]);
      }
    }

    // now allocate storage for the links
    this->AllocateLinks(numPts);
    this->MaxId = numPts - 1;

    for (cellId = 0; cellId < numCells; cellId++)
    {
      pdata->GetCellPoints(cellId, npts, pts);
      for (j = 0; j < npts; j++)
      {
        this->InsertCellReference(pts[j], (linkLoc[pts[j]])++, cellId);
      }
    }
  }

  else // any other type of dataset
  {
    vtkIdType numberOfPoints, ptId;
    vtkGenericCell* cell = vtkGenericCell::New();

    // traverse data to determine number of uses of each point
    for (cellId = 0; cellId < numCells; cellId++)
    {
      data->GetCell(cellId, cell);
      numberOfPoints = cell->GetNumberOfPoints();
      for (j = 0; j < numberOfPoints; j++)
      {
        this->IncrementLinkCount(cell->PointIds->GetId(j));
      }
    }

    // now allocate storage for the links
    this->AllocateLinks(numPts);
    this->MaxId = numPts - 1;

    for (cellId = 0; cellId < numCells; cellId++)
    {
      data->GetCell(cellId, cell);
      numberOfPoints = cell->GetNumberOfPoints();
      for (j = 0; j < numberOfPoints; j++)
      {
        ptId = cell->PointIds->GetId(j);
        this->InsertCellReference(ptId, (linkLoc[ptId])++, cellId);
      }
    }
    cell->Delete();
  } // end else
}

//------------------------------------------------------------------------------
// Insert a new point into the cell-links data structure. The size parameter
// is the initial size of the list.
vtkIdType vtkCellLinks::InsertNextPoint(int numLinks)
{
  if (++this->MaxId >= this->Size)
  {
    this->Resize(this->MaxId + 1);
  }
  this->Array[this->MaxId].cells = new vtkIdType[numLinks];
  return this->MaxId;
}

//------------------------------------------------------------------------------
// Mark cells with one or more points whose degree lies in the range indicated.
void vtkCellLinks::SelectCells(vtkIdType minMaxDegree[2], unsigned char* cellSelection)
{
  std::fill_n(cellSelection, this->NumberOfCells, 0);
  vtkSMPTools::For(0, this->NumberOfPoints,
    [this, minMaxDegree, cellSelection](vtkIdType ptId, vtkIdType endPtId) {
      for (; ptId < endPtId; ++ptId)
      {
        vtkIdType degree = this->GetNcells(0);
        if (degree >= minMaxDegree[0] && degree < minMaxDegree[1])
        {
          vtkIdType* cells = this->GetCells(ptId);
          for (auto i = 0; i < degree; ++i)
          {
            cellSelection[cells[i]] = 1;
          }
        }
      } // for all points in this batch
    }); // end lambda
}

//------------------------------------------------------------------------------
unsigned long vtkCellLinks::GetActualMemorySize()
{
  vtkIdType size = 0;
  vtkIdType ptId;

  for (ptId = 0; ptId < (this->MaxId + 1); ptId++)
  {
    size += this->GetNcells(ptId);
  }

  size *= sizeof(int*);                                   // references to cells
  size += (this->MaxId + 1) * sizeof(vtkCellLinks::Link); // list of cell lists

  return static_cast<unsigned long>(ceil(size / 1024.0)); // kibibytes
}

//------------------------------------------------------------------------------
void vtkCellLinks::DeepCopy(vtkAbstractCellLinks* src)
{
  vtkCellLinks* clinks = static_cast<vtkCellLinks*>(src);
  this->Allocate(clinks->Size, clinks->Extend);
  memcpy(this->Array, clinks->Array, this->Size * sizeof(vtkCellLinks::Link));
  this->MaxId = clinks->MaxId;
}

//------------------------------------------------------------------------------
void vtkCellLinks::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Size: " << this->Size << "\n";
  os << indent << "MaxId: " << this->MaxId << "\n";
  os << indent << "Extend: " << this->Extend << "\n";
}
