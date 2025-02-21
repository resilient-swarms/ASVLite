/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLinearSubdivisionFilter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkLinearSubdivisionFilter.h"

#include "vtkCellArray.h"
#include "vtkEdgeTable.h"
#include "vtkIdList.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"

vtkStandardNewMacro(vtkLinearSubdivisionFilter);

void vtkLinearSubdivisionFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

int vtkLinearSubdivisionFilter::GenerateSubdivisionPoints(
  vtkPolyData* inputDS, vtkIntArray* edgeData, vtkPoints* outputPts, vtkPointData* outputPD)
{
  const vtkIdType* pts = nullptr;
  int edgeId;
  vtkIdType npts, cellId, newId;
  vtkIdType p1, p2;
  vtkCellArray* inputPolys = inputDS->GetPolys();
  vtkSmartPointer<vtkIdList> cellIds = vtkSmartPointer<vtkIdList>::New();
  vtkSmartPointer<vtkIdList> pointIds = vtkSmartPointer<vtkIdList>::New();
  vtkSmartPointer<vtkEdgeTable> edgeTable = vtkSmartPointer<vtkEdgeTable>::New();
  vtkPoints* inputPts = inputDS->GetPoints();
  vtkPointData* inputPD = inputDS->GetPointData();
  static double weights[2] = { .5, .5 };

  // Create an edge table to keep track of which edges we've processed
  edgeTable->InitEdgeInsertion(inputDS->GetNumberOfPoints());

  pointIds->SetNumberOfIds(2);

  double total = inputPolys->GetNumberOfCells();
  double curr = 0;

  // Generate new points for subdivisions surface
  for (cellId = 0, inputPolys->InitTraversal(); inputPolys->GetNextCell(npts, pts); cellId++)
  {
    p1 = pts[2];
    p2 = pts[0];

    for (edgeId = 0; edgeId < 3; edgeId++)
    {
      outputPD->CopyData(inputPD, p1, p1);
      outputPD->CopyData(inputPD, p2, p2);

      // Do we need to create a point on this edge?
      if (edgeTable->IsEdge(p1, p2) == -1)
      {
        edgeTable->InsertEdge(p1, p2);

        inputDS->GetCellEdgeNeighbors(-1, p1, p2, cellIds);
        if (cellIds->GetNumberOfIds() > 2)
        {
          vtkErrorMacro("Dataset is non-manifold and cannot be subdivided.");
          return 0;
        }
        // Compute Position andnew PointData using the same subdivision scheme
        pointIds->SetId(0, p1);
        pointIds->SetId(1, p2);
        newId = this->InterpolatePosition(inputPts, outputPts, pointIds, weights);
        outputPD->InterpolatePoint(inputPD, newId, pointIds, weights);
      }
      else // we have already created a point on this edge. find it
      {
        newId = this->FindEdge(inputDS, cellId, p1, p2, edgeData, cellIds);
      }
      edgeData->InsertComponent(cellId, edgeId, newId);
      p1 = p2;
      if (edgeId < 2)
      {
        p2 = pts[edgeId + 1];
      }
    } // each edge
    this->UpdateProgress(curr / total);
    curr += 1;
  } // each cell

  return 1;
}
