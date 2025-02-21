/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTriangleMeshPointNormals.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTriangleMeshPointNormals.h"

#include "vtkArrayDispatch.h"
#include "vtkCellArray.h"
#include "vtkCellArrayIterator.h"
#include "vtkCellData.h"
#include "vtkDataArrayRange.h"
#include "vtkFloatArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"

vtkStandardNewMacro(vtkTriangleMeshPointNormals);

namespace
{

struct ComputeNormalsDirection
{
  template <typename ArrayT>
  void operator()(ArrayT* pointArray, vtkPolyData* mesh, vtkFloatArray* normalsArray)
  {
    const auto points = vtk::DataArrayTupleRange<3>(pointArray);
    auto normals = vtk::DataArrayTupleRange<3>(normalsArray);

    float a[3], b[3], tn[3];

    auto cellIter = vtk::TakeSmartPointer(mesh->GetPolys()->NewIterator());
    for (cellIter->GoToFirstCell(); !cellIter->IsDoneWithTraversal(); cellIter->GoToNextCell())
    {
      vtkIdType cellSize;
      const vtkIdType* cell;
      cellIter->GetCurrentCell(cellSize, cell);

      // First value in cellArray indicates number of points in cell.
      // We need 3 to compute normals.
      if (cellSize == 3)
      {
        const auto p0 = points[cell[0]];
        const auto p1 = points[cell[1]];
        const auto p2 = points[cell[2]];
        auto n0 = normals[cell[0]];
        auto n1 = normals[cell[1]];
        auto n2 = normals[cell[2]];

        // two vectors
        a[0] = static_cast<float>(p2[0] - p1[0]);
        a[1] = static_cast<float>(p2[1] - p1[1]);
        a[2] = static_cast<float>(p2[2] - p1[2]);
        b[0] = static_cast<float>(p0[0] - p1[0]);
        b[1] = static_cast<float>(p0[1] - p1[1]);
        b[2] = static_cast<float>(p0[2] - p1[2]);

        // cell normals by cross-product
        // (no need to normalize those + it's faster not to)
        tn[0] = (a[1] * b[2] - a[2] * b[1]);
        tn[1] = (a[2] * b[0] - a[0] * b[2]);
        tn[2] = (a[0] * b[1] - a[1] * b[0]);

        // append triangle normals to point normals
        n0[0] += tn[0];
        n0[1] += tn[1];
        n0[2] += tn[2];
        n1[0] += tn[0];
        n1[1] += tn[1];
        n1[2] += tn[2];
        n2[0] += tn[0];
        n2[1] += tn[1];
        n2[2] += tn[2];
      }
      // If degenerate cell
      else if (cellSize < 3)
      {
        vtkGenericWarningMacro("Some cells are degenerate (less than 3 points). "
                               "Use vtkCleanPolyData beforehand to correct this.");
        return;
      }
      // If cell not triangle
      else
      {
        vtkGenericWarningMacro("Some cells have too many points (more than 3 points). "
                               "Use vtkTriangulate to correct this.");
        return;
      }
    }
  }
};

} // end anon namespace

// Generate normals for polygon meshes
int vtkTriangleMeshPointNormals::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // get the info objects
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // get the input and output
  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkDebugMacro(<< "Generating surface normals");

  vtkIdType numPts = input->GetNumberOfPoints(); // nbr of points from input
  if (numPts < 1)
  {
    vtkDebugMacro(<< "No data to generate normals for!");
    return 1;
  }

  if (input->GetVerts()->GetNumberOfCells() != 0 || input->GetLines()->GetNumberOfCells() != 0 ||
    input->GetStrips()->GetNumberOfCells() != 0)
  {
    vtkErrorMacro(
      << "Can not compute normals for a mesh with Verts, Lines or Strips, as it will "
      << "corrupt the number of points used during the normals computation."
      << "Make sure your input PolyData only has triangles (Polys with 3 components)).");
    return 0;
  }

  // Copy structure and cell data
  output->CopyStructure(input);
  output->GetCellData()->PassData(input->GetCellData());

  // If there is nothing to do, pass the point data through
  if (input->GetNumberOfPolys() < 1)
  {
    output->GetPointData()->PassData(input->GetPointData());
    return 1;
  }
  // Else pass everything but normals
  output->GetPointData()->CopyNormalsOff();
  output->GetPointData()->PassData(input->GetPointData());

  // Prepare array for normals
  vtkFloatArray* normals = vtkFloatArray::New();
  normals->SetNumberOfComponents(3);
  normals->SetNumberOfTuples(numPts);
  normals->SetName("Normals");
  normals->FillValue(0.0);
  output->GetPointData()->SetNormals(normals);
  normals->Delete();

  this->UpdateProgress(0.1);

  // Fast-path for float/double points:
  using vtkArrayDispatch::Reals;
  using Dispatcher = vtkArrayDispatch::DispatchByValueType<Reals>;
  ComputeNormalsDirection worker;

  vtkDataArray* points = output->GetPoints()->GetData();
  if (!Dispatcher::Execute(points, worker, output, normals))
  { // fallback for integral point arrays
    worker(points, output, normals);
  }

  this->UpdateProgress(0.5);

  // Normalize point normals
  float l;
  unsigned int i3;
  float* n = normals->GetPointer(0);
  for (vtkIdType i = 0; i < numPts; ++i)
  {
    i3 = i * 3;
    if ((l = sqrt(n[i3] * n[i3] + n[i3 + 1] * n[i3 + 1] + n[i3 + 2] * n[i3 + 2])) != 0.0)
    {
      n[i3] /= l;
      n[i3 + 1] /= l;
      n[i3 + 2] /= l;
    }
  }
  this->UpdateProgress(0.9);

  // Update modified time
  normals->Modified();

  return 1;
}

void vtkTriangleMeshPointNormals::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
