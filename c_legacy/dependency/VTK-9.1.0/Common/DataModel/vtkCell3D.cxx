/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCell3D.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// Hide VTK_DEPRECATED_IN_9_0_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkCell3D.h"

#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkDataArrayRange.h"
#include "vtkDoubleArray.h"
#include "vtkMath.h"
#include "vtkMathUtilities.h"
#include "vtkOrderedTriangulator.h"
#include "vtkPointData.h"
#include "vtkPointLocator.h"
#include "vtkPoints.h"
#include "vtkPolygon.h"
#include "vtkTetra.h"

#include <vector>

vtkCell3D::vtkCell3D()
{
  this->Triangulator = nullptr;
  this->MergeTolerance = 0.01;
  this->ClipTetra = nullptr;
  this->ClipScalars = nullptr;
}

vtkCell3D::~vtkCell3D()
{
  if (this->Triangulator)
  {
    this->Triangulator->Delete();
    this->Triangulator = nullptr;
  }
  if (this->ClipTetra)
  {
    this->ClipTetra->Delete();
    this->ClipTetra = nullptr;
    this->ClipScalars->Delete();
    this->ClipScalars = nullptr;
  }
}

bool vtkCell3D::IsInsideOut()
{
  // Strategy:
  // - Compute the centroid of the cell.
  // - Accumulate a signed projected distance on the normal between the faces and the centroid.
  // - Check the sign to see if the cell is inside out or not.
  double centroid[3], point[3], normal[3];
  if (!this->GetCentroid(centroid))
  {
    return false;
  }
  double signedDistanceToCentroid = 0.0;
  for (vtkIdType faceId = 0; faceId < this->GetNumberOfFaces(); ++faceId)
  {
    const vtkIdType* pointIds;
    vtkIdType faceSize = this->GetFacePoints(faceId, pointIds);
    if (faceSize)
    {
      this->Points->GetPoint(pointIds[0], point);
      vtkPolygon::ComputeNormal(this->Points, faceSize, pointIds, normal);
      signedDistanceToCentroid +=
        vtkPolygon::ComputeArea(this->Points, faceSize, pointIds, normal) *
        (vtkMath::Dot(normal, centroid) - vtkMath::Dot(normal, point));
    }
  }
  return signedDistanceToCentroid > 0.0;
}

//----------------------------------------------------------------------------
int vtkCell3D::Inflate(double dist)
{
  // Strategy:
  // For each point, store in a buffer its inflated position by moving each
  // incident face their normal direction by a distance of dist. This new
  // position is done by solving a linear system of equation (intersection of 3
  // planes). It is enough to pick 3 non-coplanar incident faces. Thus, only 3
  // are picked (we discard faces coplanar to any already included face)
  // Then, each point is updated using this buffer.

  std::vector<double> buf(3 * this->Points->GetNumberOfPoints(), 0.0);
  double* position = buf.data();
  dist *= this->IsInsideOut() ? -1.0 : 1.0;
  auto pointRange = vtk::DataArrayTupleRange<3>(this->Points->GetData());
  for (vtkIdType pointId = 0; pointId < this->GetNumberOfPoints(); ++pointId, position += 3)
  {
    double normalBase[3][3] = { { 0.0 } };
    int normalId = 0;
    const vtkIdType* incidentFaceIds;
    vtkIdType numberOfIncidentFaces = this->GetPointToIncidentFaces(pointId, incidentFaceIds);
    for (vtkIdType id = 0; id < numberOfIncidentFaces && normalId < 3; ++id)
    {
      const vtkIdType* incidentFacePointIds;
      vtkIdType incidentFaceSize = this->GetFacePoints(incidentFaceIds[id], incidentFacePointIds);
      vtkPolygon::ComputeNormal(
        this->Points, incidentFaceSize, incidentFacePointIds, normalBase[normalId]);

      if (normalId == 0 ||
        (normalId == 1 &&
          (!vtkMathUtilities::NearlyEqual(
            std::fabs(vtkMath::Dot(normalBase[0], normalBase[1])), 1.0))) ||
        (normalId == 2 &&
          !vtkMathUtilities::NearlyEqual(
            std::fabs(vtkMath::Dot(normalBase[0], normalBase[2])), 1.0) &&
          !vtkMathUtilities::NearlyEqual(
            std::fabs(vtkMath::Dot(normalBase[1], normalBase[2])), 1.0)))
      {
        ++normalId;
      }
    }
    if (normalId != 3)
    {
      continue;
    }
    auto point = pointRange[pointId];
    double d[3] = { vtkMath::Dot(normalBase[0], point) + dist,
      vtkMath::Dot(normalBase[1], point) + dist, vtkMath::Dot(normalBase[2], point) + dist };
    vtkMath::LinearSolve3x3(normalBase, d, position);
  }

  using TupleRef = decltype(pointRange)::TupleReferenceType;
  position = buf.data();
  for (TupleRef point : pointRange)
  {
    point[0] = position[0];
    point[1] = position[1];
    point[2] = position[2];
    position += 3;
  }
  return 1;
}

//----------------------------------------------------------------------------
void vtkCell3D::Contour(double value, vtkDataArray* cellScalars,
  vtkIncrementalPointLocator* locator, vtkCellArray* verts, vtkCellArray* lines,
  vtkCellArray* polys, vtkPointData* inPd, vtkPointData* outPd, vtkCellData* inCd, vtkIdType cellId,
  vtkCellData* outCd)
{
  int numPts = this->GetNumberOfPoints();
  int numEdges = this->GetNumberOfEdges();
  const vtkIdType* tets;
  int v1, v2;
  int i, j;
  int type;
  vtkIdType id, ptId;
  vtkIdType internalId[VTK_CELL_SIZE];
  double s1, s2, x[3], t, p1[3], p2[3], deltaScalar;

  // Create a triangulator if necessary.
  if (!this->Triangulator)
  {
    this->Triangulator = vtkOrderedTriangulator::New();
    this->Triangulator->PreSortedOff();
    this->Triangulator->UseTemplatesOn();
    this->ClipTetra = vtkTetra::New();
    this->ClipScalars = vtkDoubleArray::New();
    this->ClipScalars->SetNumberOfTuples(4);
  }

  // If here, the ordered triangulator is going to be used so the triangulation
  // has to be initialized.
  this->Triangulator->InitTriangulation(0.0, 1.0, 0.0, 1.0, 0.0, 1.0, (numPts + numEdges));

  // Cells with fixed topology are triangulated with templates.
  double *p, *pPtr = this->GetParametricCoords();
  if (this->IsPrimaryCell())
  {
    // Some cell types support templates for interior clipping. Templates
    // are a heck of a lot faster.
    type = 0; // inside
    for (p = pPtr, i = 0; i < numPts; i++, p += 3)
    {
      ptId = this->PointIds->GetId(i);
      this->Points->GetPoint(i, x);
      this->Triangulator->InsertPoint(ptId, x, p, type);
    } // for all cell points of fixed topology

    this->Triangulator->TemplateTriangulate(this->GetCellType(), numPts, numEdges);

    // Otherwise we have produced tetrahedra and now contour these using
    // the faster vtkTetra::Contour() method.
    for (this->Triangulator->InitTetraTraversal();
         this->Triangulator->GetNextTetra(0, this->ClipTetra, cellScalars, this->ClipScalars);)
    {
      this->ClipTetra->Contour(
        value, this->ClipScalars, locator, verts, lines, polys, inPd, outPd, inCd, cellId, outCd);
    }
    return;
  } // if we are clipping fixed topology

  // If here we're left with a non-fixed topology cell (e.g. convex point set).
  // Inject cell points into triangulation. Recall that the PreSortedOff()
  // flag was set which means that the triangulator will order the points
  // according to point id.
  for (p = pPtr, i = 0; i < numPts; i++, p += 3)
  {
    ptId = this->PointIds->GetId(i);

    // Currently all points are injected because of the possibility
    // of intersection point merging.
    s1 = cellScalars->GetComponent(i, 0);
    type = 0; // inside

    // Below is the old code since 2001. Will may take a look
    // at this at some point and see if there is a place to
    // improve it.
    //
    // if ( (s1 >= value) || (s1 < value) )
    // {
    //   type = 0; //inside
    // }
    // else
    // {
    //   type = 4; //outside, its type might change later (nearby intersection)
    // }

    this->Points->GetPoint(i, x);
    if (locator->InsertUniquePoint(x, id))
    {
      outPd->CopyData(inPd, ptId, id);
    }
    internalId[i] = this->Triangulator->InsertPoint(id, x, p, type);
  } // for all points

  // For each edge intersection point, insert into triangulation. Edge
  // intersections come from contouring value. Have to be careful of
  // intersections near existing points (causes bad Delaunay behavior).
  // Intersections near existing points are collapsed to existing point.
  double pc[3], *pc1, *pc2;
  for (int edgeNum = 0; edgeNum < numEdges; edgeNum++)
  {
    this->GetEdgePoints(edgeNum, tets);

    // Calculate a preferred interpolation direction.
    // Has to be done in same direction to ensure coincident points are
    // merged (different interpolation direction causes perturbations).
    s1 = cellScalars->GetComponent(tets[0], 0);
    s2 = cellScalars->GetComponent(tets[1], 0);

    if ((s1 <= value && s2 >= value) || (s1 >= value && s2 <= value))
    {
      deltaScalar = s2 - s1;

      if (deltaScalar > 0)
      {
        v1 = tets[0];
        v2 = tets[1];
      }
      else
      {
        v1 = tets[1];
        v2 = tets[0];
        deltaScalar = -deltaScalar;
      }

      // linear interpolation
      t = (deltaScalar == 0.0 ? 0.0 : (value - cellScalars->GetComponent(v1, 0)) / deltaScalar);

      if (t < this->MergeTolerance)
      {
        this->Triangulator->UpdatePointType(internalId[v1], 2);
        continue;
      }
      else if (t > (1.0 - this->MergeTolerance))
      {
        this->Triangulator->UpdatePointType(internalId[v2], 2);
        continue;
      }

      this->Points->GetPoint(v1, p1);
      this->Points->GetPoint(v2, p2);
      pc1 = pPtr + 3 * v1;
      pc2 = pPtr + 3 * v2;

      for (j = 0; j < 3; j++)
      {
        x[j] = p1[j] + t * (p2[j] - p1[j]);
        pc[j] = pc1[j] + t * (pc2[j] - pc1[j]);
      }

      // Incorporate point into output and interpolate edge data as necessary
      if (locator->InsertUniquePoint(x, ptId))
      {
        outPd->InterpolateEdge(inPd, ptId, this->PointIds->GetId(v1), this->PointIds->GetId(v2), t);
      }

      // Insert intersection point into Delaunay triangulation
      this->Triangulator->InsertPoint(ptId, x, pc, 2);

    } // if edge intersects value
  }   // for all edges

  // triangulate the points
  this->Triangulator->Triangulate();

  // Add the triangulation to the mesh
  this->Triangulator->AddTetras(0, polys);
}

void vtkCell3D::Clip(double value, vtkDataArray* cellScalars, vtkIncrementalPointLocator* locator,
  vtkCellArray* tets, vtkPointData* inPD, vtkPointData* outPD, vtkCellData* inCD, vtkIdType cellId,
  vtkCellData* outCD, int insideOut)
{
  vtkCell3D* cell3D = static_cast<vtkCell3D*>(this); // has to be in this method
  int numPts = this->GetNumberOfPoints();
  int numEdges = this->GetNumberOfEdges();
  const vtkIdType* verts;
  int v1, v2;
  int i, j;
  int type;
  vtkIdType id, ptId;
  vtkIdType internalId[VTK_CELL_SIZE];
  double s1, s2, x[3], t, p1[3], p2[3], deltaScalar;
  int allInside = 1, allOutside = 1;

  // Create a triangulator if necessary.
  if (!this->Triangulator)
  {
    this->Triangulator = vtkOrderedTriangulator::New();
    this->Triangulator->PreSortedOff();
    this->Triangulator->UseTemplatesOn();
    this->ClipTetra = vtkTetra::New();
    this->ClipScalars = vtkDoubleArray::New();
    this->ClipScalars->SetNumberOfTuples(4);
  }

  // Make sure it's worth continuing by treating the interior and exterior
  // cases as special cases.
  for (i = 0; i < numPts; i++)
  {
    s1 = cellScalars->GetComponent(i, 0);
    if ((s1 >= value && !insideOut) || (s1 < value && insideOut))
    {
      allOutside = 0;
    }
    else
    {
      allInside = 0;
    }
  }

  if (allOutside)
  {
    return;
  }

  // If here, the ordered triangulator is going to be used so the triangulation
  // has to be initialized.
  this->Triangulator->InitTriangulation(0.0, 1.0, 0.0, 1.0, 0.0, 1.0, (numPts + numEdges));

  // Cells with fixed topology are triangulated with templates.
  double *p, *pPtr = this->GetParametricCoords();
  if (this->IsPrimaryCell())
  {
    // Some cell types support templates for interior clipping. Templates
    // are a heck of a lot faster.
    type = 0; // inside
    for (p = pPtr, i = 0; i < numPts; i++, p += 3)
    {
      ptId = this->PointIds->GetId(i);
      this->Points->GetPoint(i, x);
      if (locator->InsertUniquePoint(x, id))
      {
        outPD->CopyData(inPD, ptId, id);
      }
      this->Triangulator->InsertPoint(id, x, p, type);
    } // for all cell points of fixed topology

    this->Triangulator->TemplateTriangulate(this->GetCellType(), numPts, numEdges);
    // If the cell is interior we are done.
    if (allInside)
    {
      vtkIdType numTetras = tets->GetNumberOfCells();
      this->Triangulator->AddTetras(0, tets);
      vtkIdType numAddedTetras = tets->GetNumberOfCells() - numTetras;
      for (j = 0; j < numAddedTetras; j++)
      {
        outCD->CopyData(inCD, cellId, numTetras + j);
      }
    }
    // Otherwise we have produced tetrahedra and now clip these using
    // the faster vtkTetra::Clip() method.
    else
    {
      for (this->Triangulator->InitTetraTraversal();
           this->Triangulator->GetNextTetra(0, this->ClipTetra, cellScalars, this->ClipScalars);)
      {
        // VERY IMPORTANT: Notice that the outPD is used twice. This is because the
        // tetra has been defined in terms of point ids that are defined in the
        // output (because of the templates).
        this->ClipTetra->Clip(
          value, this->ClipScalars, locator, tets, outPD, outPD, inCD, cellId, outCD, insideOut);
      }
    } // if boundary cell
    return;
  } // if we are clipping fixed topology

  // If here we're left with a non-fixed topology cell (e.g. convex point set).
  // Inject cell points into triangulation. Recall that the PreSortedOff()
  // flag was set which means that the triangulator will order the points
  // according to point id.
  for (p = pPtr, i = 0; i < numPts; i++, p += 3)
  {
    ptId = this->PointIds->GetId(i);

    // Currently all points are injected because of the possibility
    // of intersection point merging.
    s1 = cellScalars->GetComponent(i, 0);
    if ((s1 >= value && !insideOut) || (s1 < value && insideOut))
    {
      type = 0; // inside
    }
    else
    {
      type = 4; // outside, its type might change later (nearby intersection)
    }

    this->Points->GetPoint(i, x);
    if (locator->InsertUniquePoint(x, id))
    {
      outPD->CopyData(inPD, ptId, id);
    }
    internalId[i] = this->Triangulator->InsertPoint(id, x, p, type);
  } // for all points

  // For each edge intersection point, insert into triangulation. Edge
  // intersections come from clipping value. Have to be careful of
  // intersections near existing points (causes bad Delaunay behavior).
  // Intersections near existing points are collapsed to existing point.
  double pc[3], *pc1, *pc2;
  for (int edgeNum = 0; edgeNum < numEdges; edgeNum++)
  {
    cell3D->GetEdgePoints(edgeNum, verts);

    // Calculate a preferred interpolation direction.
    // Has to be done in same direction to ensure coincident points are
    // merged (different interpolation direction causes perturbations).
    s1 = cellScalars->GetComponent(verts[0], 0);
    s2 = cellScalars->GetComponent(verts[1], 0);

    if ((s1 <= value && s2 >= value) || (s1 >= value && s2 <= value))
    {
      deltaScalar = s2 - s1;

      if (deltaScalar > 0)
      {
        v1 = verts[0];
        v2 = verts[1];
      }
      else
      {
        v1 = verts[1];
        v2 = verts[0];
        deltaScalar = -deltaScalar;
      }

      // linear interpolation
      t = (deltaScalar == 0.0 ? 0.0 : (value - cellScalars->GetComponent(v1, 0)) / deltaScalar);

      if (t < this->MergeTolerance)
      {
        this->Triangulator->UpdatePointType(internalId[v1], 2);
        continue;
      }
      else if (t > (1.0 - this->MergeTolerance))
      {
        this->Triangulator->UpdatePointType(internalId[v2], 2);
        continue;
      }

      this->Points->GetPoint(v1, p1);
      this->Points->GetPoint(v2, p2);
      pc1 = pPtr + 3 * v1;
      pc2 = pPtr + 3 * v2;

      for (j = 0; j < 3; j++)
      {
        x[j] = p1[j] + t * (p2[j] - p1[j]);
        pc[j] = pc1[j] + t * (pc2[j] - pc1[j]);
      }

      // Incorporate point into output and interpolate edge data as necessary
      if (locator->InsertUniquePoint(x, ptId))
      {
        outPD->InterpolateEdge(inPD, ptId, this->PointIds->GetId(v1), this->PointIds->GetId(v2), t);
      }

      // Insert intersection point into Delaunay triangulation
      this->Triangulator->InsertPoint(ptId, x, pc, 2);

    } // if edge intersects value
  }   // for all edges

  // triangulate the points
  this->Triangulator->Triangulate();

  // Add the triangulation to the mesh
  this->Triangulator->AddTetras(0, tets);
}

void vtkCell3D::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Merge Tolerance: " << this->MergeTolerance << "\n";
}
