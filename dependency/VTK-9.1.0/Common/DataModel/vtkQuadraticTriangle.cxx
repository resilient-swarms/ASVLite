/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkQuadraticTriangle.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkQuadraticTriangle.h"

#include "vtkDoubleArray.h"
#include "vtkLine.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPoints.h"
#include "vtkQuadraticEdge.h"
#include "vtkTriangle.h"

vtkStandardNewMacro(vtkQuadraticTriangle);

//------------------------------------------------------------------------------
// Construct the line with two points.
vtkQuadraticTriangle::vtkQuadraticTriangle()
{
  this->Edge = vtkQuadraticEdge::New();
  this->Face = vtkTriangle::New();
  this->Scalars = vtkDoubleArray::New();
  this->Scalars->SetNumberOfTuples(3);

  this->Points->SetNumberOfPoints(6);
  this->PointIds->SetNumberOfIds(6);
  for (int i = 0; i < 6; i++)
  {
    this->Points->SetPoint(i, 0.0, 0.0, 0.0);
    this->PointIds->SetId(i, 0);
  }
}

//------------------------------------------------------------------------------
vtkQuadraticTriangle::~vtkQuadraticTriangle()
{
  this->Edge->Delete();
  this->Face->Delete();
  this->Scalars->Delete();
}

//------------------------------------------------------------------------------
vtkCell* vtkQuadraticTriangle::GetEdge(int edgeId)
{
  edgeId = (edgeId < 0 ? 0 : (edgeId > 2 ? 2 : edgeId));
  int p = (edgeId + 1) % 3;

  // load point id's
  this->Edge->PointIds->SetId(0, this->PointIds->GetId(edgeId));
  this->Edge->PointIds->SetId(1, this->PointIds->GetId(p));
  this->Edge->PointIds->SetId(2, this->PointIds->GetId(edgeId + 3));

  // load coordinates
  this->Edge->Points->SetPoint(0, this->Points->GetPoint(edgeId));
  this->Edge->Points->SetPoint(1, this->Points->GetPoint(p));
  this->Edge->Points->SetPoint(2, this->Points->GetPoint(edgeId + 3));

  return this->Edge;
}

//------------------------------------------------------------------------------
// order picked carefully for parametric coordinate conversion
static int LinearTris[4][3] = {
  { 0, 3, 5 },
  { 3, 1, 4 },
  { 5, 4, 2 },
  { 4, 5, 3 },
};

int vtkQuadraticTriangle::EvaluatePosition(const double* x, double closestPoint[3], int& subId,
  double pcoords[3], double& minDist2, double weights[])
{
  double pc[3], dist2;
  int ignoreId, i, returnStatus = 0, status;
  double tempWeights[3];
  double closest[3];

  // four linear triangles are used
  for (minDist2 = VTK_DOUBLE_MAX, i = 0; i < 4; i++)
  {
    this->Face->Points->SetPoint(0, this->Points->GetPoint(LinearTris[i][0]));
    this->Face->Points->SetPoint(1, this->Points->GetPoint(LinearTris[i][1]));
    this->Face->Points->SetPoint(2, this->Points->GetPoint(LinearTris[i][2]));

    status = this->Face->EvaluatePosition(x, closest, ignoreId, pc, dist2, tempWeights);
    if (status != -1 && dist2 < minDist2)
    {
      returnStatus = status;
      minDist2 = dist2;
      subId = i;
      pcoords[0] = pc[0];
      pcoords[1] = pc[1];
    }
  }

  // adjust parametric coordinates
  if (returnStatus != -1)
  {
    if (subId == 0)
    {
      pcoords[0] /= 2.0;
      pcoords[1] /= 2.0;
    }
    else if (subId == 1)
    {
      pcoords[0] = 0.5 + (pcoords[0] / 2.0);
      pcoords[1] /= 2.0;
    }
    else if (subId == 2)
    {
      pcoords[0] /= 2.0;
      pcoords[1] = 0.5 + (pcoords[1] / 2.0);
    }
    else
    {
      pcoords[0] = 0.5 - pcoords[0] / 2.0;
      pcoords[1] = 0.5 - pcoords[1] / 2.0;
    }
    pcoords[2] = 0.0;
    if (closestPoint != nullptr)
    {
      // Compute both closestPoint and weights
      this->EvaluateLocation(subId, pcoords, closestPoint, weights);
    }
    else
    {
      // Compute weights only
      vtkQuadraticTriangle::InterpolationFunctions(pcoords, weights);
    }
  }

  return returnStatus;
}

//------------------------------------------------------------------------------
void vtkQuadraticTriangle::EvaluateLocation(
  int& vtkNotUsed(subId), const double pcoords[3], double x[3], double* weights)
{
  int i;
  double a0[3], a1[3], a2[3], a3[3], a4[3], a5[3];
  this->Points->GetPoint(0, a0);
  this->Points->GetPoint(1, a1);
  this->Points->GetPoint(2, a2);
  this->Points->GetPoint(3, a3);
  this->Points->GetPoint(4, a4);
  this->Points->GetPoint(5, a5);

  vtkQuadraticTriangle::InterpolationFunctions(pcoords, weights);

  for (i = 0; i < 3; i++)
  {
    x[i] = a0[i] * weights[0] + a1[i] * weights[1] + a2[i] * weights[2] + a3[i] * weights[3] +
      a4[i] * weights[4] + a5[i] * weights[5];
  }
}

//------------------------------------------------------------------------------
int vtkQuadraticTriangle::CellBoundary(int subId, const double pcoords[3], vtkIdList* pts)
{
  return this->Face->CellBoundary(subId, pcoords, pts);
}

//------------------------------------------------------------------------------
void vtkQuadraticTriangle::Contour(double value, vtkDataArray* cellScalars,
  vtkIncrementalPointLocator* locator, vtkCellArray* verts, vtkCellArray* lines,
  vtkCellArray* polys, vtkPointData* inPd, vtkPointData* outPd, vtkCellData* inCd, vtkIdType cellId,
  vtkCellData* outCd)
{
  for (int i = 0; i < 4; i++)
  {
    this->Face->Points->SetPoint(0, this->Points->GetPoint(LinearTris[i][0]));
    this->Face->Points->SetPoint(1, this->Points->GetPoint(LinearTris[i][1]));
    this->Face->Points->SetPoint(2, this->Points->GetPoint(LinearTris[i][2]));

    if (outPd)
    {
      this->Face->PointIds->SetId(0, this->PointIds->GetId(LinearTris[i][0]));
      this->Face->PointIds->SetId(1, this->PointIds->GetId(LinearTris[i][1]));
      this->Face->PointIds->SetId(2, this->PointIds->GetId(LinearTris[i][2]));
    }

    this->Scalars->SetTuple(0, cellScalars->GetTuple(LinearTris[i][0]));
    this->Scalars->SetTuple(1, cellScalars->GetTuple(LinearTris[i][1]));
    this->Scalars->SetTuple(2, cellScalars->GetTuple(LinearTris[i][2]));

    this->Face->Contour(
      value, this->Scalars, locator, verts, lines, polys, inPd, outPd, inCd, cellId, outCd);
  }
}

//------------------------------------------------------------------------------
// Line-line intersection. Intersection has to occur within [0,1] parametric
// coordinates and with specified tolerance.
int vtkQuadraticTriangle::IntersectWithLine(
  const double* p1, const double* p2, double tol, double& t, double* x, double* pcoords, int& subId)
{
  int subTest, i;
  subId = 0;

  for (i = 0; i < 4; i++)
  {
    this->Face->Points->SetPoint(0, this->Points->GetPoint(LinearTris[i][0]));
    this->Face->Points->SetPoint(1, this->Points->GetPoint(LinearTris[i][1]));
    this->Face->Points->SetPoint(2, this->Points->GetPoint(LinearTris[i][2]));

    if (this->Face->IntersectWithLine(p1, p2, tol, t, x, pcoords, subTest))
    {
      return 1;
    }
  }

  return 0;
}

//------------------------------------------------------------------------------
int vtkQuadraticTriangle::Triangulate(int vtkNotUsed(index), vtkIdList* ptIds, vtkPoints* pts)
{
  pts->Reset();
  ptIds->Reset();

  // Create four linear triangles
  for (int i = 0; i < 4; i++)
  {
    ptIds->InsertId(3 * i, this->PointIds->GetId(LinearTris[i][0]));
    pts->InsertPoint(3 * i, this->Points->GetPoint(LinearTris[i][0]));
    ptIds->InsertId(3 * i + 1, this->PointIds->GetId(LinearTris[i][1]));
    pts->InsertPoint(3 * i + 1, this->Points->GetPoint(LinearTris[i][1]));
    ptIds->InsertId(3 * i + 2, this->PointIds->GetId(LinearTris[i][2]));
    pts->InsertPoint(3 * i + 2, this->Points->GetPoint(LinearTris[i][2]));
  }

  return 1;
}

//------------------------------------------------------------------------------
void vtkQuadraticTriangle::Derivatives(
  int vtkNotUsed(subId), const double pcoords[3], const double* values, int dim, double* derivs)
{
  double sum[2], p[3];
  double functionDerivs[12];
  double *J[3], J0[3], J1[3], J2[3];
  double *JI[3], JI0[3], JI1[3], JI2[3];

  vtkQuadraticTriangle::InterpolationDerivs(pcoords, functionDerivs);

  // Compute transposed Jacobian and inverse Jacobian
  J[0] = J0;
  J[1] = J1;
  J[2] = J2;
  JI[0] = JI0;
  JI[1] = JI1;
  JI[2] = JI2;
  for (int k = 0; k < 3; k++)
  {
    J0[k] = J1[k] = 0.0;
  }

  for (int i = 0; i < 6; i++)
  {
    this->Points->GetPoint(i, p);
    for (int j = 0; j < 2; j++)
    {
      for (int k = 0; k < 3; k++)
      {
        J[j][k] += p[k] * functionDerivs[j * 6 + i];
      }
    }
  }

  // Compute third row vector in transposed Jacobian and normalize it, so that Jacobian
  // determinant stays the same.
  vtkMath::Cross(J0, J1, J2);
  if (vtkMath::Normalize(J2) == 0.0 || !vtkMath::InvertMatrix(J, JI, 3)) // degenerate
  {
    for (int j = 0; j < dim; j++)
    {
      for (int i = 0; i < 3; i++)
      {
        derivs[j * dim + i] = 0.0;
      }
    }
    return;
  }

  // Loop over "dim" derivative values. For each set of values, compute derivatives
  // in local system and then transform into modelling system.
  // First compute derivatives in local x'-y' coordinate system
  for (int j = 0; j < dim; j++)
  {
    sum[0] = sum[1] = 0.0;
    for (int i = 0; i < 6; i++) // loop over interp. function derivatives
    {
      sum[0] += functionDerivs[i] * values[dim * i + j];
      sum[1] += functionDerivs[6 + i] * values[dim * i + j];
    }

    // Transform into global system (dot product with global axes)
    derivs[3 * j] = sum[0] * JI[0][0] + sum[1] * JI[0][1];
    derivs[3 * j + 1] = sum[0] * JI[1][0] + sum[1] * JI[1][1];
    derivs[3 * j + 2] = sum[0] * JI[2][0] + sum[1] * JI[2][1];
  }
}

//------------------------------------------------------------------------------
// Clip this quadratic triangle using the scalar value provided. Like
// contouring, except that it cuts the triangle to produce other quads
// and triangles.
void vtkQuadraticTriangle::Clip(double value, vtkDataArray* cellScalars,
  vtkIncrementalPointLocator* locator, vtkCellArray* polys, vtkPointData* inPd, vtkPointData* outPd,
  vtkCellData* inCd, vtkIdType cellId, vtkCellData* outCd, int insideOut)
{
  for (int i = 0; i < 4; i++)
  {
    this->Face->Points->SetPoint(0, this->Points->GetPoint(LinearTris[i][0]));
    this->Face->Points->SetPoint(1, this->Points->GetPoint(LinearTris[i][1]));
    this->Face->Points->SetPoint(2, this->Points->GetPoint(LinearTris[i][2]));

    this->Face->PointIds->SetId(0, this->PointIds->GetId(LinearTris[i][0]));
    this->Face->PointIds->SetId(1, this->PointIds->GetId(LinearTris[i][1]));
    this->Face->PointIds->SetId(2, this->PointIds->GetId(LinearTris[i][2]));

    this->Scalars->SetTuple(0, cellScalars->GetTuple(LinearTris[i][0]));
    this->Scalars->SetTuple(1, cellScalars->GetTuple(LinearTris[i][1]));
    this->Scalars->SetTuple(2, cellScalars->GetTuple(LinearTris[i][2]));

    this->Face->Clip(
      value, this->Scalars, locator, polys, inPd, outPd, inCd, cellId, outCd, insideOut);
  }
}

//------------------------------------------------------------------------------
// Compute maximum parametric distance to cell
double vtkQuadraticTriangle::GetParametricDistance(const double pcoords[3])
{
  int i;
  double pDist, pDistMax = 0.0;
  double pc[3];

  pc[0] = pcoords[0];
  pc[1] = pcoords[1];
  pc[2] = 1.0 - pcoords[0] - pcoords[1];

  for (i = 0; i < 3; i++)
  {
    if (pc[i] < 0.0)
    {
      pDist = -pc[i];
    }
    else if (pc[i] > 1.0)
    {
      pDist = pc[i] - 1.0;
    }
    else // inside the cell in the parametric direction
    {
      pDist = 0.0;
    }
    if (pDist > pDistMax)
    {
      pDistMax = pDist;
    }
  }

  return pDistMax;
}

//------------------------------------------------------------------------------
// Compute interpolation functions. The first three nodes are the triangle
// vertices; the others are mid-edge nodes.
void vtkQuadraticTriangle::InterpolationFunctions(const double pcoords[3], double weights[6])
{
  double r = pcoords[0];
  double s = pcoords[1];
  double t = 1.0 - r - s;

  weights[0] = t * (2.0 * t - 1.0);
  weights[1] = r * (2.0 * r - 1.0);
  weights[2] = s * (2.0 * s - 1.0);
  weights[3] = 4.0 * r * t;
  weights[4] = 4.0 * r * s;
  weights[5] = 4.0 * s * t;
}

//------------------------------------------------------------------------------
// Derivatives in parametric space.
void vtkQuadraticTriangle::InterpolationDerivs(const double pcoords[3], double derivs[12])
{
  double r = pcoords[0];
  double s = pcoords[1];

  // r-derivatives
  derivs[0] = 4.0 * r + 4.0 * s - 3.0;
  derivs[1] = 4.0 * r - 1.0;
  derivs[2] = 0.0;
  derivs[3] = 4.0 - 8.0 * r - 4.0 * s;
  derivs[4] = 4.0 * s;
  derivs[5] = -4.0 * s;

  // s-derivatives
  derivs[6] = 4.0 * r + 4.0 * s - 3.0;
  derivs[7] = 0.0;
  derivs[8] = 4.0 * s - 1.0;
  derivs[9] = -4.0 * r;
  derivs[10] = 4.0 * r;
  derivs[11] = 4.0 - 8.0 * s - 4.0 * r;
}

//------------------------------------------------------------------------------
static double vtkQTriangleCellPCoords[18] = {
  0.0, 0.0, 0.0, //
  1.0, 0.0, 0.0, //
  0.0, 1.0, 0.0, //
  0.5, 0.0, 0.0, //
  0.5, 0.5, 0.0, //
  0.0, 0.5, 0.0  //
};
double* vtkQuadraticTriangle::GetParametricCoords()
{
  return vtkQTriangleCellPCoords;
}

//------------------------------------------------------------------------------
void vtkQuadraticTriangle::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Edge:\n";
  this->Edge->PrintSelf(os, indent.GetNextIndent());
  os << indent << "Edge:\n";
  this->Edge->PrintSelf(os, indent.GetNextIndent());
  os << indent << "Scalars:\n";
  this->Scalars->PrintSelf(os, indent.GetNextIndent());
}
