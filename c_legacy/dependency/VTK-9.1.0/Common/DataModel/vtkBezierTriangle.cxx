/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkBezierTriangle.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkBezierTriangle.h"
#include "vtkBezierInterpolation.h"

#include "vtkBezierCurve.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkDataSet.h"
#include "vtkDoubleArray.h"
#include "vtkIncrementalPointLocator.h"
#include "vtkLine.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkTriangle.h"
#include "vtkVector.h"

#define ENABLE_CACHING
#define SEVEN_POINT_TRIANGLE

vtkStandardNewMacro(vtkBezierTriangle);
//------------------------------------------------------------------------------
vtkBezierTriangle::vtkBezierTriangle() = default;

//------------------------------------------------------------------------------
vtkBezierTriangle::~vtkBezierTriangle() = default;

void vtkBezierTriangle::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

vtkCell* vtkBezierTriangle::GetEdge(int edgeId)
{
  vtkBezierCurve* result = EdgeCell;
  if (this->GetRationalWeights()->GetNumberOfTuples() > 0)
  {
    const auto set_number_of_ids_and_points = [&](const vtkIdType& npts) -> void {
      result->Points->SetNumberOfPoints(npts);
      result->PointIds->SetNumberOfIds(npts);
      result->GetRationalWeights()->SetNumberOfTuples(npts);
    };
    const auto set_ids_and_points = [&](const vtkIdType& edge_id, const vtkIdType& vol_id) -> void {
      result->Points->SetPoint(edge_id, this->Points->GetPoint(vol_id));
      result->PointIds->SetId(edge_id, this->PointIds->GetId(vol_id));
      result->GetRationalWeights()->SetValue(edge_id, this->GetRationalWeights()->GetValue(vol_id));
    };
    this->SetEdgeIdsAndPoints(edgeId, set_number_of_ids_and_points, set_ids_and_points);
  }
  else
  {
    const auto set_number_of_ids_and_points = [&](const vtkIdType& npts) -> void {
      result->Points->SetNumberOfPoints(npts);
      result->PointIds->SetNumberOfIds(npts);
      result->GetRationalWeights()->Reset();
    };
    const auto set_ids_and_points = [&](const vtkIdType& edge_id, const vtkIdType& vol_id) -> void {
      result->Points->SetPoint(edge_id, this->Points->GetPoint(vol_id));
      result->PointIds->SetId(edge_id, this->PointIds->GetId(vol_id));
    };
    this->SetEdgeIdsAndPoints(edgeId, set_number_of_ids_and_points, set_ids_and_points);
  }

  return result;
}

/**\brief EvaluateLocation Given a point_id. This is required by Bezier because the interior points
 * are non-interpolatory .
 */
void vtkBezierTriangle::EvaluateLocationProjectedNode(
  int& subId, const vtkIdType point_id, double x[3], double* weights)
{
  this->vtkHigherOrderTriangle::SetParametricCoords();
  double pcoords[3];
  this->PointParametricCoordinates->GetPoint(this->PointIds->FindIdLocation(point_id), pcoords);
  this->vtkHigherOrderTriangle::EvaluateLocation(subId, pcoords, x, weights);
}

/**\brief Set the rational weight of the cell, given a vtkDataSet
 */
void vtkBezierTriangle::SetRationalWeightsFromPointData(
  vtkPointData* point_data, const vtkIdType numPts)
{
  vtkDataArray* v = point_data->GetRationalWeights();
  if (v)
  {
    this->GetRationalWeights()->SetNumberOfTuples(numPts);
    for (vtkIdType i = 0; i < numPts; i++)
    {
      this->GetRationalWeights()->SetValue(i, v->GetTuple1(this->PointIds->GetId(i)));
    }
  }
  else
    this->GetRationalWeights()->Reset();
}

//------------------------------------------------------------------------------
void vtkBezierTriangle::InterpolateFunctions(const double pcoords[3], double* weights)
{
  const int dim = 2;
  const int deg = GetOrder();
  const vtkIdType nPoints = this->GetPoints()->GetNumberOfPoints();
  std::vector<double> coeffs(nPoints, 0.0);
  vtkBezierInterpolation::DeCasteljauSimplex(dim, deg, pcoords, &coeffs[0]);
  for (vtkIdType i = 0; i < nPoints; ++i)
  {
    vtkVector3i bv = vtkBezierInterpolation::UnFlattenSimplex(dim, deg, i);
    vtkIdType lbv[3] = { bv[0], bv[1], bv[2] };
    weights[Index(lbv, deg)] = coeffs[i];
  }

  // If the unit cell has rational weigths: weights_i = weights_i * rationalWeights / sum( weights_i
  // * rationalWeights )
  const bool has_rational_weights = RationalWeights->GetNumberOfTuples() > 0;
  if (has_rational_weights)
  {
    double w = 0;
    for (vtkIdType idx = 0; idx < nPoints; ++idx)
    {
      weights[idx] *= RationalWeights->GetTuple1(idx);
      w += weights[idx];
    }
    const double one_over_rational_weight = 1. / w;
    for (vtkIdType idx = 0; idx < nPoints; ++idx)
      weights[idx] *= one_over_rational_weight;
  }
}

//------------------------------------------------------------------------------
void vtkBezierTriangle::InterpolateDerivs(const double pcoords[3], double* derivs)
{
  const int dim = 2;
  const int deg = GetOrder();
  const vtkIdType nPoints = this->GetPoints()->GetNumberOfPoints();
  std::vector<double> coeffs(nPoints, 0.0);
  vtkBezierInterpolation::DeCasteljauSimplexDeriv(dim, deg, pcoords, &coeffs[0]);
  for (vtkIdType i = 0; i < nPoints; ++i)
  {
    vtkVector3i bv = vtkBezierInterpolation::UnFlattenSimplex(dim, deg, i);
    vtkIdType lbv[3] = { bv[0], bv[1], bv[2] };
    for (int j = 0; j < dim; ++j)
    {
      derivs[j * nPoints + Index(lbv, deg)] = coeffs[j * nPoints + i];
    }
  }
}

vtkDoubleArray* vtkBezierTriangle::GetRationalWeights()
{
  return RationalWeights.Get();
}

vtkHigherOrderCurve* vtkBezierTriangle::GetEdgeCell()
{
  return EdgeCell;
}
