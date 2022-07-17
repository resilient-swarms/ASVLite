//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#include <vtkm/rendering/raytracing/ShapeIntersector.h>
#include <vtkm/worklet/DispatcherMapField.h>

namespace vtkm
{
namespace rendering
{
namespace raytracing
{

namespace
{
class IntersectionPointMap : public vtkm::worklet::WorkletMapField
{
public:
  typedef void ControlSignature(FieldIn,
                                FieldIn,
                                FieldIn,
                                FieldIn,
                                FieldInOut,
                                FieldInOut,
                                FieldInOut,
                                FieldInOut);
  typedef void ExecutionSignature(_1, _2, _3, _4, _5, _6, _7, _8);
  template <typename Precision>
  VTKM_EXEC inline void operator()(const vtkm::Id& hitIndex,
                                   const Precision& distance,
                                   const vtkm::Vec<Precision, 3>& rayDir,
                                   const vtkm::Vec<Precision, 3>& rayOrigin,
                                   Precision& intersectionX,
                                   Precision& intersectionY,
                                   Precision& intersectionZ,
                                   Precision& maxDistance) const
  {
    if (hitIndex < 0)
      return;

    intersectionX = rayOrigin[0] + rayDir[0] * distance;
    intersectionY = rayOrigin[1] + rayDir[1] * distance;
    intersectionZ = rayOrigin[2] + rayDir[2] * distance;
    maxDistance = distance;
  }
}; //class IntersectionPoint
} // anon namespace

ShapeIntersector::ShapeIntersector() {}
ShapeIntersector::~ShapeIntersector(){};

void ShapeIntersector::IntersectionPoint(Ray<vtkm::Float32>& rays)
{
  this->IntersectionPointImp(rays);
}

void ShapeIntersector::IntersectionPoint(Ray<vtkm::Float64>& rays)
{
  this->IntersectionPointImp(rays);
}

template <typename Precision>
void ShapeIntersector::IntersectionPointImp(Ray<Precision>& rays)
{
  rays.EnableIntersectionData();
  // Find the intersection point from hit distance
  // and set the new max distance
  vtkm::worklet::DispatcherMapField<IntersectionPointMap>(IntersectionPointMap())
    .Invoke(rays.HitIdx,
            rays.Distance,
            rays.Dir,
            rays.Origin,
            rays.IntersectionX,
            rays.IntersectionY,
            rays.IntersectionZ,
            rays.MaxDistance);
}

vtkm::Bounds ShapeIntersector::GetShapeBounds() const
{
  return ShapeBounds;
}

void ShapeIntersector::SetAABBs(AABBs& aabbs)
{
  this->BVH.SetData(aabbs);
  this->BVH.Construct();
  this->ShapeBounds = this->BVH.TotalBounds;
}
}
}
} //namespace vtkm::rendering::raytracing
