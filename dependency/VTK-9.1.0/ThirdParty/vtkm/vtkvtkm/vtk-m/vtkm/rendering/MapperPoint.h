//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_rendering_MapperPoint_h
#define vtk_m_rendering_MapperPoint_h

#include <vtkm/cont/ColorTable.h>
#include <vtkm/rendering/Camera.h>
#include <vtkm/rendering/Mapper.h>

#include <memory>

namespace vtkm
{
namespace rendering
{

/**
 * \brief MapperPonts renders points from a cell set.
 *        This mapper can natively create points from
 *        vertex cell shapes as well as use the points
 *        defined by a coordinate system.
 *
 */
class VTKM_RENDERING_EXPORT MapperPoint : public Mapper
{
public:
  MapperPoint();

  ~MapperPoint();

  void SetCanvas(vtkm::rendering::Canvas* canvas) override;
  virtual vtkm::rendering::Canvas* GetCanvas() const override;

  /**
   * \brief render points based on cell shape point
   *
   */
  void UseCells();
  /**
   * \brief render points using the nodes of the mesh.
   *        This is the default.
   */
  void UseNodes();

  /**
   * \brief render points using a variable radius based on
   *        the scalar field.
   *        The default is false.
   */
  void UseVariableRadius(bool useVariableRadius);

  /**
   * \brief Set a base raidus for all points. If a
   *        radius is never specified the default heuristic
   *        is used.
   */
  void SetRadius(const vtkm::Float32& radius);
  /**
   * \brief When using a variable raidus for all points, the
   *        radius delta controls how much larger and smaller
   *        radii become based on the scalar field. If the delta
   *        is 0 all points will have the same radius. If the delta
   *        is 0.5 then the max/min scalar values would have a radii
   *        of base +/- base * 0.5.
   */
  void SetRadiusDelta(const vtkm::Float32& delta);

  void RenderCells(const vtkm::cont::DynamicCellSet& cellset,
                   const vtkm::cont::CoordinateSystem& coords,
                   const vtkm::cont::Field& scalarField,
                   const vtkm::cont::ColorTable& colorTable,
                   const vtkm::rendering::Camera& camera,
                   const vtkm::Range& scalarRange) override;

  void SetCompositeBackground(bool on);
  vtkm::rendering::Mapper* NewCopy() const override;

private:
  struct InternalsType;
  std::shared_ptr<InternalsType> Internals;

  struct RenderFunctor;
};
}
} //namespace vtkm::rendering

#endif //vtk_m_rendering_MapperPoint_h
