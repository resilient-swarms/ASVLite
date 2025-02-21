//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_cont_CellLocatorChooser_h
#define vtk_m_cont_CellLocatorChooser_h

#include <vtkm/cont/CastAndCall.h>
#include <vtkm/cont/CellLocatorRectilinearGrid.h>
#include <vtkm/cont/CellLocatorTwoLevel.h>
#include <vtkm/cont/CellLocatorUniformGrid.h>
#include <vtkm/cont/CellSetStructured.h>
#include <vtkm/cont/DataSet.h>

namespace vtkm
{
namespace cont
{

namespace detail
{

template <typename CellSetType, typename CoordinateSystemArrayType>
struct CellLocatorChooserImpl
{
  using type = vtkm::cont::CellLocatorTwoLevel;
};

using UniformArray = vtkm::cont::ArrayHandleUniformPointCoordinates;

template <>
struct CellLocatorChooserImpl<vtkm::cont::CellSetStructured<3>, UniformArray>
{
  using type = vtkm::cont::CellLocatorUniformGrid;
};

using RectilinearArray =
  vtkm::cont::ArrayHandleCartesianProduct<vtkm::cont::ArrayHandle<vtkm::FloatDefault>,
                                          vtkm::cont::ArrayHandle<vtkm::FloatDefault>,
                                          vtkm::cont::ArrayHandle<vtkm::FloatDefault>>;

template <>
struct CellLocatorChooserImpl<vtkm::cont::CellSetStructured<3>, RectilinearArray>
{
  using type = vtkm::cont::CellLocatorRectilinearGrid;
};

} // namespace detail

/// \brief A template to select an appropriate CellLocator based on CellSet type.
///
/// Given a concrete type for a `CellSet` subclass and a type of `ArrayHandle` for the
/// coordinate system, `CellLocatorChooser` picks an appropriate `CellLocator` for that
/// type of grid. It is a convenient class to use when you can resolve your templates
/// to discover the type of data set being used for location.
///
template <typename CellSetType, typename CoordinateSystemArrayType>
using CellLocatorChooser =
  typename detail::CellLocatorChooserImpl<CellSetType, CoordinateSystemArrayType>::type;

namespace detail
{

struct CastAndCallCellLocatorChooserFunctor
{
  template <typename CellLocatorType, typename Functor, typename... Args>
  void CallFunctorWithLocator(const vtkm::cont::DynamicCellSet& cellSet,
                              const vtkm::cont::CoordinateSystem& coordinateSystem,
                              Functor&& functor,
                              Args&&... args) const
  {
    CellLocatorType locator;
    locator.SetCellSet(cellSet);
    locator.SetCoordinates(coordinateSystem);

    functor(locator, std::forward<Args>(args)...);
  }

  template <typename CellSetType, typename Functor, typename... Args>
  void operator()(const CellSetType& cellSet,
                  const vtkm::cont::CoordinateSystem& coordinateSystem,
                  Functor&& functor,
                  Args&&... args) const
  {
    this->CallFunctorWithLocator<vtkm::cont::CellLocatorTwoLevel>(
      cellSet, coordinateSystem, std::forward<Functor>(functor), std::forward<Args>(args)...);
  }

  template <typename Functor, typename... Args>
  void operator()(const vtkm::cont::CellSetStructured<3>& cellSet,
                  const vtkm::cont::CoordinateSystem& coordinateSystem,
                  Functor&& functor,
                  Args&&... args) const
  {
    auto coordArray = coordinateSystem.GetData();
    if (coordArray.IsType<detail::UniformArray>())
    {
      this->CallFunctorWithLocator<vtkm::cont::CellLocatorUniformGrid>(
        cellSet, coordinateSystem, std::forward<Functor>(functor), std::forward<Args>(args)...);
    }
    else if (coordArray.IsType<detail::RectilinearArray>())
    {
      this->CallFunctorWithLocator<vtkm::cont::CellLocatorRectilinearGrid>(
        cellSet, coordinateSystem, std::forward<Functor>(functor), std::forward<Args>(args)...);
    }
    else
    {
      this->CallFunctorWithLocator<vtkm::cont::CellLocatorTwoLevel>(
        cellSet, coordinateSystem, std::forward<Functor>(functor), std::forward<Args>(args)...);
    }
  }
};

} // namespace detail

/// \brief Calls a functor with the appropriate type of `CellLocator`.
///
/// Given a cell set and a coordinate system of unknown types, calls a functor with an appropriate
/// CellLocator of the given type. The CellLocator is populated with the provided cell set and
/// coordinate system.
///
/// Any additional args are passed to the functor.
///
template <typename CellSetList, typename Functor, typename... Args>
VTKM_CONT void CastAndCallCellLocatorChooser(
  const vtkm::cont::DynamicCellSetBase<CellSetList>& cellSet,
  const vtkm::cont::CoordinateSystem& coordinateSystem,
  Functor&& functor,
  Args&&... args)
{
  vtkm::cont::CastAndCall(cellSet,
                          detail::CastAndCallCellLocatorChooserFunctor{},
                          coordinateSystem,
                          std::forward<Functor>(functor),
                          std::forward<Args>(args)...);
}

/// \brief Calls a functor with the appropriate type of `CellLocator`.
///
/// Given a `DataSet`, calls a functor with an appropriate CellLocator of the given type.
/// The CellLocator is populated with the provided cell set and coordinate system.
///
/// Any additional args are passed to the functor.
///
template <typename Functor, typename... Args>
VTKM_CONT void CastAndCallCellLocatorChooser(const vtkm::cont::DataSet& dataSet,
                                             Functor&& functor,
                                             Args&&... args)
{
  CastAndCallCellLocatorChooser(dataSet.GetCellSet(),
                                dataSet.GetCoordinateSystem(),
                                std::forward<Functor>(functor),
                                std::forward<Args>(args)...);
}

}
} // namespace vtkm::cont

#endif //vtk_m_cont_CellLocatorChooser_h
