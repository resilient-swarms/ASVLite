//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_cont_FieldRangeGlobalCompute_h
#define vtk_m_cont_FieldRangeGlobalCompute_h

#include <vtkm/cont/FieldRangeCompute.h>

namespace vtkm
{
namespace cont
{

namespace detail
{

VTKM_CONT_EXPORT VTKM_CONT vtkm::cont::ArrayHandle<vtkm::Range> MergeRangesGlobal(
  const vtkm::cont::ArrayHandle<vtkm::Range>& range);

} // namespace detail

/// \brief utility functions to compute global ranges for dataset fields.
///
/// These functions compute global ranges for fields in a single DataSet or a
/// PartitionedDataSet.
/// In non-distributed environments, this is exactly same as `FieldRangeCompute`. In
/// distributed environments, however, the range is computed locally on each rank
/// and then a reduce-all collective is performed to reduces the ranges on all ranks.

//{@
/// Returns the range for a field from a dataset. If the field is not present, an empty
/// ArrayHandle will be returned.
VTKM_CONT_EXPORT
VTKM_CONT
vtkm::cont::ArrayHandle<vtkm::Range> FieldRangeGlobalCompute(
  const vtkm::cont::DataSet& dataset,
  const std::string& name,
  vtkm::cont::Field::Association assoc = vtkm::cont::Field::Association::ANY);

template <typename TypeList>
VTKM_DEPRECATED(1.6, "FieldRangeGlobalCompute no longer supports TypeList.")
VTKM_CONT vtkm::cont::ArrayHandle<vtkm::Range> FieldRangeGlobalCompute(
  const vtkm::cont::DataSet& dataset,
  const std::string& name,
  vtkm::cont::Field::Association assoc,
  TypeList)
{
  VTKM_IS_LIST(TypeList);
  VTKM_DEPRECATED_SUPPRESS_BEGIN
  auto lrange = vtkm::cont::FieldRangeCompute(dataset, name, assoc, TypeList());
  VTKM_DEPRECATED_SUPPRESS_END
  return vtkm::cont::detail::MergeRangesGlobal(lrange);
}

//@}

//{@
/// Returns the range for a field from a PartitionedDataSet. If the field is
/// not present on any of the partitions, an empty ArrayHandle will be
/// returned. If the field is present on some partitions, but not all, those
/// partitions without the field are skipped.
///
/// The returned array handle will have as many values as the maximum number of
/// components for the selected field across all partitions.
VTKM_CONT_EXPORT
VTKM_CONT
vtkm::cont::ArrayHandle<vtkm::Range> FieldRangeGlobalCompute(
  const vtkm::cont::PartitionedDataSet& pds,
  const std::string& name,
  vtkm::cont::Field::Association assoc = vtkm::cont::Field::Association::ANY);

template <typename TypeList>
VTKM_DEPRECATED(1.6, "FieldRangeGlobalCompute no longer supports TypeList.")
VTKM_CONT vtkm::cont::ArrayHandle<vtkm::Range> FieldRangeGlobalCompute(
  const vtkm::cont::PartitionedDataSet& pds,
  const std::string& name,
  vtkm::cont::Field::Association assoc,
  TypeList)
{
  VTKM_IS_LIST(TypeList);
  VTKM_DEPRECATED_SUPPRESS_BEGIN
  auto lrange = vtkm::cont::FieldRangeCompute(pds, name, assoc, TypeList());
  VTKM_DEPRECATED_SUPPRESS_END
  return vtkm::cont::detail::MergeRangesGlobal(lrange);
}
//@}
}
} // namespace vtkm::cont

#endif
