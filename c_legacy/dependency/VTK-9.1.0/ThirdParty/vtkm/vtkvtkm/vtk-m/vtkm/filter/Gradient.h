//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#ifndef vtk_m_filter_Gradient_h
#define vtk_m_filter_Gradient_h

#include <vtkm/filter/vtkm_filter_gradient_export.h>

#include <vtkm/filter/FilterField.h>

#include <vtkm/cont/ArrayHandleSOA.h>

namespace vtkm
{
namespace filter
{

/// \brief A general filter for gradient estimation.
/// Estimates the gradient of a point field in a data set. The created gradient array
/// can be determined at either each point location or at the center of each cell.
///
/// The default for the filter is output as cell centered gradients.
/// To enable point based gradient computation enable \c SetComputePointGradient
///
/// Note: If no explicit name for the output field is provided the filter will
/// default to "Gradients"
class VTKM_FILTER_GRADIENT_EXPORT Gradient : public vtkm::filter::FilterField<Gradient>
{
public:
  using SupportedTypes = vtkm::List<vtkm::Float32, vtkm::Float64, vtkm::Vec3f_32, vtkm::Vec3f_64>;

  /// When this flag is on (default is off), the gradient filter will provide a
  /// point based gradients, which are significantly more costly since for each
  /// point we need to compute the gradient of each cell that uses it.
  void SetComputePointGradient(bool enable) { ComputePointGradient = enable; }
  bool GetComputePointGradient() const { return ComputePointGradient; }

  /// Add divergence field to the output data.  The name of the array
  /// will be Divergence and will be a cell field unless \c ComputePointGradient
  /// is enabled.  The input array must have 3 components in order to
  /// compute this. The default is off.
  void SetComputeDivergence(bool enable) { ComputeDivergence = enable; }
  bool GetComputeDivergence() const { return ComputeDivergence; }

  /// Add voriticity/curl field to the output data.  The name of the array
  /// will be Vorticity and will be a cell field unless \c ComputePointGradient
  /// is enabled.  The input array must have 3 components in order to
  /// compute this. The default is off.
  void SetComputeVorticity(bool enable) { ComputeVorticity = enable; }
  bool GetComputeVorticity() const { return ComputeVorticity; }

  /// Add Q-criterion field to the output data.  The name of the array
  /// will be QCriterion and will be a cell field unless \c ComputePointGradient
  /// is enabled.  The input array must have 3 components in order to
  /// compute this. The default is off.
  void SetComputeQCriterion(bool enable) { ComputeQCriterion = enable; }
  bool GetComputeQCriterion() const { return ComputeQCriterion; }

  /// Add gradient field to the output data.  The name of the array
  /// will be Gradients and will be a cell field unless \c ComputePointGradient
  /// is enabled. It is useful to turn this off when you are only interested
  /// in the results of Divergence, Vorticity, or QCriterion. The default is on.
  void SetComputeGradient(bool enable) { StoreGradient = enable; }
  bool GetComputeGradient() const { return StoreGradient; }

  /// Make the vector gradient output format be in FORTRAN Column-major order.
  /// This is only used when the input field is a vector field ( 3 components ).
  /// Enabling  column-major is important if integrating with other projects
  /// such as VTK.
  /// Default: Row Order
  void SetColumnMajorOrdering() { RowOrdering = false; }

  /// Make the vector gradient output format be in C Row-major order.
  /// This is only used when the input field is a vector field ( 3 components ).
  /// Default: Row Order
  void SetRowMajorOrdering() { RowOrdering = true; }

  void SetDivergenceName(const std::string& name) { this->DivergenceName = name; }
  const std::string& GetDivergenceName() const { return this->DivergenceName; }

  void SetVorticityName(const std::string& name) { this->VorticityName = name; }
  const std::string& GetVorticityName() const { return this->VorticityName; }

  void SetQCriterionName(const std::string& name) { this->QCriterionName = name; }
  const std::string& GetQCriterionName() const { return this->QCriterionName; }

  template <typename T, typename StorageType, typename DerivedPolicy>
  vtkm::cont::DataSet DoExecute(const vtkm::cont::DataSet& input,
                                const vtkm::cont::ArrayHandle<T, StorageType>& field,
                                const vtkm::filter::FieldMetadata& fieldMeta,
                                vtkm::filter::PolicyBase<DerivedPolicy> policy);

private:
  bool ComputePointGradient = false;
  bool ComputeDivergence = false;
  bool ComputeVorticity = false;
  bool ComputeQCriterion = false;
  bool StoreGradient = true;
  bool RowOrdering = true;

  std::string DivergenceName = "Divergence";
  std::string GradientsName = "Gradients";
  std::string QCriterionName = "QCriterion";
  std::string VorticityName = "Vorticity";
};

#ifndef vtkm_filter_Gradient_cxx

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Float32>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

#ifndef VTKM_NO_DEPRECATED_VIRTUAL
VTKM_DEPRECATED_SUPPRESS_BEGIN
extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Float32, vtkm::cont::StorageTagVirtual>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);
VTKM_DEPRECATED_SUPPRESS_END
#endif

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Float64>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

#ifndef VTKM_NO_DEPRECATED_VIRTUAL
VTKM_DEPRECATED_SUPPRESS_BEGIN
extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Float64, vtkm::cont::StorageTagVirtual>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);
VTKM_DEPRECATED_SUPPRESS_END
#endif

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f_32>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f_32, vtkm::cont::StorageTagSOA>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

#ifndef VTKM_NO_DEPRECATED_VIRTUAL
VTKM_DEPRECATED_SUPPRESS_BEGIN
extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f_32, vtkm::cont::StorageTagVirtual>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);
VTKM_DEPRECATED_SUPPRESS_END
#endif

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f_64>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f_64, vtkm::cont::StorageTagSOA>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

#ifndef VTKM_NO_DEPRECATED_VIRTUAL
VTKM_DEPRECATED_SUPPRESS_BEGIN
extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f_64, vtkm::cont::StorageTagVirtual>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);
VTKM_DEPRECATED_SUPPRESS_END
#endif

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec3f, vtkm::cont::StorageTagUniformPoints>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<
    vtkm::Vec3f_32,
    vtkm::cont::StorageTagCartesianProduct<vtkm::cont::StorageTagBasic,
                                           vtkm::cont::StorageTagBasic,
                                           vtkm::cont::StorageTagBasic>>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<
    vtkm::Vec3f_64,
    vtkm::cont::StorageTagCartesianProduct<vtkm::cont::StorageTagBasic,
                                           vtkm::cont::StorageTagBasic,
                                           vtkm::cont::StorageTagBasic>>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

#ifdef VTKM_ADD_XGC_DEFAULT_TYPES
extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec<float, 3>, vtkm::cont::StorageTagXGCCoordinates>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);

extern template VTKM_FILTER_GRADIENT_TEMPLATE_EXPORT vtkm::cont::DataSet Gradient::DoExecute(
  const vtkm::cont::DataSet&,
  const vtkm::cont::ArrayHandle<vtkm::Vec<double, 3>, vtkm::cont::StorageTagXGCCoordinates>&,
  const vtkm::filter::FieldMetadata&,
  vtkm::filter::PolicyBase<vtkm::filter::PolicyDefault>);
#endif

#endif //vtkm_filter_Gradient_cxx
}
} // namespace vtkm::filter

#endif // vtk_m_filter_Gradient_h
