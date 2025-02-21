//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#ifndef vtk_m_filter_Pathline_h
#define vtk_m_filter_Pathline_h

#include <vtkm/filter/FilterDataSetWithField.h>
#include <vtkm/worklet/ParticleAdvection.h>
#include <vtkm/worklet/particleadvection/GridEvaluators.h>
#include <vtkm/worklet/particleadvection/Stepper.h>

namespace vtkm
{
namespace filter
{
/// \brief generate streamlines from a vector field.

/// Takes as input a vector field and seed locations and generates the
/// paths taken by the seeds through the vector field.
class Pathline : public vtkm::filter::FilterDataSetWithField<Pathline>
{
public:
  using SupportedTypes = vtkm::TypeListFieldVec3;

  VTKM_CONT
  Pathline();

  VTKM_CONT
  void SetPreviousTime(vtkm::FloatDefault t) { this->PreviousTime = t; }
  VTKM_CONT
  void SetNextTime(vtkm::FloatDefault t) { this->NextTime = t; }

  VTKM_CONT
  void SetNextDataSet(const vtkm::cont::DataSet& ds)
  {
    this->NextDataSet = vtkm::cont::PartitionedDataSet(ds);
  }

  VTKM_CONT
  void SetNextDataSet(const vtkm::cont::PartitionedDataSet& pds) { this->NextDataSet = pds; }


  VTKM_CONT
  void SetStepSize(vtkm::FloatDefault s) { this->StepSize = s; }

  VTKM_CONT
  void SetNumberOfSteps(vtkm::Id n) { this->NumberOfSteps = n; }

  VTKM_CONT
  void SetSeeds(vtkm::cont::ArrayHandle<vtkm::Particle>& seeds);

  VTKM_CONT
  bool GetUseThreadedAlgorithm() { return this->UseThreadedAlgorithm; }

  VTKM_CONT
  void SetUseThreadedAlgorithm(bool val) { this->UseThreadedAlgorithm = val; }

  template <typename DerivedPolicy>
  vtkm::cont::PartitionedDataSet PrepareForExecution(
    const vtkm::cont::PartitionedDataSet& input,
    const vtkm::filter::PolicyBase<DerivedPolicy>& policy);

  template <typename DerivedPolicy>
  VTKM_CONT bool MapFieldOntoOutput(vtkm::cont::DataSet& result,
                                    const vtkm::cont::Field& field,
                                    vtkm::filter::PolicyBase<DerivedPolicy> policy);

private:
  vtkm::FloatDefault StepSize;
  vtkm::FloatDefault PreviousTime;
  vtkm::FloatDefault NextTime;
  vtkm::cont::PartitionedDataSet NextDataSet;
  vtkm::Id NumberOfSteps;
  vtkm::cont::ArrayHandle<vtkm::Particle> Seeds;
  bool UseThreadedAlgorithm;
};
}
} // namespace vtkm::filter

#include <vtkm/filter/Pathline.hxx>

#endif // vtk_m_filter_Pathline_h
