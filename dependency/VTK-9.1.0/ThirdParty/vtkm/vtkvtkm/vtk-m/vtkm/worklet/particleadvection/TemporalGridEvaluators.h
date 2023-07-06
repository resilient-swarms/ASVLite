//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#ifndef vtk_m_worklet_particleadvection_TemporalGridEvaluators_h
#define vtk_m_worklet_particleadvection_TemporalGridEvaluators_h

#include <vtkm/worklet/particleadvection/GridEvaluatorStatus.h>
#include <vtkm/worklet/particleadvection/GridEvaluators.h>

namespace vtkm
{
namespace worklet
{
namespace particleadvection
{

template <typename FieldType>
class ExecutionTemporalGridEvaluator
{
private:
  using GridEvaluator = vtkm::worklet::particleadvection::GridEvaluator<FieldType>;
  using ExecutionGridEvaluator =
    vtkm::worklet::particleadvection::ExecutionGridEvaluator<FieldType>;

public:
  VTKM_CONT
  ExecutionTemporalGridEvaluator() = default;

  VTKM_CONT
  ExecutionTemporalGridEvaluator(const GridEvaluator& evaluatorOne,
                                 const vtkm::FloatDefault timeOne,
                                 const GridEvaluator& evaluatorTwo,
                                 const vtkm::FloatDefault timeTwo,
                                 vtkm::cont::DeviceAdapterId device,
                                 vtkm::cont::Token& token)
    : EvaluatorOne(evaluatorOne.PrepareForExecution(device, token))
    , EvaluatorTwo(evaluatorTwo.PrepareForExecution(device, token))
    , TimeOne(timeOne)
    , TimeTwo(timeTwo)
    , TimeDiff(timeTwo - timeOne)
  {
  }

  template <typename Point>
  VTKM_EXEC bool IsWithinSpatialBoundary(const Point point) const
  {
    return this->EvaluatorOne.IsWithinSpatialBoundary(point) &&
      this->EvaluatorTwo.IsWithinSpatialBoundary(point);
  }

  VTKM_EXEC
  bool IsWithinTemporalBoundary(const vtkm::FloatDefault time) const
  {
    return time >= this->TimeOne && time <= this->TimeTwo;
  }

  VTKM_EXEC
  vtkm::Bounds GetSpatialBoundary() const { return this->EvaluatorTwo.GetSpatialBoundary(); }

  VTKM_EXEC_CONT
  vtkm::FloatDefault GetTemporalBoundary(vtkm::Id direction) const
  {
    return direction > 0 ? this->TimeTwo : this->TimeOne;
  }

  template <typename Point>
  VTKM_EXEC GridEvaluatorStatus Evaluate(const Point& particle,
                                         vtkm::FloatDefault time,
                                         vtkm::VecVariable<Point, 2>& out) const
  {
    // Validate time is in bounds for the current two slices.
    GridEvaluatorStatus status;

    if (!(time >= TimeOne && time <= TimeTwo))
    {
      status.SetFail();
      status.SetTemporalBounds();
      return status;
    }

    vtkm::VecVariable<Point, 2> e1, e2;
    status = this->EvaluatorOne.Evaluate(particle, time, e1);
    if (status.CheckFail())
      return status;
    status = this->EvaluatorTwo.Evaluate(particle, time, e2);
    if (status.CheckFail())
      return status;

    // LERP between the two values of calculated fields to obtain the new value
    vtkm::FloatDefault proportion = (time - this->TimeOne) / this->TimeDiff;
    VTKM_ASSERT(e1.GetNumberOfComponents() != 0 &&
                e1.GetNumberOfComponents() == e2.GetNumberOfComponents());
    out = vtkm::VecVariable<Point, 2>{};
    for (vtkm::IdComponent index = 0; index < e1.GetNumberOfComponents(); ++index)
      out.Append(vtkm::Lerp(e1[index], e2[index], proportion));

    status.SetOk();
    return status;
  }

private:
  ExecutionGridEvaluator EvaluatorOne;
  ExecutionGridEvaluator EvaluatorTwo;
  vtkm::FloatDefault TimeOne;
  vtkm::FloatDefault TimeTwo;
  vtkm::FloatDefault TimeDiff;
};

template <typename FieldType>
class TemporalGridEvaluator : public vtkm::cont::ExecutionObjectBase
{
private:
  using GridEvaluator = vtkm::worklet::particleadvection::GridEvaluator<FieldType>;

public:
  VTKM_CONT TemporalGridEvaluator() = default;

  VTKM_CONT TemporalGridEvaluator(const vtkm::cont::DataSet& ds1,
                                  const vtkm::FloatDefault t1,
                                  const FieldType& field1,
                                  const vtkm::cont::DataSet& ds2,
                                  const vtkm::FloatDefault t2,
                                  const FieldType& field2)
    : EvaluatorOne(GridEvaluator(ds1, field1))
    , EvaluatorTwo(GridEvaluator(ds2, field2))
    , TimeOne(t1)
    , TimeTwo(t2)

  {
  }


  VTKM_CONT TemporalGridEvaluator(GridEvaluator& evaluatorOne,
                                  const vtkm::FloatDefault timeOne,
                                  GridEvaluator& evaluatorTwo,
                                  const vtkm::FloatDefault timeTwo)
    : EvaluatorOne(evaluatorOne)
    , EvaluatorTwo(evaluatorTwo)
    , TimeOne(timeOne)
    , TimeTwo(timeTwo)
  {
  }

  VTKM_CONT TemporalGridEvaluator(const vtkm::cont::CoordinateSystem& coordinatesOne,
                                  const vtkm::cont::DynamicCellSet& cellsetOne,
                                  const FieldType& fieldOne,
                                  const vtkm::FloatDefault timeOne,
                                  const vtkm::cont::CoordinateSystem& coordinatesTwo,
                                  const vtkm::cont::DynamicCellSet& cellsetTwo,
                                  const FieldType& fieldTwo,
                                  const vtkm::FloatDefault timeTwo)
    : EvaluatorOne(GridEvaluator(coordinatesOne, cellsetOne, fieldOne))
    , EvaluatorTwo(GridEvaluator(coordinatesTwo, cellsetTwo, fieldTwo))
    , TimeOne(timeOne)
    , TimeTwo(timeTwo)
  {
  }

  VTKM_CONT ExecutionTemporalGridEvaluator<FieldType> PrepareForExecution(
    vtkm::cont::DeviceAdapterId device,
    vtkm::cont::Token& token) const
  {
    return ExecutionTemporalGridEvaluator<FieldType>(
      this->EvaluatorOne, this->TimeOne, this->EvaluatorTwo, this->TimeTwo, device, token);
  }

private:
  GridEvaluator EvaluatorOne;
  GridEvaluator EvaluatorTwo;
  vtkm::FloatDefault TimeOne;
  vtkm::FloatDefault TimeTwo;
};

} // namespace particleadvection
} // namespace worklet
} // namespace vtkm

#endif
