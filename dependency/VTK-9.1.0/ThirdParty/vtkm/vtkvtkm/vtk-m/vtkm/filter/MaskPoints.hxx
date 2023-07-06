//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#ifndef vtk_m_filter_MaskPoints_hxx
#define vtk_m_filter_MaskPoints_hxx

namespace vtkm
{
namespace filter
{

//-----------------------------------------------------------------------------
inline VTKM_CONT MaskPoints::MaskPoints()
  : vtkm::filter::FilterDataSet<MaskPoints>()
  , Stride(1)
  , CompactPoints(true)
{
}

//-----------------------------------------------------------------------------
template <typename DerivedPolicy>
inline VTKM_CONT vtkm::cont::DataSet MaskPoints::DoExecute(
  const vtkm::cont::DataSet& input,
  vtkm::filter::PolicyBase<DerivedPolicy> policy)
{
  // extract the input cell set
  const vtkm::cont::DynamicCellSet& cells = input.GetCellSet();

  // run the worklet on the cell set and input field
  vtkm::cont::CellSetSingleType<> outCellSet;
  vtkm::worklet::MaskPoints worklet;

  outCellSet = worklet.Run(vtkm::filter::ApplyPolicyCellSet(cells, policy, *this), this->Stride);

  // create the output dataset
  vtkm::cont::DataSet output;
  output.SetCellSet(outCellSet);
  output.AddCoordinateSystem(input.GetCoordinateSystem(this->GetActiveCoordinateSystemIndex()));

  // compact the unused points in the output dataset
  if (this->CompactPoints)
  {
    this->Compactor.SetCompactPointFields(true);
    this->Compactor.SetMergePoints(false);
    return this->Compactor.Execute(output);
  }
  else
  {
    return output;
  }
}

//-----------------------------------------------------------------------------
template <typename DerivedPolicy>
inline VTKM_CONT bool MaskPoints::MapFieldOntoOutput(vtkm::cont::DataSet& result,
                                                     const vtkm::cont::Field& field,
                                                     vtkm::filter::PolicyBase<DerivedPolicy> policy)
{
  // point data is copied as is because it was not collapsed
  if (field.IsFieldPoint())
  {
    if (this->CompactPoints)
    {
      return this->Compactor.MapFieldOntoOutput(result, field, policy);
    }
    else
    {
      result.AddField(field);
      return true;
    }
  }
  else if (field.IsFieldGlobal())
  {
    result.AddField(field);
    return true;
  }
  else
  {
    // cell data does not apply
    return false;
  }
}
}
}
#endif
