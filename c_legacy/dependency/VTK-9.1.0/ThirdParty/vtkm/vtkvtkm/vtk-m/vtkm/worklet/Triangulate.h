//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtkm_m_worklet_Triangulate_h
#define vtkm_m_worklet_Triangulate_h

#include <vtkm/worklet/triangulate/TriangulateExplicit.h>
#include <vtkm/worklet/triangulate/TriangulateStructured.h>

namespace vtkm
{
namespace worklet
{

class Triangulate
{
public:
  //
  // Distribute multiple copies of cell data depending on cells create from original
  //
  struct DistributeCellData : public vtkm::worklet::WorkletMapField
  {
    using ControlSignature = void(FieldIn inIndices, FieldOut outIndices);

    using ScatterType = vtkm::worklet::ScatterCounting;

    template <typename CountArrayType>
    VTKM_CONT static ScatterType MakeScatter(const CountArrayType& countArray)
    {
      return ScatterType(countArray);
    }

    template <typename T>
    VTKM_EXEC void operator()(T inputIndex, T& outputIndex) const
    {
      outputIndex = inputIndex;
    }
  };

  Triangulate()
    : OutCellScatter(vtkm::cont::ArrayHandle<vtkm::IdComponent>{})
  {
  }

  // Triangulate explicit data set, save number of triangulated cells per input
  template <typename CellSetType>
  vtkm::cont::CellSetSingleType<> Run(const CellSetType& cellSet)
  {
    TriangulateExplicit worklet;
    vtkm::cont::ArrayHandle<vtkm::IdComponent> outCellsPerCell;
    vtkm::cont::CellSetSingleType<> result = worklet.Run(cellSet, outCellsPerCell);
    this->OutCellScatter = DistributeCellData::MakeScatter(outCellsPerCell);
    return result;
  }

  // Triangulate structured data set, save number of triangulated cells per input
  vtkm::cont::CellSetSingleType<> Run(const vtkm::cont::CellSetStructured<2>& cellSet)
  {
    TriangulateStructured worklet;
    vtkm::cont::ArrayHandle<vtkm::IdComponent> outCellsPerCell;
    vtkm::cont::CellSetSingleType<> result = worklet.Run(cellSet, outCellsPerCell);
    this->OutCellScatter = DistributeCellData::MakeScatter(outCellsPerCell);
    return result;
  }

  vtkm::cont::CellSetSingleType<> Run(const vtkm::cont::CellSetStructured<3>&)
  {
    throw vtkm::cont::ErrorBadType("CellSetStructured<3> can't be triangulated");
  }

  // Using the saved input to output cells, expand cell data
  template <typename ValueType, typename StorageType>
  vtkm::cont::ArrayHandle<ValueType> ProcessCellField(
    const vtkm::cont::ArrayHandle<ValueType, StorageType>& input) const
  {
    vtkm::cont::ArrayHandle<ValueType> output;

    vtkm::worklet::DispatcherMapField<DistributeCellData> dispatcher(this->OutCellScatter);
    dispatcher.Invoke(input, output);

    return output;
  }

  DistributeCellData::ScatterType GetOutCellScatter() const { return this->OutCellScatter; }

private:
  DistributeCellData::ScatterType OutCellScatter;
};
}
} // namespace vtkm::worklet

#endif // vtkm_m_worklet_Triangulate_h
