//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#include <vtkm/worklet/DispatcherMapTopology.h>
#include <vtkm/worklet/WorkletMapTopology.h>

#include <vtkm/cont/ArrayHandleXGCCoordinates.h>
#include <vtkm/cont/CellSetExtrude.h>
#include <vtkm/cont/testing/Testing.h>

#include <vtkm/filter/PointAverage.h>
#include <vtkm/filter/PointAverage.hxx>

namespace
{
std::vector<float> points_rz = { 1.72485139f, 0.020562f,   1.73493571f,
                                 0.02052826f, 1.73478011f, 0.02299051f }; //really a vec<float,2>
std::vector<int> topology = { 0, 2, 1 };
std::vector<int> nextNode = { 0, 1, 2 };


struct CopyTopo : public vtkm::worklet::WorkletVisitCellsWithPoints
{
  typedef void ControlSignature(CellSetIn, FieldOutCell);
  typedef _2 ExecutionSignature(CellShape, PointIndices);

  template <typename T>
  VTKM_EXEC T&& operator()(vtkm::CellShapeTagWedge, T&& t) const
  {
    return std::forward<T>(t);
  }
};

struct CopyReverseCellCount : public vtkm::worklet::WorkletVisitPointsWithCells
{
  typedef void ControlSignature(CellSetIn, FieldOutPoint);
  typedef _2 ExecutionSignature(CellShape, CellCount, CellIndices);

  template <typename T>
  VTKM_EXEC vtkm::Int32 operator()(vtkm::CellShapeTagVertex shape,
                                   vtkm::IdComponent count,
                                   T&& t) const
  {
    if (shape.Id == vtkm::CELL_SHAPE_VERTEX)
    {
      bool valid = true;
      for (vtkm::IdComponent i = 0; i < count; ++i)
      {
        valid = valid && t[i] > 0;
      }
      return (valid && count == t.GetNumberOfComponents()) ? count : -1;
    }
    return -1;
  }
};

template <typename T, typename S>
void verify_topo(vtkm::cont::ArrayHandle<vtkm::Vec<T, 6>, S> const& handle, vtkm::Id expectedLen)
{
  auto portal = handle.ReadPortal();
  VTKM_TEST_ASSERT(portal.GetNumberOfValues() == expectedLen, "topology portal size is incorrect");

  for (vtkm::Id i = 0; i < expectedLen - 1; ++i)
  {
    auto v = portal.Get(i);
    vtkm::Vec<vtkm::Id, 6> e;
    e[0] = (static_cast<vtkm::Id>(topology[0]) + (i * static_cast<vtkm::Id>(topology.size())));
    e[1] = (static_cast<vtkm::Id>(topology[1]) + (i * static_cast<vtkm::Id>(topology.size())));
    e[2] = (static_cast<vtkm::Id>(topology[2]) + (i * static_cast<vtkm::Id>(topology.size())));
    e[3] =
      (static_cast<vtkm::Id>(topology[0]) + ((i + 1) * static_cast<vtkm::Id>(topology.size())));
    e[4] =
      (static_cast<vtkm::Id>(topology[1]) + ((i + 1) * static_cast<vtkm::Id>(topology.size())));
    e[5] =
      (static_cast<vtkm::Id>(topology[2]) + ((i + 1) * static_cast<vtkm::Id>(topology.size())));
    std::cout << "v, e: " << v << ", " << e << "\n";
    VTKM_TEST_ASSERT(test_equal(v, e), "incorrect conversion of topology to Cartesian space");
  }

  auto v = portal.Get(expectedLen - 1);
  vtkm::Vec<vtkm::Id, 6> e;
  e[0] = (static_cast<vtkm::Id>(topology[0]) +
          ((expectedLen - 1) * static_cast<vtkm::Id>(topology.size())));
  e[1] = (static_cast<vtkm::Id>(topology[1]) +
          ((expectedLen - 1) * static_cast<vtkm::Id>(topology.size())));
  e[2] = (static_cast<vtkm::Id>(topology[2]) +
          ((expectedLen - 1) * static_cast<vtkm::Id>(topology.size())));
  e[3] = (static_cast<vtkm::Id>(topology[0]));
  e[4] = (static_cast<vtkm::Id>(topology[1]));
  e[5] = (static_cast<vtkm::Id>(topology[2]));
  VTKM_TEST_ASSERT(test_equal(v, e), "incorrect conversion of topology to Cartesian space");
}

template <typename T, typename S>
void verify_reverse_topo(vtkm::cont::ArrayHandle<T, S> const& handle, vtkm::Id expectedLen)
{
  auto portal = handle.ReadPortal();
  VTKM_TEST_ASSERT(portal.GetNumberOfValues() == expectedLen, "topology portal size is incorrect");
  for (vtkm::Id i = 0; i < expectedLen - 1; ++i)
  {
    auto v = portal.Get(i);
    VTKM_TEST_ASSERT((v <= 2), "incorrect conversion to reverse topology");
  }
}
int TestCellSetExtrude()
{
  const std::size_t numPlanes = 8;

  auto coords = vtkm::cont::make_ArrayHandleXGCCoordinates(points_rz, numPlanes, false);
  auto cells = vtkm::cont::make_CellSetExtrude(topology, coords, nextNode);
  VTKM_TEST_ASSERT(cells.GetNumberOfPoints() == coords.GetNumberOfValues(),
                   "number of points don't match between cells and coordinates");

  // Verify the topology by copying it into another array
  {
    vtkm::cont::ArrayHandle<vtkm::Vec<int, 6>> output;
    vtkm::worklet::DispatcherMapTopology<CopyTopo> dispatcher;
    dispatcher.Invoke(cells, output);
    verify_topo(output, 8);
  }


  // Verify the reverse topology by copying the number of cells each point is
  // used by it into another array
  {
    vtkm::cont::ArrayHandle<int> output;
    vtkm::worklet::DispatcherMapTopology<CopyReverseCellCount> dispatcher;
    dispatcher.Invoke(cells, output);
    verify_reverse_topo(output, 24);
  }

  return 0;
}
}

int UnitTestCellSetExtrude(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(TestCellSetExtrude, argc, argv);
}
