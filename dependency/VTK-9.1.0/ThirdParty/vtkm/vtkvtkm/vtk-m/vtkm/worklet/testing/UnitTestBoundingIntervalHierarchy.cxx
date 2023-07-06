//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/cont/Algorithm.h>
#include <vtkm/cont/ArrayHandleConcatenate.h>
#include <vtkm/cont/CellLocatorBoundingIntervalHierarchy.h>
#include <vtkm/cont/DataSetBuilderUniform.h>
#include <vtkm/cont/Invoker.h>
#include <vtkm/cont/Timer.h>
#include <vtkm/cont/testing/Testing.h>
#include <vtkm/exec/CellInterpolate.h>
#include <vtkm/exec/ParametricCoordinates.h>
#include <vtkm/io/VTKDataSetReader.h>

namespace
{
struct CellCentroidCalculator : public vtkm::worklet::WorkletVisitCellsWithPoints
{
  using ControlSignature = void(CellSetIn, FieldInPoint, FieldOut);
  using ExecutionSignature = void(_1, PointCount, _2, _3);

  template <typename CellShape, typename InputPointField>
  VTKM_EXEC void operator()(CellShape shape,
                            vtkm::IdComponent numPoints,
                            const InputPointField& inputPointField,
                            typename InputPointField::ComponentType& outputField) const
  {
    vtkm::Vec3f parametricCenter;
    vtkm::exec::ParametricCoordinatesCenter(numPoints, shape, parametricCenter);
    vtkm::exec::CellInterpolate(inputPointField, parametricCenter, shape, outputField);
  }
}; // struct CellCentroidCalculator

struct BoundingIntervalHierarchyTester : public vtkm::worklet::WorkletMapField
{
  typedef void ControlSignature(FieldIn, ExecObject, FieldIn, FieldOut);
  typedef _4 ExecutionSignature(_1, _2, _3);

  template <typename Point, typename BoundingIntervalHierarchyExecObject>
  VTKM_EXEC vtkm::IdComponent operator()(const Point& point,
                                         const BoundingIntervalHierarchyExecObject& bih,
                                         const vtkm::Id expectedId) const
  {
    vtkm::Vec3f parametric;
    vtkm::Id cellId;
    bih.FindCell(point, cellId, parametric);
    return (1 - static_cast<vtkm::IdComponent>(expectedId == cellId));
  }
}; // struct BoundingIntervalHierarchyTester

vtkm::cont::DataSet ConstructDataSet(vtkm::Id size)
{
  return vtkm::cont::DataSetBuilderUniform().Create(vtkm::Id3(size, size, size));
}

void TestBoundingIntervalHierarchy(vtkm::cont::DataSet dataSet, vtkm::IdComponent numPlanes)
{

  vtkm::cont::DynamicCellSet cellSet = dataSet.GetCellSet();
  auto vertices = dataSet.GetCoordinateSystem().GetDataAsMultiplexer();

  vtkm::cont::CellLocatorBoundingIntervalHierarchy bih =
    vtkm::cont::CellLocatorBoundingIntervalHierarchy(numPlanes, 5);
  bih.SetCellSet(cellSet);
  bih.SetCoordinates(dataSet.GetCoordinateSystem());
  bih.Update();

  vtkm::cont::ArrayHandle<vtkm::Vec3f> centroids;
  vtkm::worklet::DispatcherMapTopology<CellCentroidCalculator>().Invoke(
    cellSet, vertices, centroids);


  vtkm::cont::ArrayHandleCounting<vtkm::Id> expectedCellIds(0, 1, cellSet.GetNumberOfCells());
  vtkm::cont::ArrayHandle<vtkm::IdComponent> results;

  vtkm::worklet::DispatcherMapField<BoundingIntervalHierarchyTester>().Invoke(
    centroids, bih, expectedCellIds, results);

  vtkm::Id numDiffs = vtkm::cont::Algorithm::Reduce(results, 0, vtkm::Add());
  VTKM_TEST_ASSERT(numDiffs == 0, "Calculated cell Ids not the same as expected cell Ids");
}

void RunTest()
{
//If this test is run on a machine that already has heavy
//cpu usage it will fail, so we limit the number of threads
//to avoid the test timing out
#ifdef VTKM_ENABLE_OPENMP
  omp_set_num_threads(std::min(4, omp_get_max_threads()));
#endif

  TestBoundingIntervalHierarchy(ConstructDataSet(8), 3);
  TestBoundingIntervalHierarchy(ConstructDataSet(8), 4);
  TestBoundingIntervalHierarchy(ConstructDataSet(8), 6);
  TestBoundingIntervalHierarchy(ConstructDataSet(8), 9);
}

} // anonymous namespace

int UnitTestBoundingIntervalHierarchy(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(RunTest, argc, argv);
}
