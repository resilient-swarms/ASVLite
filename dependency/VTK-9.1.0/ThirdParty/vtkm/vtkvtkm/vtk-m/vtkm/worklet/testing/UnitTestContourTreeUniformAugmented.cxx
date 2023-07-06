//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
// Copyright (c) 2018, The Regents of the University of California, through
// Lawrence Berkeley National Laboratory (subject to receipt of any required approvals
// from the U.S. Dept. of Energy).  All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// (1) Redistributions of source code must retain the above copyright notice, this
//     list of conditions and the following disclaimer.
//
// (2) Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
// (3) Neither the name of the University of California, Lawrence Berkeley National
//     Laboratory, U.S. Dept. of Energy nor the names of its contributors may be
//     used to endorse or promote products derived from this software without
//     specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//
//=============================================================================
//
//  This code is an extension of the algorithm presented in the paper:
//  Parallel Peak Pruning for Scalable SMP Contour Tree Computation.
//  Hamish Carr, Gunther Weber, Christopher Sewell, and James Ahrens.
//  Proceedings of the IEEE Symposium on Large Data Analysis and Visualization
//  (LDAV), October 2016, Baltimore, Maryland.
//
//  The PPP2 algorithm and software were jointly developed by
//  Hamish Carr (University of Leeds), Gunther H. Weber (LBNL), and
//  Oliver Ruebel (LBNL)
//==============================================================================

#include <vtkm/worklet/ContourTreeUniformAugmented.h>
#include <vtkm/worklet/contourtree_augmented/ContourTree.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

#include <vtkm/cont/testing/MakeTestDataSet.h>
#include <vtkm/cont/testing/Testing.h>

#include <vtkm/worklet/contourtree_augmented/PrintVectors.h>
#include <vtkm/worklet/contourtree_augmented/ProcessContourTree.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

#include <typeinfo>
#include <utility>
#include <vector>
#include <vtkm/Types.h>

namespace
{

using vtkm::cont::testing::MakeTestDataSet;


class TestContourTreeUniform
{
private:
  /// Helper function used to compae two IdArrayType ArrayHandles
  void AssertIdArrayHandles(vtkm::worklet::contourtree_augmented::IdArrayType& result,
                            vtkm::worklet::contourtree_augmented::IdArrayType& expected,
                            std::string arrayName) const
  {
    TestEqualResult testResult = test_equal_ArrayHandles(result, expected);
    if (!testResult)
    {
      std::cout << arrayName << " sizes; result=" << result.GetNumberOfValues()
                << " expected=" << expected.GetNumberOfValues() << std::endl;
      vtkm::worklet::contourtree_augmented::PrintIndices(arrayName + " result", result);
      vtkm::worklet::contourtree_augmented::PrintIndices(arrayName + " expected", expected);
    }
    VTKM_TEST_ASSERT(testResult, "Wrong result for " + arrayName);
  }

  ///
  ///  Helper struct to store all arrays with the expected results for comparison during
  ///  the computation of the contour tree
  ///
  struct ExpectedStepResults
  {
  public:
    ExpectedStepResults(
      vtkm::worklet::contourtree_augmented::IdArrayType& expectedSortOrder,
      vtkm::worklet::contourtree_augmented::IdArrayType& expectedSortIndices,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPeaksJoin,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPitsJoin,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPeaksBuildRegularChainsJoin,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPitsBuildRegularChainsJoin,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPeaksSplit,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPitsSplit,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPeaksBuildRegularChainsSplit,
      vtkm::worklet::contourtree_augmented::IdArrayType& meshExtremaPitsBuildRegularChainsSplit,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitGlobalIndex,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitFirstEdge,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitOutdegree,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitHyperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitActiveVertices,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitEdgeNear,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitEdgeFar,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphJoinTreeInitActiveEdges,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitGlobalIndex,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitFirstEdge,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitOutdegree,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitHyperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitActiveVertices,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitEdgeNear,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitEdgeFar,
      vtkm::worklet::contourtree_augmented::IdArrayType& activeGraphSplitTreeInitActiveEdges,
      vtkm::Id makeJoinTreeNumIterations,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeArcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeSuperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeSupernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeSuperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeHyperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeHypernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeHyperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeJoinTreeFirstSuperchild,
      vtkm::Id makeSplitTreeNumIterations,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeArcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeSuperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeSupernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeSuperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeHyperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeHypernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeHyperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeSplitTreeFirstSuperchild,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeNodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeArcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeSuperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeSupernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeSuperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeAugmentnodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeAugmentarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeHyperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeWhenTransferred,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeHypernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeContourTreeHyperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureNodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureArcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureSuperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureSupernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureSuperarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureAugmentnodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureAugmentarcs,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureHyperparents,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureWhenTransferred,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureHypernodes,
      vtkm::worklet::contourtree_augmented::IdArrayType& makeRegularStructureHyperarcs)
      : SortOrder(expectedSortOrder)
      , SortIndices(expectedSortIndices)
      , MeshExtremaPeaksJoin(meshExtremaPeaksJoin)
      , MeshExtremaPitsJoin(meshExtremaPitsJoin)
      , MeshExtremaPeaksBuildRegularChainsJoin(meshExtremaPeaksBuildRegularChainsJoin)
      , MeshExtremaPitsBuildRegularChainsJoin(meshExtremaPitsBuildRegularChainsJoin)
      , MeshExtremaPeaksSplit(meshExtremaPeaksSplit)
      , MeshExtremaPitsSplit(meshExtremaPitsSplit)
      , MeshExtremaPeaksBuildRegularChainsSplit(meshExtremaPeaksBuildRegularChainsSplit)
      , MeshExtremaPitsBuildRegularChainsSplit(meshExtremaPitsBuildRegularChainsSplit)
      , ActiveGraphJoinTreeInitGlobalIndex(activeGraphJoinTreeInitGlobalIndex)
      , ActiveGraphJoinTreeInitFirstEdge(activeGraphJoinTreeInitFirstEdge)
      , ActiveGraphJoinTreeInitOutdegree(activeGraphJoinTreeInitOutdegree)
      , ActiveGraphJoinTreeInitHyperarcs(activeGraphJoinTreeInitHyperarcs)
      , ActiveGraphJoinTreeInitActiveVertices(activeGraphJoinTreeInitActiveVertices)
      , ActiveGraphJoinTreeInitEdgeNear(activeGraphJoinTreeInitEdgeNear)
      , ActiveGraphJoinTreeInitEdgeFar(activeGraphJoinTreeInitEdgeFar)
      , ActiveGraphJoinTreeInitActiveEdges(activeGraphJoinTreeInitActiveEdges)
      , ActiveGraphSplitTreeInitGlobalIndex(activeGraphSplitTreeInitGlobalIndex)
      , ActiveGraphSplitTreeInitFirstEdge(activeGraphSplitTreeInitFirstEdge)
      , ActiveGraphSplitTreeInitOutdegree(activeGraphSplitTreeInitOutdegree)
      , ActiveGraphSplitTreeInitHyperarcs(activeGraphSplitTreeInitHyperarcs)
      , ActiveGraphSplitTreeInitActiveVertices(activeGraphSplitTreeInitActiveVertices)
      , ActiveGraphSplitTreeInitEdgeNear(activeGraphSplitTreeInitEdgeNear)
      , ActiveGraphSplitTreeInitEdgeFar(activeGraphSplitTreeInitEdgeFar)
      , ActiveGraphSplitTreeInitActiveEdges(activeGraphSplitTreeInitActiveEdges)
      , MakeJoinTreeNumIterations(makeJoinTreeNumIterations)
      , MakeJoinTreeArcs(makeJoinTreeArcs)
      , MakeJoinTreeSuperparents(makeJoinTreeSuperparents)
      , MakeJoinTreeSupernodes(makeJoinTreeSupernodes)
      , MakeJoinTreeSuperarcs(makeJoinTreeSuperarcs)
      , MakeJoinTreeHyperparents(makeJoinTreeHyperparents)
      , MakeJoinTreeHypernodes(makeJoinTreeHypernodes)
      , MakeJoinTreeHyperarcs(makeJoinTreeHyperarcs)
      , MakeJoinTreeFirstSuperchild(makeJoinTreeFirstSuperchild)
      , MakeSplitTreeNumIterations(makeSplitTreeNumIterations)
      , MakeSplitTreeArcs(makeSplitTreeArcs)
      , MakeSplitTreeSuperparents(makeSplitTreeSuperparents)
      , MakeSplitTreeSupernodes(makeSplitTreeSupernodes)
      , MakeSplitTreeSuperarcs(makeSplitTreeSuperarcs)
      , MakeSplitTreeHyperparents(makeSplitTreeHyperparents)
      , MakeSplitTreeHypernodes(makeSplitTreeHypernodes)
      , MakeSplitTreeHyperarcs(makeSplitTreeHyperarcs)
      , MakeSplitTreeFirstSuperchild(makeSplitTreeFirstSuperchild)
      , MakeContourTreeNodes(makeContourTreeNodes)
      , MakeContourTreeArcs(makeContourTreeArcs)
      , MakeContourTreeSuperparents(makeContourTreeSuperparents)
      , MakeContourTreeSupernodes(makeContourTreeSupernodes)
      , MakeContourTreeSuperarcs(makeContourTreeSuperarcs)
      , MakeContourTreeAugmentnodes(makeContourTreeAugmentnodes)
      , MakeContourTreeAugmentarcs(makeContourTreeAugmentarcs)
      , MakeContourTreeHyperparents(makeContourTreeHyperparents)
      , MakeContourTreeWhenTransferred(makeContourTreeWhenTransferred)
      , MakeContourTreeHypernodes(makeContourTreeHypernodes)
      , MakeContourTreeHyperarcs(makeContourTreeHyperarcs)
      , MakeRegularStructureNodes(makeRegularStructureNodes)
      , MakeRegularStructureArcs(makeRegularStructureArcs)
      , MakeRegularStructureSuperparents(makeRegularStructureSuperparents)
      , MakeRegularStructureSupernodes(makeRegularStructureSupernodes)
      , MakeRegularStructureSuperarcs(makeRegularStructureSuperarcs)
      , MakeRegularStructureAugmentnodes(makeRegularStructureAugmentnodes)
      , MakeRegularStructureAugmentarcs(makeRegularStructureAugmentarcs)
      , MakeRegularStructureHyperparents(makeRegularStructureHyperparents)
      , MakeRegularStructureWhenTransferred(makeRegularStructureWhenTransferred)
      , MakeRegularStructureHypernodes(makeRegularStructureHypernodes)
      , MakeRegularStructureHyperarcs(makeRegularStructureHyperarcs)
    {
    }

    vtkm::worklet::contourtree_augmented::IdArrayType SortOrder;
    vtkm::worklet::contourtree_augmented::IdArrayType SortIndices;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPeaksJoin;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPitsJoin;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPeaksBuildRegularChainsJoin;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPitsBuildRegularChainsJoin;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPeaksSplit;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPitsSplit;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPeaksBuildRegularChainsSplit;
    vtkm::worklet::contourtree_augmented::IdArrayType MeshExtremaPitsBuildRegularChainsSplit;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitGlobalIndex;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitFirstEdge;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitOutdegree;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitHyperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitActiveVertices;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitEdgeNear;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitEdgeFar;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphJoinTreeInitActiveEdges;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitGlobalIndex;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitFirstEdge;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitOutdegree;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitHyperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitActiveVertices;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitEdgeNear;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitEdgeFar;
    vtkm::worklet::contourtree_augmented::IdArrayType ActiveGraphSplitTreeInitActiveEdges;
    vtkm::Id MakeJoinTreeNumIterations;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeArcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeSuperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeSupernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeSuperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeHyperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeHypernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeHyperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeJoinTreeFirstSuperchild;
    vtkm::Id MakeSplitTreeNumIterations;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeArcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeSuperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeSupernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeSuperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeHyperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeHypernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeHyperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeSplitTreeFirstSuperchild;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeNodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeArcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeSuperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeSupernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeSuperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeAugmentnodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeAugmentarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeHyperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeWhenTransferred;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeHypernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeContourTreeHyperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureNodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureArcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureSuperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureSupernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureSuperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureAugmentnodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureAugmentarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureHyperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureWhenTransferred;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureHypernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType MakeRegularStructureHyperarcs;
  };

  //
  // Internal helper function to run the individual steps of the ContourTreeAugmented worklet
  // locally here to be able to test intermediarry results. This function sets up the mesh
  // structure needed so we can all our detailed test
  template <typename FieldType, typename StorageType>
  void CallTestContourTreeAugmentedSteps(
    const vtkm::cont::ArrayHandle<FieldType, StorageType> fieldArray,
    const vtkm::Id3 meshSize,
    bool useMarchingCubes,
    unsigned int computeRegularStructure,
    ExpectedStepResults& expectedResults) const
  {
    using namespace vtkm::worklet::contourtree_augmented;
    // 2D Contour Tree
    if (meshSize[2] == 1)
    {
      // Build the mesh and fill in the values
      DataSetMeshTriangulation2DFreudenthal mesh(vtkm::Id2{ meshSize[0], meshSize[1] });
      // Run the contour tree on the mesh
      RunTestContourTreeAugmentedSteps(fieldArray,
                                       mesh,
                                       computeRegularStructure,
                                       mesh.GetMeshBoundaryExecutionObject(),
                                       expectedResults);
      return;
    }
    // 3D Contour Tree using marching cubes
    else if (useMarchingCubes)
    {
      // Build the mesh and fill in the values
      DataSetMeshTriangulation3DMarchingCubes mesh(meshSize);
      // Run the contour tree on the mesh
      RunTestContourTreeAugmentedSteps(fieldArray,
                                       mesh,
                                       computeRegularStructure,
                                       mesh.GetMeshBoundaryExecutionObject(),
                                       expectedResults);
      return;
    }
    // 3D Contour Tree with Freudenthal
    else
    {
      // Build the mesh and fill in the values
      DataSetMeshTriangulation3DFreudenthal mesh(meshSize);
      // Run the contour tree on the mesh
      RunTestContourTreeAugmentedSteps(fieldArray,
                                       mesh,
                                       computeRegularStructure,
                                       mesh.GetMeshBoundaryExecutionObject(),
                                       expectedResults);
      return;
    }
  }

  ///
  /// Helper function to generate the test data for 3D contour tree tests.
  /// The function in turns call the CallTestContourTreeAugmentedSteps
  /// function which sets up the mesh, which finally calls the
  /// RunTestContourTreeAugmentedSteps to actually execute all the
  /// steps and validate the results.
  ///
  void TestContourTreeAugmentedSteps3D(bool useMarchingCubes,
                                       unsigned int computeRegularStructure,
                                       ExpectedStepResults& expectedResults) const
  {
    // Create the input uniform cell set with values to contour
    vtkm::cont::DataSet dataSet = MakeTestDataSet().Make3DUniformDataSet1();

    vtkm::cont::CellSetStructured<3> cellSet;
    dataSet.GetCellSet().CopyTo(cellSet);

    vtkm::Id3 pointDimensions = cellSet.GetPointDimensions();

    vtkm::cont::ArrayHandle<vtkm::Float32> field;
    dataSet.GetField("pointvar").GetData().AsArrayHandle(field);

    // Run the specific test
    CallTestContourTreeAugmentedSteps(
      field, pointDimensions, useMarchingCubes, computeRegularStructure, expectedResults);
  }

  ///
  /// Helper function running all the steps from the contour tree worklet and testing
  /// at each step that the results match the provided expected results
  ///
  template <typename FieldType,
            typename StorageType,
            typename MeshClass,
            typename MeshBoundaryClass>
  void RunTestContourTreeAugmentedSteps(
    const vtkm::cont::ArrayHandle<FieldType, StorageType> fieldArray,
    MeshClass& mesh,
    unsigned int computeRegularStructure,
    const MeshBoundaryClass& meshBoundary,
    ExpectedStepResults& expectedResults) const
  {
    std::cout << "Testing contour tree steps with computeRegularStructure="
              << computeRegularStructure << " meshtype=" << typeid(MeshClass).name() << std::endl;

    using namespace vtkm::worklet::contourtree_augmented;

    // Stage 1: Load the data into the mesh. This is done in the Run() method above and accessible
    //          here via the mesh parameter. The actual data load is performed outside of the
    //          worklet in the example contour tree app (or whoever uses the worklet)

    // Stage 2 : Sort the data on the mesh to initialize sortIndex & indexReverse on the mesh
    // Sort the mesh data
    mesh.SortData(fieldArray);
    // Test that the sort is correct
    AssertIdArrayHandles(mesh.SortOrder, expectedResults.SortOrder, "mesh.SortOrder");
    AssertIdArrayHandles(mesh.SortOrder, expectedResults.SortOrder, "mesh.SortOrder");

    // Stage 3: Assign every mesh vertex to a peak
    MeshExtrema extrema(mesh.NumVertices);
    extrema.SetStarts(mesh, true);
    AssertIdArrayHandles(extrema.Peaks, expectedResults.MeshExtremaPeaksJoin, "extrema.Peaks");
    AssertIdArrayHandles(extrema.Pits, expectedResults.MeshExtremaPitsJoin, "extrema.Pits");

    extrema.BuildRegularChains(true);
    AssertIdArrayHandles(
      extrema.Peaks, expectedResults.MeshExtremaPeaksBuildRegularChainsJoin, "extrema.Peaks");
    AssertIdArrayHandles(
      extrema.Pits, expectedResults.MeshExtremaPitsBuildRegularChainsJoin, "extrema.Pits");

    // Stage 4: Identify join saddles & construct Active Join Graph
    MergeTree joinTree(mesh.NumVertices, true);
    ActiveGraph joinGraph(true);
    VTKM_TEST_ASSERT(test_equal(joinGraph.IsJoinGraph, true), "Bad joinGraph.IsJoinGraph");
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumIterations, 0), "Bad joinGraph.NumIterations");
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumSupernodes, 0), "Bad joinGraph.NumSupernodes");
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumHypernodes, 0), "Bad joinGraph.NumHypernodes");

    joinGraph.Initialise(mesh, extrema);
    VTKM_TEST_ASSERT(test_equal(joinGraph.IsJoinGraph, true), "Bad joinGraph.IsJoinGraph");
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumIterations, 0), "Bad joinGraph.NumIterations");
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumSupernodes, 0), "Bad joinGraph.NumSupernodes");
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumHypernodes, 0), "Bad joinGraph.NumHypernodes");
    AssertIdArrayHandles(joinGraph.GlobalIndex,
                         expectedResults.ActiveGraphJoinTreeInitGlobalIndex,
                         "joinGraph.GlobalIndex (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.FirstEdge,
                         expectedResults.ActiveGraphJoinTreeInitFirstEdge,
                         "joinGraph.FirstEdge (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.Outdegree,
                         expectedResults.ActiveGraphJoinTreeInitOutdegree,
                         "joinGraph.Outdegree (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.Hyperarcs,
                         expectedResults.ActiveGraphJoinTreeInitHyperarcs,
                         "joinGraph.Hyperarcs (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.ActiveVertices,
                         expectedResults.ActiveGraphJoinTreeInitActiveVertices,
                         "joinGraph.ActiveVertices (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.EdgeNear,
                         expectedResults.ActiveGraphJoinTreeInitEdgeNear,
                         "joinGraph.EdgeNear (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.EdgeFar,
                         expectedResults.ActiveGraphJoinTreeInitEdgeFar,
                         "joinGraph.EdgeFar (after joinGraph.Initialise");
    AssertIdArrayHandles(joinGraph.ActiveEdges,
                         expectedResults.ActiveGraphJoinTreeInitActiveEdges,
                         "joinGraph.ActiveEdges (after joinGraph.Initialise");

    // Stage 5: Compute Join Tree Hyperarcs from Active Join Graph
    joinGraph.MakeMergeTree(joinTree, extrema);
    // Make sure all temporary arrays have been released
    vtkm::worklet::contourtree_augmented::IdArrayType tempEmpty;
    AssertIdArrayHandles(
      joinGraph.GlobalIndex, tempEmpty, "joinGraph.GlobalIndex (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.FirstEdge, tempEmpty, "joinGraph.FirstEdge (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.Outdegree, tempEmpty, "joinGraph.Outdegree (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.Hyperarcs, tempEmpty, "joinGraph.Hyperarcs (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(joinGraph.ActiveVertices,
                         tempEmpty,
                         "joinGraph.ActiveVertices (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.EdgeNear, tempEmpty, "joinGraph.EdgeNear (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.EdgeFar, tempEmpty, "joinGraph.EdgeFar (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.ActiveEdges, tempEmpty, "joinGraph.ActiveEdges (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.EdgeSorter, tempEmpty, "joinGraph.EdgeSorters (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.SuperID, tempEmpty, "joinGraph.SuperID (after joinGraph.MakeMergeTree");
    AssertIdArrayHandles(
      joinGraph.HyperID, tempEmpty, "joinGraph.HyperID (after joinGraph.MakeMergeTree");
    // Make sure the actual join tree data is correct
    VTKM_TEST_ASSERT(test_equal(joinGraph.NumIterations, expectedResults.MakeJoinTreeNumIterations),
                     "Bad joinGraph.NumIterations");
    AssertIdArrayHandles(joinTree.Arcs, expectedResults.MakeJoinTreeArcs, "Bad joinTree.Arcs");
    AssertIdArrayHandles(
      joinTree.Superparents, expectedResults.MakeJoinTreeSuperparents, "Bad joinTree.Superparents");
    AssertIdArrayHandles(
      joinTree.Supernodes, expectedResults.MakeJoinTreeSupernodes, "Bad joinTree.Supernodes");
    AssertIdArrayHandles(
      joinTree.Superarcs, expectedResults.MakeJoinTreeSuperarcs, "Bad joinTree.Superarcs");
    AssertIdArrayHandles(
      joinTree.Hyperparents, expectedResults.MakeJoinTreeHyperparents, "Bad joinTree.Hyperparents");
    AssertIdArrayHandles(
      joinTree.Hypernodes, expectedResults.MakeJoinTreeHypernodes, "Bad joinTree.Hypernodes");
    AssertIdArrayHandles(
      joinTree.Hyperarcs, expectedResults.MakeJoinTreeHyperarcs, "Bad joinTree.Hyperarcs");
    AssertIdArrayHandles(joinTree.FirstSuperchild,
                         expectedResults.MakeJoinTreeFirstSuperchild,
                         "Bad joinTree.FirstSuperchild");

    // Stage 6: Assign every mesh vertex to a pit
    extrema.SetStarts(mesh, false);
    AssertIdArrayHandles(extrema.Peaks, expectedResults.MeshExtremaPeaksSplit, "extrema.Peaks");
    AssertIdArrayHandles(extrema.Pits, expectedResults.MeshExtremaPitsSplit, "extrema.Pits");

    extrema.BuildRegularChains(false);
    AssertIdArrayHandles(
      extrema.Peaks, expectedResults.MeshExtremaPeaksBuildRegularChainsSplit, "extrema.Peaks");
    AssertIdArrayHandles(
      extrema.Pits, expectedResults.MeshExtremaPitsBuildRegularChainsSplit, "extrema.Pits");

    // Stage 7:     Identify split saddles & construct Active Split Graph
    MergeTree splitTree(mesh.NumVertices, false);
    ActiveGraph splitGraph(false);
    VTKM_TEST_ASSERT(test_equal(splitGraph.IsJoinGraph, false), "Bad splitGraph.IsJoinGraph");
    VTKM_TEST_ASSERT(test_equal(splitGraph.NumIterations, 0), "Bad splitGraph.NumIterations");
    VTKM_TEST_ASSERT(test_equal(splitGraph.NumSupernodes, 0), "Bad splitGraph.NumSupernodes");
    VTKM_TEST_ASSERT(test_equal(splitGraph.NumHypernodes, 0), "Bad splitGraph.NumHypernodes");

    splitGraph.Initialise(mesh, extrema);
    VTKM_TEST_ASSERT(test_equal(splitGraph.IsJoinGraph, false), "Bad splitGraph.IsJoinGraph");
    VTKM_TEST_ASSERT(test_equal(splitGraph.NumIterations, 0), "Bad splitGraph.NumIterations");
    VTKM_TEST_ASSERT(test_equal(splitGraph.NumSupernodes, 0), "Bad splitGraph.NumSupernodes");
    VTKM_TEST_ASSERT(test_equal(splitGraph.NumHypernodes, 0), "Bad splitGraph.NumHypernodes");
    AssertIdArrayHandles(splitGraph.GlobalIndex,
                         expectedResults.ActiveGraphSplitTreeInitGlobalIndex,
                         "splitGraph.GlobalIndex (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.FirstEdge,
                         expectedResults.ActiveGraphSplitTreeInitFirstEdge,
                         "splitGraph.FirstEdge (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.Outdegree,
                         expectedResults.ActiveGraphSplitTreeInitOutdegree,
                         "splitGraph.Outdegree (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.Hyperarcs,
                         expectedResults.ActiveGraphSplitTreeInitHyperarcs,
                         "splitGraph.Hyperarcs (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.ActiveVertices,
                         expectedResults.ActiveGraphSplitTreeInitActiveVertices,
                         "splitGraph.ActiveVertices (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.EdgeNear,
                         expectedResults.ActiveGraphSplitTreeInitEdgeNear,
                         "splitGraph.EdgeNear (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.EdgeFar,
                         expectedResults.ActiveGraphSplitTreeInitEdgeFar,
                         "splitGraph.EdgeFar (after splitGraph.Initialise");
    AssertIdArrayHandles(splitGraph.ActiveEdges,
                         expectedResults.ActiveGraphSplitTreeInitActiveEdges,
                         "splitGraph.ActiveEdges (after splitGraph.Initialise");

    // Stage 8: Compute Split Tree Hyperarcs from Active Split Graph
    splitGraph.MakeMergeTree(splitTree, extrema);
    AssertIdArrayHandles(
      splitGraph.GlobalIndex, tempEmpty, "splitGraph.GlobalIndex (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.FirstEdge, tempEmpty, "splitGraph.FirstEdge (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.Outdegree, tempEmpty, "splitGraph.Outdegree (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.Hyperarcs, tempEmpty, "splitGraph.Hyperarcs (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(splitGraph.ActiveVertices,
                         tempEmpty,
                         "splitGraph.ActiveVertices (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.EdgeNear, tempEmpty, "splitGraph.EdgeNear (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.EdgeFar, tempEmpty, "splitGraph.EdgeFar (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.ActiveEdges, tempEmpty, "splitGraph.ActiveEdges (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.EdgeSorter, tempEmpty, "splitGraph.EdgeSorters (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.SuperID, tempEmpty, "splitGraph.SuperID (after splitGraph.MakeMergeTree");
    AssertIdArrayHandles(
      splitGraph.HyperID, tempEmpty, "splitGraph.HyperID (after splitGraph.MakeMergeTree");
    // Make sure the actual split tree data is correct
    VTKM_TEST_ASSERT(
      test_equal(splitGraph.NumIterations, expectedResults.MakeSplitTreeNumIterations),
      "Bad splitGraph.NumIterations");
    AssertIdArrayHandles(splitTree.Arcs, expectedResults.MakeSplitTreeArcs, "Bad splitTree.Arcs");
    AssertIdArrayHandles(splitTree.Superparents,
                         expectedResults.MakeSplitTreeSuperparents,
                         "Bad splitTree.Superparents");
    AssertIdArrayHandles(
      splitTree.Supernodes, expectedResults.MakeSplitTreeSupernodes, "Bad splitTree.Supernodes");
    AssertIdArrayHandles(
      splitTree.Superarcs, expectedResults.MakeSplitTreeSuperarcs, "Bad splitTree.Superarcs");
    AssertIdArrayHandles(splitTree.Hyperparents,
                         expectedResults.MakeSplitTreeHyperparents,
                         "Bad splitTree.Hyperparents");
    AssertIdArrayHandles(
      splitTree.Hypernodes, expectedResults.MakeSplitTreeHypernodes, "Bad splitTree.Hypernodes");
    AssertIdArrayHandles(
      splitTree.Hyperarcs, expectedResults.MakeSplitTreeHyperarcs, "Bad splitTree.Hyperarcs");
    AssertIdArrayHandles(splitTree.FirstSuperchild,
                         expectedResults.MakeSplitTreeFirstSuperchild,
                         "Bad splitTree.FirstSuperchild");

    // Stage 9: Join & Split Tree are Augmented, then combined to construct Contour Tree
    vtkm::worklet::contourtree_augmented::ContourTree contourTree;
    contourTree.Init(mesh.NumVertices);
    // confirm that the arcs and superparents are initialized as NO_SUCH_ELEMENT
    vtkm::worklet::contourtree_augmented::IdArrayType tempNoSuchElementArray;
    vtkm::cont::Algorithm::Copy(
      vtkm::cont::ArrayHandleConstant<vtkm::Id>(
        (vtkm::Id)vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT, mesh.NumVertices),
      tempNoSuchElementArray);
    AssertIdArrayHandles(
      contourTree.Arcs, tempNoSuchElementArray, "Bad contourTree.Arcs after init");
    AssertIdArrayHandles(
      contourTree.Superparents, tempNoSuchElementArray, "Bad contourTree.Superparents after init");

    ContourTreeMaker treeMaker(contourTree, joinTree, splitTree);
    // 9.1 First we compute the hyper- and super- structure
    treeMaker.ComputeHyperAndSuperStructure();
    // Confirm that we compute the correct hyper and super structure for the contour tree
    AssertIdArrayHandles(contourTree.Nodes,
                         expectedResults.MakeContourTreeNodes,
                         "Bad contourTree.Nodes after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Arcs,
                         expectedResults.MakeContourTreeArcs,
                         "Bad contourTree.Arcs after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Superparents,
                         expectedResults.MakeContourTreeSuperparents,
                         "Bad contourTree.Superparents after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Supernodes,
                         expectedResults.MakeContourTreeSupernodes,
                         "Bad contourTree.Supernodes after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Superarcs,
                         expectedResults.MakeContourTreeSuperarcs,
                         "Bad contourTree.Superarcs after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Augmentnodes,
                         expectedResults.MakeContourTreeAugmentnodes,
                         "Bad contourTree.Augmentnodes after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Augmentarcs,
                         expectedResults.MakeContourTreeAugmentarcs,
                         "Bad contourTree.Augmentarcs after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Hyperparents,
                         expectedResults.MakeContourTreeHyperparents,
                         "Bad contourTree.Hyperparents after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.WhenTransferred,
                         expectedResults.MakeContourTreeWhenTransferred,
                         "Bad contourTree.WhenTransferred after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Hypernodes,
                         expectedResults.MakeContourTreeHypernodes,
                         "Bad contourTree.Hypernodes after ComputeHyperAndSuperStructure");
    AssertIdArrayHandles(contourTree.Hyperarcs,
                         expectedResults.MakeContourTreeHyperarcs,
                         "Bad contourTree.Hyperarcs after ComputeHyperAndSuperStructure");

    // 9.2 Then we compute the regular structure
    if (computeRegularStructure == 1) // augment with all vertices
    {
      treeMaker.ComputeRegularStructure(extrema);
    }
    else if (computeRegularStructure == 2) // augment by the mesh boundary
    {
      treeMaker.ComputeBoundaryRegularStructure(extrema, mesh, meshBoundary);
    }
    // Asserts for treeMaker.ComputeRegularStructure / treeMaker.ComputeBoundaryRegularStructure
    // Confirm that we compute the correct hyper and super structure for the contour tree
    AssertIdArrayHandles(contourTree.Nodes,
                         expectedResults.MakeRegularStructureNodes,
                         "Bad contourTree.Nodes after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Arcs,
                         expectedResults.MakeRegularStructureArcs,
                         "Bad contourTree.Arcs after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Superparents,
                         expectedResults.MakeRegularStructureSuperparents,
                         "Bad contourTree.Superparents after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Supernodes,
                         expectedResults.MakeRegularStructureSupernodes,
                         "Bad contourTree.Supernodes after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Superarcs,
                         expectedResults.MakeRegularStructureSuperarcs,
                         "Bad contourTree.Superarcs after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Augmentnodes,
                         expectedResults.MakeRegularStructureAugmentnodes,
                         "Bad contourTree.Augmentnodes after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Augmentarcs,
                         expectedResults.MakeRegularStructureAugmentarcs,
                         "Bad contourTree.Augmentarcs after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Hyperparents,
                         expectedResults.MakeRegularStructureHyperparents,
                         "Bad contourTree.Hyperparents after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.WhenTransferred,
                         expectedResults.MakeRegularStructureWhenTransferred,
                         "Bad contourTree.WhenTransferred after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Hypernodes,
                         expectedResults.MakeRegularStructureHypernodes,
                         "Bad contourTree.Hypernodes after ComputeRegularStructure");
    AssertIdArrayHandles(contourTree.Hyperarcs,
                         expectedResults.MakeRegularStructureHyperarcs,
                         "Bad contourTree.Hyperarcs after ComputeRegularStructure");
  }


public:
  //
  // Create a uniform 2D structured cell set as input with values for contours
  //
  void TestContourTree_Mesh2D_Freudenthal() const
  {
    std::cout << "Testing ContourTree_Augmented 2D Mesh" << std::endl;

    // Create the input uniform cell set with values to contour
    vtkm::cont::DataSet dataSet = MakeTestDataSet().Make2DUniformDataSet1();

    vtkm::cont::CellSetStructured<2> cellSet;
    dataSet.GetCellSet().CopyTo(cellSet);

    vtkm::Id2 pointDimensions2D = cellSet.GetPointDimensions();
    vtkm::Id3 meshSize{ pointDimensions2D[0], pointDimensions2D[1], 1 };

    vtkm::cont::ArrayHandle<vtkm::Float32> field;
    dataSet.GetField("pointvar").GetData().AsArrayHandle(field);

    // Create the worklet and run it
    vtkm::worklet::ContourTreeAugmented contourTreeWorklet;
    vtkm::worklet::contourtree_augmented::ContourTree contourTree;
    vtkm::worklet::contourtree_augmented::IdArrayType meshSortOrder;
    vtkm::Id numIterations;
    const bool useMarchingCubes = false;
    const int computeRegularStructure = 1;

    contourTreeWorklet.Run(field,
                           contourTree,
                           meshSortOrder,
                           numIterations,
                           meshSize,
                           useMarchingCubes,
                           computeRegularStructure);

    // Compute the saddle peaks to make sure the contour tree is correct
    vtkm::worklet::contourtree_augmented::EdgePairArray saddlePeak;
    vtkm::worklet::contourtree_augmented::ProcessContourTree::CollectSortedSuperarcs(
      contourTree, meshSortOrder, saddlePeak);
    // Print the contour tree we computed
    std::cout << "Computed Contour Tree" << std::endl;
    vtkm::worklet::contourtree_augmented::PrintEdgePairArrayColumnLayout(saddlePeak);
    // Print the expected contour tree
    std::cout << "Expected Contour Tree" << std::endl;
    std::cout << "           0           12" << std::endl;
    std::cout << "           4           13" << std::endl;
    std::cout << "          12           13" << std::endl;
    std::cout << "          12           18" << std::endl;
    std::cout << "          12           20" << std::endl;
    std::cout << "          13           14" << std::endl;
    std::cout << "          13           19" << std::endl;

    VTKM_TEST_ASSERT(test_equal(saddlePeak.GetNumberOfValues(), 7),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(0), vtkm::make_Pair(0, 12)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(1), vtkm::make_Pair(4, 13)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(2), vtkm::make_Pair(12, 13)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(3), vtkm::make_Pair(12, 18)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(4), vtkm::make_Pair(12, 20)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(5), vtkm::make_Pair(13, 14)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(6), vtkm::make_Pair(13, 19)),
                     "Wrong result for ContourTree filter");
  }

  void TestContourTree_Mesh3D_Freudenthal() const
  {
    std::cout << "Testing ContourTree_Augmented 3D Mesh" << std::endl;

    // Create the input uniform cell set with values to contour
    vtkm::cont::DataSet dataSet = MakeTestDataSet().Make3DUniformDataSet1();

    vtkm::cont::CellSetStructured<3> cellSet;
    dataSet.GetCellSet().CopyTo(cellSet);

    vtkm::Id3 pointDimensions = cellSet.GetPointDimensions();

    vtkm::cont::ArrayHandle<vtkm::Float32> field;
    dataSet.GetField("pointvar").GetData().AsArrayHandle(field);

    // Create the worklet and run it
    vtkm::worklet::ContourTreeAugmented contourTreeWorklet;
    vtkm::worklet::contourtree_augmented::ContourTree contourTree;
    vtkm::worklet::contourtree_augmented::IdArrayType meshSortOrder;
    vtkm::Id numIterations;
    const bool useMarchingCubes = false;
    const int computeRegularStructure = 1;

    contourTreeWorklet.Run(field,
                           contourTree,
                           meshSortOrder,
                           numIterations,
                           pointDimensions,
                           useMarchingCubes,
                           computeRegularStructure);

    // Compute the saddle peaks to make sure the contour tree is correct
    vtkm::worklet::contourtree_augmented::EdgePairArray saddlePeak;
    vtkm::worklet::contourtree_augmented::ProcessContourTree::CollectSortedSuperarcs(
      contourTree, meshSortOrder, saddlePeak);
    // Print the contour tree we computed
    std::cout << "Computed Contour Tree" << std::endl;
    vtkm::worklet::contourtree_augmented::PrintEdgePairArrayColumnLayout(saddlePeak);
    // Print the expected contour tree
    std::cout << "Expected Contour Tree" << std::endl;
    std::cout << "           0           67" << std::endl;
    std::cout << "          31           42" << std::endl;
    std::cout << "          42           43" << std::endl;
    std::cout << "          42           56" << std::endl;
    std::cout << "          56           67" << std::endl;
    std::cout << "          56           92" << std::endl;
    std::cout << "          62           67" << std::endl;
    std::cout << "          81           92" << std::endl;
    std::cout << "          92           93" << std::endl;

    // Make sure the contour tree is correct
    VTKM_TEST_ASSERT(test_equal(saddlePeak.GetNumberOfValues(), 9),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(0), vtkm::make_Pair(0, 67)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(1), vtkm::make_Pair(31, 42)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(2), vtkm::make_Pair(42, 43)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(3), vtkm::make_Pair(42, 56)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(4), vtkm::make_Pair(56, 67)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(5), vtkm::make_Pair(56, 92)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(6), vtkm::make_Pair(62, 67)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(7), vtkm::make_Pair(81, 92)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(8), vtkm::make_Pair(92, 93)),
                     "Wrong result for ContourTree filter");
  }

  void TestContourTree_Mesh3D_MarchingCubes() const
  {
    std::cout << "Testing ContourTree_Augmented 3D Mesh Marching Cubes" << std::endl;

    // Create the input uniform cell set with values to contour
    vtkm::cont::DataSet dataSet = MakeTestDataSet().Make3DUniformDataSet1();

    vtkm::cont::CellSetStructured<3> cellSet;
    dataSet.GetCellSet().CopyTo(cellSet);

    vtkm::Id3 pointDimensions = cellSet.GetPointDimensions();

    vtkm::cont::ArrayHandle<vtkm::Float32> field;
    dataSet.GetField("pointvar").GetData().AsArrayHandle(field);

    // Create the worklet and run it
    vtkm::worklet::ContourTreeAugmented contourTreeWorklet;
    vtkm::worklet::contourtree_augmented::ContourTree contourTree;
    vtkm::worklet::contourtree_augmented::IdArrayType meshSortOrder;
    vtkm::Id numIterations;
    const bool useMarchingCubes = true;
    const int computeRegularStructure = 1;

    contourTreeWorklet.Run(field,
                           contourTree,
                           meshSortOrder,
                           numIterations,
                           pointDimensions,
                           useMarchingCubes,
                           computeRegularStructure);

    // Compute the saddle peaks to make sure the contour tree is correct
    vtkm::worklet::contourtree_augmented::EdgePairArray saddlePeak;
    vtkm::worklet::contourtree_augmented::ProcessContourTree::CollectSortedSuperarcs(
      contourTree, meshSortOrder, saddlePeak);
    // Print the contour tree we computed
    std::cout << "Computed Contour Tree" << std::endl;
    vtkm::worklet::contourtree_augmented::PrintEdgePairArrayColumnLayout(saddlePeak);
    // Print the expected contour tree
    std::cout << "Expected Contour Tree" << std::endl;
    std::cout << "           0          118" << std::endl;
    std::cout << "          31           41" << std::endl;
    std::cout << "          41           43" << std::endl;
    std::cout << "          41           56" << std::endl;
    std::cout << "          56           67" << std::endl;
    std::cout << "          56           91" << std::endl;
    std::cout << "          62           67" << std::endl;
    std::cout << "          67          118" << std::endl;
    std::cout << "          81           91" << std::endl;
    std::cout << "          91           93" << std::endl;
    std::cout << "         118          124" << std::endl;

    VTKM_TEST_ASSERT(test_equal(saddlePeak.GetNumberOfValues(), 11),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(0), vtkm::make_Pair(0, 118)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(1), vtkm::make_Pair(31, 41)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(2), vtkm::make_Pair(41, 43)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(3), vtkm::make_Pair(41, 56)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(4), vtkm::make_Pair(56, 67)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(5), vtkm::make_Pair(56, 91)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(6), vtkm::make_Pair(62, 67)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(7), vtkm::make_Pair(67, 118)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(8), vtkm::make_Pair(81, 91)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(9), vtkm::make_Pair(91, 93)),
                     "Wrong result for ContourTree filter");
    VTKM_TEST_ASSERT(test_equal(saddlePeak.WritePortal().Get(10), vtkm::make_Pair(118, 124)),
                     "Wrong result for ContourTree filter");
  }


  void TestContourTreeAugmentedStepsFreudenthal3D(unsigned int computeRegularStructure) const
  {

    // Create the expected results
    vtkm::Id expectedSortOrderArr[125] = {
      0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,
      18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  34,  35,  39,  40,  44,
      45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  59,  60,  64,  65,  69,  70,  71,
      72,  73,  74,  75,  76,  77,  78,  79,  80,  84,  85,  89,  90,  94,  95,  96,  97,  98,
      99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
      117, 118, 119, 120, 121, 122, 123, 124, 62,  67,  63,  57,  61,  66,  58,  68,  56,  87,
      37,  83,  91,  33,  41,  82,  92,  32,  42,  86,  88,  36,  38,  81,  93,  31,  43
    };
    vtkm::worklet::contourtree_augmented::IdArrayType expectedSortOrder =
      vtkm::cont::make_ArrayHandle(expectedSortOrderArr, 125, vtkm::CopyFlag::On);

    vtkm::Id expectedSortIndicesArr[125] = {
      0,   1,   2,   3,   4,   5,   6,   7,   8,  9,   10,  11,  12,  13,  14,  15,  16,  17,
      18,  19,  20,  21,  22,  23,  24,  25,  26, 27,  28,  29,  30,  123, 115, 111, 31,  32,
      119, 108, 120, 33,  34,  112, 116, 124, 35, 36,  37,  38,  39,  40,  41,  42,  43,  44,
      45,  46,  106, 101, 104, 47,  48,  102, 98, 100, 49,  50,  103, 99,  105, 51,  52,  53,
      54,  55,  56,  57,  58,  59,  60,  61,  62, 121, 113, 109, 63,  64,  117, 107, 118, 65,
      66,  110, 114, 122, 67,  68,  69,  70,  71, 72,  73,  74,  75,  76,  77,  78,  79,  80,
      81,  82,  83,  84,  85,  86,  87,  88,  89, 90,  91,  92,  93,  94,  95,  96,  97
    };
    vtkm::worklet::contourtree_augmented::IdArrayType expectedSortIndices =
      vtkm::cont::make_ArrayHandle(expectedSortIndicesArr, 125, vtkm::CopyFlag::On);

    //
    // Join Tree Set Starts
    //
    vtkm::Id meshExtremaPeaksJoinArr[125] = {
      1,   2,   3,   4,   9,   6,   7,   8,   9,   14,  11,  12,  13,  14,  19,  16,  17,  18,
      19,  24,  21,  22,  23,  24,  40,  26,  27,  28,  29,  31,  123, 111, 119, 120, 112, 124,
      37,  112, 116, 124, 124, 42,  43,  44,  45,  47,  106, 111, 102, 111, 103, 120, 53,  103,
      112, 116, 124, 58,  59,  60,  61,  63,  121, 104, 117, 104, 110, 100, 69,  110, 103, 99,
      105, 74,  75,  76,  77,  82,  79,  121, 113, 109, 109, 84,  121, 121, 113, 109, 89,  117,
      117, 107, 118, 94,  110, 110, 114, 122, 123, 119, 115, 115, 106, 119, 111, 108, 123, 113,
      115, 113, 117, 115, 119, 121, 117, 123, 119, 121, 122, 123, 124, 121, 122, 123, 124
    };
    for (vtkm::Id i = 124; i > 120; i--)
    {
      meshExtremaPeaksJoinArr[i] =
        meshExtremaPeaksJoinArr[i] | vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPeaksJoin =
      vtkm::cont::make_ArrayHandle(meshExtremaPeaksJoinArr, 125, vtkm::CopyFlag::On);
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPitsJoin;
    vtkm::cont::Algorithm::Copy(vtkm::cont::ArrayHandleConstant<vtkm::Id>(0, 125),
                                meshExtremaPitsJoin);

    //
    // Join Tree Build Regular chains
    //
    vtkm::Id meshExtremaPeaksBuildRegularChainsJoinArr[125] = {
      124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124,
      124, 124, 124, 124, 124, 124, 124, 123, 123, 123, 123, 123, 123, 123, 123, 124, 123, 124,
      123, 123, 123, 124, 124, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 124, 123, 123,
      123, 123, 124, 123, 123, 123, 123, 123, 121, 123, 121, 123, 121, 123, 121, 121, 123, 123,
      123, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121,
      121, 121, 122, 121, 121, 121, 121, 122, 123, 123, 123, 123, 123, 123, 123, 123, 123, 121,
      123, 121, 121, 123, 123, 121, 121, 123, 123, 121, 122, 123, 124, 121, 122, 123, 124
    };
    for (vtkm::Id i = 0; i < 125; i++)
    {
      meshExtremaPeaksBuildRegularChainsJoinArr[i] = meshExtremaPeaksBuildRegularChainsJoinArr[i] |
        vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPeaksBuildRegularChainsJoin =
      vtkm::cont::make_ArrayHandle(
        meshExtremaPeaksBuildRegularChainsJoinArr, 125, vtkm::CopyFlag::On);

    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPitsBuildRegularChainsJoin =
      meshExtremaPitsJoin; // should remain all at 0


    //
    // Split Tree Set Starts
    //
    vtkm::Id meshExtremaPeaksSplitArr[125] = {
      124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124,
      124, 124, 124, 124, 124, 124, 124, 123, 123, 123, 123, 123, 123, 123, 123, 124, 123, 124,
      123, 123, 123, 124, 124, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 124, 123, 123,
      123, 123, 124, 123, 123, 123, 123, 123, 121, 123, 121, 123, 121, 123, 121, 121, 123, 123,
      123, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121,
      121, 121, 122, 121, 121, 121, 121, 122, 123, 123, 123, 123, 123, 123, 123, 123, 123, 121,
      123, 121, 121, 123, 123, 121, 121, 123, 123, 121, 122, 123, 124, 121, 122, 123, 124
    };
    for (vtkm::Id i = 0; i < 125; i++)
    {
      meshExtremaPeaksSplitArr[i] =
        meshExtremaPeaksSplitArr[i] | vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPeaksSplit =
      vtkm::cont::make_ArrayHandle(meshExtremaPeaksSplitArr, 125, vtkm::CopyFlag::On);

    vtkm::Id meshExtremaPitsSplitArr[125] = {
      0,   0,  1,   2,  3,  0,  1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12, 13, 14, 15,
      16,  17, 18,  19, 0,  1,  2,  3,  4,  25,  3,  30, 8,  32,  13, 34, 15, 16, 17, 18, 25,
      26,  27, 28,  29, 41, 28, 46, 47, 48, 49,  50, 34, 38, 39,  51, 41, 42, 43, 44, 45, 57,
      44,  62, 63,  64, 65, 66, 50, 54, 55, 67,  57, 58, 59, 60,  61, 73, 57, 58, 59, 60, 78,
      62,  80, 81,  82, 83, 64, 85, 86, 87, 88,  66, 90, 91, 92,  98, 98, 98, 26, 30, 32, 27,
      100, 25, 106, 6,  43, 48, 2,  10, 42, 102, 1,  11, 46, 101, 5,  7,  41, 98, 0,  12
    };
    meshExtremaPitsSplitArr[0] =
      meshExtremaPitsSplitArr[0] | vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    meshExtremaPitsSplitArr[98] =
      meshExtremaPitsSplitArr[97] | vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPitsSplit =
      vtkm::cont::make_ArrayHandle(meshExtremaPitsSplitArr, 125, vtkm::CopyFlag::On);

    //
    // Split Tree Build Regular chains
    //
    vtkm::Id meshExtremaPeaksBuildRegularChainsSplitArr[125] = {
      124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124, 124,
      124, 124, 124, 124, 124, 124, 124, 123, 123, 123, 123, 123, 123, 123, 123, 124, 123, 124,
      123, 123, 123, 124, 124, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 124, 123, 123,
      123, 123, 124, 123, 123, 123, 123, 123, 121, 123, 121, 123, 121, 123, 121, 121, 123, 123,
      123, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121, 121,
      121, 121, 122, 121, 121, 121, 121, 122, 123, 123, 123, 123, 123, 123, 123, 123, 123, 121,
      123, 121, 121, 123, 123, 121, 121, 123, 123, 121, 122, 123, 124, 121, 122, 123, 124
    };
    for (vtkm::Id i = 0; i < 125; i++)
    {
      meshExtremaPeaksBuildRegularChainsSplitArr[i] =
        meshExtremaPeaksBuildRegularChainsSplitArr[i] |
        vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPeaksBuildRegularChainsSplit =
      vtkm::cont::make_ArrayHandle(
        meshExtremaPeaksBuildRegularChainsSplitArr, 125, vtkm::CopyFlag::On);

    vtkm::Id meshExtremaPitsBuildRegularChainsSplitArr[125] = {
      0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,
      0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,
      0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,
      0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  98, 98,
      98, 0, 0, 0, 0, 98, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 98, 0,  0,
    };
    for (vtkm::Id i = 0; i < 125; i++)
    {
      meshExtremaPitsBuildRegularChainsSplitArr[i] = meshExtremaPitsBuildRegularChainsSplitArr[i] |
        vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType meshExtremaPitsBuildRegularChainsSplit =
      vtkm::cont::make_ArrayHandle(
        meshExtremaPitsBuildRegularChainsSplitArr, 125, vtkm::CopyFlag::On);

    //
    // Join Graph Initialize
    //
    // Active graph join graph initialize GlobalIndex
    vtkm::Id activeGraphJoinTreeInitGlobalIndexArr[12] = { 103, 104, 105, 106, 113, 114,
                                                           115, 116, 121, 122, 123, 124 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitGlobalIndex =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitGlobalIndexArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize FirstEdge
    vtkm::Id activeGraphJoinTreeInitFirstEdgeArr[12] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 16, 16, 16
    };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitFirstEdge =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitFirstEdgeArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize Outdegree
    vtkm::Id activeGraphJoinTreeInitOutdegreeArr[12] = { 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitOutdegree =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitOutdegreeArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize Hyperarcs
    vtkm::Id activeGraphJoinTreeInitHyperarcsArr[12] = {
      10, 10, 10, 10, 8, 8, 10, 10, 8, 9, 10, 11
    };
    for (vtkm::Id i = 8; i < 12; i++)
    {
      activeGraphJoinTreeInitHyperarcsArr[i] = activeGraphJoinTreeInitHyperarcsArr[i] |
        vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitHyperarcs =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitHyperarcsArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize ActiveVertices
    vtkm::Id activeGraphJoinTreeInitActiveVerticesArr[12] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
    };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitActiveVertices =
      vtkm::cont::make_ArrayHandle(
        activeGraphJoinTreeInitActiveVerticesArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize EdgeNear
    vtkm::Id activeGraphJoinTreeInitEdgeNearArr[16] = { 0, 0, 1, 1, 2, 2, 3, 3,
                                                        4, 4, 5, 5, 6, 6, 7, 7 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitEdgeNear =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitEdgeNearArr, 16, vtkm::CopyFlag::On);

    // Active graph join graph initialize , EdgeFar
    vtkm::Id activeGraphJoinTreeInitEdgeFarArr[16] = { 10, 8, 10, 9, 10, 9,  10, 8,
                                                       8,  9, 8,  9, 10, 11, 10, 11 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitEdgeFar =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitEdgeFarArr, 16, vtkm::CopyFlag::On);

    // Active graph join graph initialize , ActiveEdges
    vtkm::Id activeGraphJoinTreeInitActiveEdgesArr[16] = { 0, 1, 2,  3,  4,  5,  6,  7,
                                                           8, 9, 10, 11, 12, 13, 14, 15 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphJoinTreeInitActiveEdges =
      vtkm::cont::make_ArrayHandle(activeGraphJoinTreeInitActiveEdgesArr, 16, vtkm::CopyFlag::On);

    //
    // Split Graph Initialize
    //
    // Active graph join graph initialize GlobalIndex
    vtkm::Id activeGraphSplitTreeInitGlobalIndexArr[8] = { 0, 98, 99, 100, 101, 102, 107, 108 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitGlobalIndex =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitGlobalIndexArr, 8, vtkm::CopyFlag::On);

    // Active graph join graph initialize FirstEdge
    vtkm::Id activeGraphSplitTreeInitFirstEdgeArr[8] = { 0, 0, 0, 2, 4, 6, 8, 10 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitFirstEdge =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitFirstEdgeArr, 8, vtkm::CopyFlag::On);

    // Active graph join graph initialize Outdegree
    vtkm::Id activeGraphSplitTreeInitOutdegreeArr[8] = { 0, 0, 2, 2, 2, 2, 2, 2 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitOutdegree =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitOutdegreeArr, 8, vtkm::CopyFlag::On);

    // Active graph join graph initialize Hyperarcs
    vtkm::Id activeGraphSplitTreeInitHyperarcsArr[8] = { 0, 1, 1, 1, 0, 0, 0, 0 };
    for (vtkm::Id i = 0; i < 2; i++)
    {
      activeGraphSplitTreeInitHyperarcsArr[i] = activeGraphSplitTreeInitHyperarcsArr[i] |
        vtkm::worklet::contourtree_augmented::TERMINAL_ELEMENT;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitHyperarcs =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitHyperarcsArr, 8, vtkm::CopyFlag::On);

    // Active graph join graph initialize ActiveVertices
    vtkm::Id activeGraphSplitTreeInitActiveVerticesArr[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitActiveVertices =
      vtkm::cont::make_ArrayHandle(
        activeGraphSplitTreeInitActiveVerticesArr, 8, vtkm::CopyFlag::On);

    // Active graph join graph initialize EdgeNear
    vtkm::Id activeGraphSplitTreeInitEdgeNearArr[12] = { 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitEdgeNear =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitEdgeNearArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize , EdgeFar
    vtkm::Id activeGraphSplitTreeInitEdgeFarArr[12] = { 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitEdgeFar =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitEdgeFarArr, 12, vtkm::CopyFlag::On);

    // Active graph join graph initialize , ActiveEdges
    vtkm::Id activeGraphSplitTreeInitActiveEdgesArr[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
    vtkm::worklet::contourtree_augmented::IdArrayType activeGraphSplitTreeInitActiveEdges =
      vtkm::cont::make_ArrayHandle(activeGraphSplitTreeInitActiveEdgesArr, 12, vtkm::CopyFlag::On);

    //
    // JoinTree MakeMergeTree
    //
    vtkm::Id makeJoinTreeNumIterations = 2;

    vtkm::Id makeJoinTreeArcsArr[125] = {
      0,   0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,
      17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,
      35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,
      53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
      71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,
      89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,  100, 101, 102, 103, 104, 105, 106,
      106, 107, 109, 108, 111, 110, 113, 112, 115, 114, 114, 116, 116, 117, 118, 119, 120
    };
    makeJoinTreeArcsArr[0] =
      makeJoinTreeArcsArr[0] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeArcs =
      vtkm::cont::make_ArrayHandle(makeJoinTreeArcsArr, 125, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeSuperparentsArr[125] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 2, 1, 1, 2, 2, 1, 1, 2, 2, 3, 4, 5, 6, 3, 4, 5, 6
    };
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeSuperparents =
      vtkm::cont::make_ArrayHandle(makeJoinTreeSuperparentsArr, 125, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeSupernodesArr[7] = { 106, 114, 116, 121, 122, 123, 124 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeSupernodes =
      vtkm::cont::make_ArrayHandle(makeJoinTreeSupernodesArr, 7, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeSuperarcsArr[7] = { 0, 0, 0, 1, 1, 2, 2 };
    makeJoinTreeSuperarcsArr[0] =
      makeJoinTreeSuperarcsArr[0] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeSuperarcs =
      vtkm::cont::make_ArrayHandle(makeJoinTreeSuperarcsArr, 7, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeHyperparentsArr[7] = { 0, 1, 2, 3, 4, 5, 6 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeHyperparents =
      vtkm::cont::make_ArrayHandle(makeJoinTreeHyperparentsArr, 7, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeHypernodesArr[7] = { 0, 1, 2, 3, 4, 5, 6 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeHypernodes =
      vtkm::cont::make_ArrayHandle(makeJoinTreeHypernodesArr, 7, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeHyperarcsArr[7] = { 0, 0, 0, 1, 1, 2, 2 };
    makeJoinTreeHyperarcsArr[0] =
      makeJoinTreeHyperarcsArr[0] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeHyperarcs =
      vtkm::cont::make_ArrayHandle(makeJoinTreeHyperarcsArr, 7, vtkm::CopyFlag::On);

    vtkm::Id makeJoinTreeFirstSuperchildArr[7] = { 0, 1, 2, 3, 4, 5, 6 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeJoinTreeFirstSuperchild =
      vtkm::cont::make_ArrayHandle(makeJoinTreeFirstSuperchildArr, 7, vtkm::CopyFlag::On);


    //
    // SplitTree MakeMergeTree
    //
    vtkm::Id makeSplitTreeNumIterations = 1;

    vtkm::Id makeSplitTreeArcsArr[125] = {
      1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,
      19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,
      37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,
      55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,
      73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,
      91,  92,  93,  94,  95,  96,  97,  99,  99,  100, 101, 102, 103, 104, 105, 106, 107, 108,
      109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 0
    };
    makeSplitTreeArcsArr[124] =
      makeSplitTreeArcsArr[124] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeArcs =
      vtkm::cont::make_ArrayHandle(makeSplitTreeArcsArr, 125, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeSuperparentsArr[125] = {
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeSuperparents =
      vtkm::cont::make_ArrayHandle(makeSplitTreeSuperparentsArr, 125, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeSupernodesArr[3] = { 99, 98, 0 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeSupernodes =
      vtkm::cont::make_ArrayHandle(makeSplitTreeSupernodesArr, 3, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeSuperarcsArr[3] = {
      0 | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT, 0, 0
    };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeSuperarcs =
      vtkm::cont::make_ArrayHandle(makeSplitTreeSuperarcsArr, 3, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeHyperparentsArr[3] = { 2, 1, 0 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeHyperparents =
      vtkm::cont::make_ArrayHandle(makeSplitTreeHyperparentsArr, 3, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeHypernodesArr[3] = {
      2,
      1,
      0,
    };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeHypernodes =
      vtkm::cont::make_ArrayHandle(makeSplitTreeHypernodesArr, 3, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeHyperarcsArr[3] = {
      0, 0, 0 | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT
    };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeHyperarcs =
      vtkm::cont::make_ArrayHandle(makeSplitTreeHyperarcsArr, 3, vtkm::CopyFlag::On);

    vtkm::Id makeSplitTreeFirstSuperchildArr[3] = { 2, 1, 0 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeSplitTreeFirstSuperchild =
      vtkm::cont::make_ArrayHandle(makeSplitTreeFirstSuperchildArr, 3, vtkm::CopyFlag::On);

    //
    //  Contour Tree Compute
    //
    vtkm::worklet::contourtree_augmented::IdArrayType tempNoSuchElementArray;
    vtkm::cont::Algorithm::Copy(
      vtkm::cont::ArrayHandleConstant<vtkm::Id>(
        (vtkm::Id)vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT, 125),
      tempNoSuchElementArray);

    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeNodes;

    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeArcs = tempNoSuchElementArray;

    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeSuperparents =
      tempNoSuchElementArray;

    vtkm::Id makeContourTreeSupernodesArr[10] = { 121, 122, 123, 124, 0, 98, 114, 116, 99, 106 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeSupernodes =
      vtkm::cont::make_ArrayHandle(makeContourTreeSupernodesArr, 10, vtkm::CopyFlag::On);

    vtkm::Id makeContourTreeSuperarcsArr[10] = { 6, 6, 7, 7, 8, 8, 9, 9, 9, 0 };
    makeContourTreeSuperarcsArr[4] =
      makeContourTreeSuperarcsArr[5] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
    makeContourTreeSuperarcsArr[5] =
      makeContourTreeSuperarcsArr[6] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
    makeContourTreeSuperarcsArr[8] =
      makeContourTreeSuperarcsArr[8] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
    makeContourTreeSuperarcsArr[9] =
      makeContourTreeSuperarcsArr[9] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;

    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeSuperarcs =
      vtkm::cont::make_ArrayHandle(makeContourTreeSuperarcsArr, 10, vtkm::CopyFlag::On);

    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeAugmentnodes;

    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeAugmentarcs;

    vtkm::Id makeContourTreeHyperparentsArr[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeHyperparents =
      vtkm::cont::make_ArrayHandle(makeContourTreeHyperparentsArr, 10, vtkm::CopyFlag::On);

    vtkm::Id makeContourTreeWhenTransferredArr[10] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 4 };
    for (vtkm::Id i = 0; i < 10; i++)
    {
      makeContourTreeWhenTransferredArr[i] =
        makeContourTreeWhenTransferredArr[i] | vtkm::worklet::contourtree_augmented::IS_HYPERNODE;
    }
    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeWhenTransferred =
      vtkm::cont::make_ArrayHandle(makeContourTreeWhenTransferredArr, 10, vtkm::CopyFlag::On);

    vtkm::Id makeContourTreeHypernodesArr[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeHypernodes =
      vtkm::cont::make_ArrayHandle(makeContourTreeHypernodesArr, 10, vtkm::CopyFlag::On);

    vtkm::Id makeContourTreeHyperarcsArr[10] = { 6, 6, 7, 7, 8, 8, 9, 9, 9, 0 };
    makeContourTreeHyperarcsArr[4] =
      makeContourTreeHyperarcsArr[5] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
    makeContourTreeHyperarcsArr[5] =
      makeContourTreeHyperarcsArr[6] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
    makeContourTreeHyperarcsArr[8] =
      makeContourTreeHyperarcsArr[8] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
    makeContourTreeHyperarcsArr[9] =
      makeContourTreeHyperarcsArr[9] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
    vtkm::worklet::contourtree_augmented::IdArrayType makeContourTreeHyperarcs =
      vtkm::cont::make_ArrayHandle(makeContourTreeHyperarcsArr, 10, vtkm::CopyFlag::On);

    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureNodes;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureArcs;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureSuperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureSupernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureSuperarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureAugmentnodes;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureAugmentarcs;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureHyperparents;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureWhenTransferred;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureHypernodes;
    vtkm::worklet::contourtree_augmented::IdArrayType makeRegularStructureHyperarcs;

    // Depending on the computeRegularStructure setting the values of some arrays changes
    // We define them here so we can populate the data in the if/else blocks below and
    // still keep the data in scope during the actual tests
    vtkm::Id* makeRegularStructureNodesArr = NULL;
    vtkm::Id* makeRegularStructureArcsArr = NULL;
    vtkm::Id* makeRegularStructureSuperparentsArr = NULL;
    vtkm::Id* makeRegularStructureAugmentnodesArr = NULL;
    vtkm::Id* makeRegularStructureAugmentarcsArr = NULL;

    if (computeRegularStructure == 0)
    {
      // No augmentation so nothing changes
      makeRegularStructureNodes = makeContourTreeNodes;
      makeRegularStructureArcs = makeContourTreeArcs;
      makeRegularStructureSuperparents = makeContourTreeSuperparents;
      makeRegularStructureSupernodes = makeContourTreeSupernodes;
      makeRegularStructureSuperarcs = makeContourTreeSuperarcs;
      makeRegularStructureAugmentnodes = makeContourTreeAugmentnodes;
      makeRegularStructureAugmentarcs = makeContourTreeAugmentarcs;
      makeRegularStructureHyperparents = makeContourTreeHyperparents;
      makeRegularStructureWhenTransferred = makeContourTreeWhenTransferred;
      makeRegularStructureHypernodes = makeContourTreeHypernodes;
      makeRegularStructureHyperarcs = makeContourTreeHyperarcs;
    }
    else if (computeRegularStructure == 1)
    {
      makeRegularStructureNodesArr = new vtkm::Id[125]{
        121, 117, 122, 118, 123, 119, 124, 120, 0,   1,  2,   3,   4,   5,   6,   7,   8,  9,
        10,  11,  12,  13,  14,  15,  16,  17,  18,  19, 20,  21,  22,  23,  24,  25,  26, 27,
        28,  29,  30,  31,  32,  33,  34,  35,  36,  37, 38,  39,  40,  41,  42,  43,  44, 45,
        46,  47,  48,  49,  50,  51,  52,  53,  54,  55, 56,  57,  58,  59,  60,  61,  62, 63,
        64,  65,  66,  67,  68,  69,  70,  71,  72,  73, 74,  75,  76,  77,  78,  79,  80, 81,
        82,  83,  84,  85,  86,  87,  88,  89,  90,  91, 92,  93,  94,  95,  96,  97,  98, 114,
        113, 110, 109, 107, 116, 115, 112, 111, 108, 99, 100, 101, 102, 103, 104, 105, 106
      };
      makeRegularStructureNodes =
        vtkm::cont::make_ArrayHandle(makeRegularStructureNodesArr, 125, vtkm::CopyFlag::On);

      makeRegularStructureArcsArr = new vtkm::Id[125]{
        1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17, 18,
        19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35, 36,
        37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53, 54,
        55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71, 72,
        73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89, 90,
        91,  92,  93,  94,  95,  96,  97,  99,  99,  100, 101, 102, 103, 104, 105, 106, 0,  106,
        106, 107, 109, 108, 111, 110, 113, 112, 115, 114, 114, 116, 116, 117, 118, 119, 120
      };
      makeRegularStructureArcsArr[106] =
        makeRegularStructureArcsArr[107] | vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
      for (vtkm::Id i = 0; i < 106; i++)
      {
        makeRegularStructureArcsArr[i] =
          makeRegularStructureArcsArr[i] | vtkm::worklet::contourtree_augmented::IS_ASCENDING;
      }
      makeRegularStructureArcs =
        vtkm::cont::make_ArrayHandle(makeRegularStructureArcsArr, 125, vtkm::CopyFlag::On);

      makeRegularStructureSuperparentsArr =
        new vtkm::Id[125]{ 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                           4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                           4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                           4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                           4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 8, 8, 8, 8, 8, 8,
                           8, 9, 6, 7, 6, 6, 7, 7, 6, 6, 7, 7, 0, 1, 2, 3, 0, 1, 2, 3 };
      makeRegularStructureSuperparents =
        vtkm::cont::make_ArrayHandle(makeRegularStructureSuperparentsArr, 125, vtkm::CopyFlag::On);
      makeRegularStructureSupernodes = makeContourTreeSupernodes;
      makeRegularStructureSuperarcs = makeContourTreeSuperarcs;
      makeRegularStructureAugmentnodes = makeContourTreeAugmentnodes;
      makeRegularStructureAugmentarcs = makeContourTreeAugmentarcs;
      makeRegularStructureHyperparents = makeContourTreeHyperparents;
      makeRegularStructureWhenTransferred = makeContourTreeWhenTransferred;
      makeRegularStructureHypernodes = makeContourTreeHypernodes;
      makeRegularStructureHyperarcs = makeContourTreeHyperarcs;
    }
    else if (computeRegularStructure == 2)
    {
      makeRegularStructureNodes = makeContourTreeNodes;
      makeRegularStructureArcs = makeContourTreeArcs;
      makeRegularStructureSuperparents = makeContourTreeSuperparents;
      makeRegularStructureSupernodes = makeContourTreeSupernodes;
      makeRegularStructureSuperarcs = makeContourTreeSuperarcs;

      makeRegularStructureAugmentnodesArr =
        new vtkm::Id[107]{ 0,  1,  2,  3,  4,   5,   6,   7,   8,   9,   10, 11, 12, 13, 14, 15,
                           16, 17, 18, 19, 20,  21,  22,  23,  24,  25,  26, 27, 28, 29, 30, 31,
                           32, 33, 34, 35, 36,  37,  38,  39,  40,  41,  42, 43, 44, 45, 46, 47,
                           48, 49, 50, 51, 52,  53,  54,  55,  56,  57,  58, 59, 60, 61, 62, 63,
                           64, 65, 66, 67, 68,  69,  70,  71,  72,  73,  74, 75, 76, 77, 78, 79,
                           80, 81, 82, 83, 84,  85,  86,  87,  88,  89,  90, 91, 92, 93, 94, 95,
                           96, 97, 98, 99, 106, 114, 116, 121, 122, 123, 124 };
      makeRegularStructureAugmentnodes =
        vtkm::cont::make_ArrayHandle(makeRegularStructureAugmentnodesArr, 107, vtkm::CopyFlag::On);

      makeRegularStructureAugmentarcsArr =
        new vtkm::Id[107]{ 1,  2,  3,  4,   5,  6,   7,   8,   9,   10,  11, 12, 13, 14, 15, 16,
                           17, 18, 19, 20,  21, 22,  23,  24,  25,  26,  27, 28, 29, 30, 31, 32,
                           33, 34, 35, 36,  37, 38,  39,  40,  41,  42,  43, 44, 45, 46, 47, 48,
                           49, 50, 51, 52,  53, 54,  55,  56,  57,  58,  59, 60, 61, 62, 63, 64,
                           65, 66, 67, 68,  69, 70,  71,  72,  73,  74,  75, 76, 77, 78, 79, 80,
                           81, 82, 83, 84,  85, 86,  87,  88,  89,  90,  91, 92, 93, 94, 95, 96,
                           97, 99, 99, 100, 0,  100, 100, 101, 101, 102, 102 };
      makeRegularStructureAugmentarcsArr[100] = makeRegularStructureAugmentarcsArr[100] |
        vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT;
      for (vtkm::Id i = 0; i < 100; i++)
      {
        makeRegularStructureAugmentarcsArr[i] = makeRegularStructureAugmentarcsArr[i] |
          vtkm::worklet::contourtree_augmented::IS_ASCENDING;
      }

      makeRegularStructureAugmentarcs =
        vtkm::cont::make_ArrayHandle(makeRegularStructureAugmentarcsArr, 107, vtkm::CopyFlag::On);


      makeRegularStructureHyperparents = makeContourTreeHyperparents;
      makeRegularStructureWhenTransferred = makeContourTreeWhenTransferred;
      makeRegularStructureHypernodes = makeContourTreeHypernodes;
      makeRegularStructureHyperarcs = makeContourTreeHyperarcs;
    }

    //
    // Setup the expected results object
    //
    ExpectedStepResults expectedResults(expectedSortOrder,
                                        expectedSortIndices,
                                        meshExtremaPeaksJoin,
                                        meshExtremaPitsJoin,
                                        meshExtremaPeaksBuildRegularChainsJoin,
                                        meshExtremaPitsBuildRegularChainsJoin,
                                        meshExtremaPeaksSplit,
                                        meshExtremaPitsSplit,
                                        meshExtremaPeaksBuildRegularChainsSplit,
                                        meshExtremaPitsBuildRegularChainsSplit,
                                        activeGraphJoinTreeInitGlobalIndex,
                                        activeGraphJoinTreeInitFirstEdge,
                                        activeGraphJoinTreeInitOutdegree,
                                        activeGraphJoinTreeInitHyperarcs,
                                        activeGraphJoinTreeInitActiveVertices,
                                        activeGraphJoinTreeInitEdgeNear,
                                        activeGraphJoinTreeInitEdgeFar,
                                        activeGraphJoinTreeInitActiveEdges,
                                        activeGraphSplitTreeInitGlobalIndex,
                                        activeGraphSplitTreeInitFirstEdge,
                                        activeGraphSplitTreeInitOutdegree,
                                        activeGraphSplitTreeInitHyperarcs,
                                        activeGraphSplitTreeInitActiveVertices,
                                        activeGraphSplitTreeInitEdgeNear,
                                        activeGraphSplitTreeInitEdgeFar,
                                        activeGraphSplitTreeInitActiveEdges,
                                        makeJoinTreeNumIterations,
                                        makeJoinTreeArcs,
                                        makeJoinTreeSuperparents,
                                        makeJoinTreeSupernodes,
                                        makeJoinTreeSuperarcs,
                                        makeJoinTreeHyperparents,
                                        makeJoinTreeHypernodes,
                                        makeJoinTreeHyperarcs,
                                        makeJoinTreeFirstSuperchild,
                                        makeSplitTreeNumIterations,
                                        makeSplitTreeArcs,
                                        makeSplitTreeSuperparents,
                                        makeSplitTreeSupernodes,
                                        makeSplitTreeSuperarcs,
                                        makeSplitTreeHyperparents,
                                        makeSplitTreeHypernodes,
                                        makeSplitTreeHyperarcs,
                                        makeSplitTreeFirstSuperchild,
                                        makeContourTreeNodes,
                                        makeContourTreeArcs,
                                        makeContourTreeSuperparents,
                                        makeContourTreeSupernodes,
                                        makeContourTreeSuperarcs,
                                        makeContourTreeAugmentnodes,
                                        makeContourTreeAugmentarcs,
                                        makeContourTreeHyperparents,
                                        makeContourTreeWhenTransferred,
                                        makeContourTreeHypernodes,
                                        makeContourTreeHyperarcs,
                                        makeRegularStructureNodes,
                                        makeRegularStructureArcs,
                                        makeRegularStructureSuperparents,
                                        makeRegularStructureSupernodes,
                                        makeRegularStructureSuperarcs,
                                        makeRegularStructureAugmentnodes,
                                        makeRegularStructureAugmentarcs,
                                        makeRegularStructureHyperparents,
                                        makeRegularStructureWhenTransferred,
                                        makeRegularStructureHypernodes,
                                        makeRegularStructureHyperarcs);

    //
    // Execute the test for the current settings
    //
    TestContourTreeAugmentedSteps3D(false,                   // don't use marchin cubes
                                    computeRegularStructure, // fully augment the tree
                                    expectedResults);

    //
    // Free temporary arrays allocated with new
    if (makeRegularStructureNodesArr != NULL)
    {
      delete[] makeRegularStructureNodesArr;
    }
    if (makeRegularStructureArcsArr != NULL)
    {
      delete[] makeRegularStructureArcsArr;
    }
    if (makeRegularStructureSuperparentsArr != NULL)
    {
      delete[] makeRegularStructureSuperparentsArr;
    }
    if (makeRegularStructureAugmentnodesArr != NULL)
    {
      delete[] makeRegularStructureAugmentnodesArr;
    }
    if (makeRegularStructureAugmentarcsArr != NULL)
    {
      delete[] makeRegularStructureAugmentarcsArr;
    }
  }


  void operator()() const
  {
    this->TestContourTree_Mesh2D_Freudenthal();
    this->TestContourTree_Mesh3D_Freudenthal();
    this->TestContourTree_Mesh3D_MarchingCubes();
    this->TestContourTreeAugmentedStepsFreudenthal3D(0); // without augmentation
    this->TestContourTreeAugmentedStepsFreudenthal3D(1); // with full augmentation
    this->TestContourTreeAugmentedStepsFreudenthal3D(2); // with full augmentation
  }
};
}

int UnitTestContourTreeUniformAugmented(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(TestContourTreeUniform(), argc, argv);
}
