//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
//  Copyright (c) 2016, Los Alamos National Security, LLC
//  All rights reserved.
//
//  Copyright 2016. Los Alamos National Security, LLC.
//  This software was produced under U.S. Government contract DE-AC52-06NA25396
//  for Los Alamos National Laboratory (LANL), which is operated by
//  Los Alamos National Security, LLC for the U.S. Department of Energy.
//  The U.S. Government has rights to use, reproduce, and distribute this
//  software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC
//  MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE
//  USE OF THIS SOFTWARE.  If software is modified to produce derivative works,
//  such modified software should be clearly marked, so as not to confuse it
//  with the version available from LANL.
//
//  Additionally, redistribution and use in source and binary forms, with or
//  without modification, are permitted provided that the following conditions
//  are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//  3. Neither the name of Los Alamos National Security, LLC, Los Alamos
//     National Laboratory, LANL, the U.S. Government, nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND
//  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
//  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS
//  NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
//  USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//============================================================================

//  This code is based on the algorithm presented in the paper:
//  “Parallel Peak Pruning for Scalable SMP Contour Tree Computation.”
//  Hamish Carr, Gunther Weber, Christopher Sewell, and James Ahrens.
//  Proceedings of the IEEE Symposium on Large Data Analysis and Visualization
//  (LDAV), October 2016, Baltimore, Maryland.

//=======================================================================================
//
// COMMENTS:
//
//
// i.e. based on PeakPitPruningCriticalSerial
//
// Under the old merge approach, we had an essentially breadth-first queue for transferring
// leaves from the merge trees to the contour tree.
//
// Most of these leaves are completely independent of each other, and can (on principle)
// be processed simultaneously.  However, the interior of the tree is dependent on them
// having been dealt with already. This version, therefore, will make multiple passes,
// in each pass pruning all maxima then all minima, interspersed with updating the merge
// and split trees. To understand this, consider what happens in the merge algorithm when
// a maximum is added:
//
// 1. The vertex v is removed from the queue: it has one join neighbour, w
// 2. Edge (v,w) is removed from the join tree, along with vertex v
// 3. Edge (v,w) is added to the contour tree, with v, w if necessary
// 4. Vertex v is removed from the split tree, bridging edges past it if necessary
// 5. Vertex w is added to the queue iff it is now a leaf
//
// To parallelise this:
// For all vertices v
// 		Set contourArc[v] = NO_VERTEX_ASSIGNED
// Set nContourArcs = 0;
// While (nContourArcs) > 0 // might be one, or something else - base case isn't clear
//	a.	Use reduction to compute updegree from join tree, downdegree from split tree
// 	b.	For each vertex v
//		// omit previously processed vertices
//		if (contourArc[v] == NO_VERTEX_ASSIGNED)
//			continue;
//		// Test for extremality
//		i.	If ((updegree[v] == 0) && (downdegree[v] == 1))
//			{ // Maximum
//			contourArc[v] = joinArc[v];
//			} // Maximum
//		ii.	Else if ((updegree[v] = 1) && (downdegree[v] == 0))
//			{ // Minimum
//			contourArc[v] = splitArc[v];
//			} // Minimum
//	c.	For (log n iterations)
//		i.	For each vertex v
//			retrieve it's join neighbour j
//			retrieve it's split neighbour s
//			if v has no join neighbour (i.e. j == -1)
//				skip (i.e. v is the root)
//			else if j has a contour arc assigned
//				set v's neighbour to j's neighbour
//			if v has no split neighbour (i.e. s == -1)
//				skip (i.e. v is the root)
//			else if s has a contour arc assigned
//				set v's neighbour to s's neighbour
//
// Initially, we will do this with all vertices, regular or otherwise, then restrict to
// the critical points. Number of iterations - regular vertices will slow this down, so
// the worst case is O(n) passes.  Even if we restrict to critical points, W's in the tree
// will serialise, so O(n) still applies. I believe that the W edges can be suppressed,
// but let's leave that to optimisation for now.
//
//=======================================================================================

#ifndef vtkm_worklet_contourtree_contourtree_h
#define vtkm_worklet_contourtree_contourtree_h

// local includes
#include <vtkm/worklet/contourtree/ChainGraph.h>
#include <vtkm/worklet/contourtree/CopyJoinSplit.h>
#include <vtkm/worklet/contourtree/CopyNeighbors.h>
#include <vtkm/worklet/contourtree/CopySupernodes.h>
#include <vtkm/worklet/contourtree/DegreeDelta.h>
#include <vtkm/worklet/contourtree/DegreeSubrangeOffset.h>
#include <vtkm/worklet/contourtree/FillSupernodes.h>
#include <vtkm/worklet/contourtree/FindLeaves.h>
#include <vtkm/worklet/contourtree/MergeTree.h>
#include <vtkm/worklet/contourtree/PrintVectors.h>
#include <vtkm/worklet/contourtree/RegularToCandidate.h>
#include <vtkm/worklet/contourtree/RegularToCriticalDown.h>
#include <vtkm/worklet/contourtree/RegularToCriticalUp.h>
#include <vtkm/worklet/contourtree/ResetDegrees.h>
#include <vtkm/worklet/contourtree/SetJoinAndSplitArcs.h>
#include <vtkm/worklet/contourtree/SetSupernodeInward.h>
#include <vtkm/worklet/contourtree/SkipVertex.h>
#include <vtkm/worklet/contourtree/SubrangeOffset.h>
#include <vtkm/worklet/contourtree/Types.h>
#include <vtkm/worklet/contourtree/UpdateOutbound.h>

#include <vtkm/Pair.h>
#include <vtkm/cont/ArrayGetValues.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/ArrayHandleConcatenate.h>
#include <vtkm/cont/ArrayHandlePermutation.h>
#include <vtkm/worklet/WorkletMapField.h>

//#define DEBUG_PRINT 1
//#define DEBUG_TIMING 1

namespace vtkm
{
namespace worklet
{
namespace contourtree
{

template <typename T, typename StorageType>
class ContourTree
{
public:
  using IdArrayType = vtkm::cont::ArrayHandle<vtkm::Id>;
  using ValueArrayType = vtkm::cont::ArrayHandle<T>;
  using PermuteIndexType = vtkm::cont::ArrayHandlePermutation<IdArrayType, IdArrayType>;
  using PermuteValueType = vtkm::cont::ArrayHandlePermutation<IdArrayType, ValueArrayType>;

  // reference to the underlying data
  const vtkm::cont::ArrayHandle<T, StorageType> values;

  // vector of superarcs in the contour tree (stored as inward-pointing)
  vtkm::cont::ArrayHandle<vtkm::Id> superarcs;

  // vector of supernodes
  vtkm::cont::ArrayHandle<vtkm::Id> supernodes;

  // vector of supernodes still unprocessed
  vtkm::cont::ArrayHandle<vtkm::Id> activeSupernodes;

  // references to join & split trees
  MergeTree<T, StorageType>&joinTree, &splitTree;

  // references to join & split graphs
  ChainGraph<T, StorageType>&joinGraph, &splitGraph;

  // vectors of up & down degree used during computation
  vtkm::cont::ArrayHandle<vtkm::Id> updegree, downdegree;

  // vectors for tracking merge arcs
  vtkm::cont::ArrayHandle<vtkm::Id> joinArcs, splitArcs;

  // counter for how many iterations it took to compute
  vtkm::Id nIterations;

  // contour tree constructor
  ContourTree(const vtkm::cont::ArrayHandle<T, StorageType>& Values,
              MergeTree<T, StorageType>& JoinTree,
              MergeTree<T, StorageType>& SplitTree,
              ChainGraph<T, StorageType>& JoinGraph,
              ChainGraph<T, StorageType>& SplitGraph);

  // routines for computing the contour tree
  // combines the list of active vertices for join & split trees
  // then reduces them to eliminate regular vertices & non-connectivity critical points
  void FindSupernodes();

  // transfers leaves from join/split trees to contour tree
  void TransferLeaves();

  // collapses regular edges along leaf superarcs
  void CollapseRegular(bool isJoin);

  // compresses trees to remove transferred vertices
  void CompressTrees();

  // compresses active set of supernodes
  void CompressActiveSupernodes();

  // finds the degree of each supernode from the join & split trees
  void FindDegrees();

  // collect the resulting saddle peaks in sort pairs
  void CollectSaddlePeak(vtkm::cont::ArrayHandle<vtkm::Pair<vtkm::Id, vtkm::Id>>& saddlePeak);

  void DebugPrint(const char* message);

}; // class ContourTree

struct VertexAssigned : vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(FieldIn supernode, WholeArrayIn superarcs, FieldOut hasSuperArc);
  using ExecutionSignature = _3(_1, _2);
  using InputDomain = _1;

  bool vertexIsAssigned;

  VTKM_EXEC_CONT
  VertexAssigned(bool VertexIsAssigned)
    : vertexIsAssigned(VertexIsAssigned)
  {
  }

  template <typename InPortalFieldType>
  VTKM_EXEC vtkm::Id operator()(const vtkm::Id supernode, const InPortalFieldType& superarcs) const
  {
    if (vertexIsAssigned == false)
    {
      if (superarcs.Get(supernode) == NO_VERTEX_ASSIGNED)
        return vtkm::Id(1);
      else
        return vtkm::Id(0);
    }
    else
    {
      if (superarcs.Get(supernode) != NO_VERTEX_ASSIGNED)
        return vtkm::Id(1);
      else
        return vtkm::Id(0);
    }
  }
};

// creates contour tree
template <typename T, typename StorageType>
ContourTree<T, StorageType>::ContourTree(const vtkm::cont::ArrayHandle<T, StorageType>& Values,
                                         MergeTree<T, StorageType>& JoinTree,
                                         MergeTree<T, StorageType>& SplitTree,
                                         ChainGraph<T, StorageType>& JoinGraph,
                                         ChainGraph<T, StorageType>& SplitGraph)
  : values(Values)
  , joinTree(JoinTree)
  , splitTree(SplitTree)
  , joinGraph(JoinGraph)
  , splitGraph(SplitGraph)
{

  // first we have to get the correct list of supernodes
  // this will also set the degrees of the vertices initially
  FindSupernodes();

  // and track how many iterations it takes
  nIterations = 0;

  // loop until no arcs remaining to be found
  // tree can end with either 0 or 1 vertices unprocessed
  // 0 means the last edge was pruned from both ends
  // 1 means that there were two final edges meeting at a vertex

  while (activeSupernodes.GetNumberOfValues() > 1)
  { // loop until no active vertices remaining
#ifdef DEBUG_PRINT
    std::cout << "========================================" << std::endl;
    std::cout << "                                        " << std::endl;
    std::cout << "Iteration " << nIterations << " Size " << activeSupernodes.GetNumberOfValues()
              << std::endl;
    std::cout << "                                        " << std::endl;
    std::cout << "========================================" << std::endl;
#endif

    // transfer all leaves to the contour tree
    TransferLeaves();

    // collapse regular vertices from leaves, upper then lower
    CollapseRegular(true);
    CollapseRegular(false);

    // compress the join and split trees
    CompressTrees();

    // compress the active list of supernodes
    CompressActiveSupernodes();

    // recompute the vertex degrees
    FindDegrees();

    nIterations++;
  }
} // constructor

// combines the list of active vertices for join & split trees
// then reduces them to eliminate regular vertices & non-connectivity critical points
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::FindSupernodes()
{
  // both trees may have non-connectivity critical points, so we first make a joint list
  // here, we will explicitly assume that the active lists are in numerical order
  // which is how we are currently constructing them
  vtkm::Id nCandidates =
    joinGraph.valueIndex.GetNumberOfValues() + splitGraph.valueIndex.GetNumberOfValues();
  vtkm::cont::ArrayHandle<vtkm::Id> candidates;

  // take the union of the two sets of vertices
  vtkm::cont::ArrayHandleConcatenate<IdArrayType, IdArrayType> candidateArray(
    joinGraph.valueIndex, splitGraph.valueIndex);
  vtkm::cont::Algorithm::Copy(candidateArray, candidates);
  vtkm::cont::Algorithm::Sort(candidates);
  vtkm::cont::Algorithm::Unique(candidates);

  nCandidates = candidates.GetNumberOfValues();
  vtkm::cont::ArrayHandleIndex candidateIndexArray(nCandidates);

  // we need an array lookup to convert vertex ID's
  vtkm::Id nValues = values.GetNumberOfValues();
  vtkm::cont::ArrayHandle<vtkm::Id> regularToCritical;
  vtkm::cont::ArrayHandleConstant<vtkm::Id> noVertArray(NO_VERTEX_ASSIGNED, nValues);
  vtkm::cont::Algorithm::Copy(noVertArray, regularToCritical);

  if (nCandidates > 0)
  {
    RegularToCriticalUp regularToCriticalUp;
    vtkm::worklet::DispatcherMapField<RegularToCriticalUp> regularToCriticalUpDispatcher(
      regularToCriticalUp);

    regularToCriticalUpDispatcher.Invoke(candidateIndexArray, // input
                                         candidates,          // input
                                         regularToCritical);  // output (whole array)
  }

  // now that we have a complete list of active nodes from each, we can call the trees
  // to connect them properly
  joinTree.ComputeAugmentedSuperarcs();
  joinTree.ComputeAugmentedArcs(candidates);
  splitTree.ComputeAugmentedSuperarcs();
  splitTree.ComputeAugmentedArcs(candidates);

  // we create up & down degree arrays
  vtkm::cont::ArrayHandleConstant<vtkm::Id> initCandidateArray(0, nCandidates);
  vtkm::cont::ArrayHandle<vtkm::Id> upCandidate;
  vtkm::cont::ArrayHandle<vtkm::Id> downCandidate;
  vtkm::cont::Algorithm::Copy(initCandidateArray, upCandidate);
  vtkm::cont::Algorithm::Copy(initCandidateArray, downCandidate);

  // This next chunk changes in parallel - it has to count the up & down degree for each
  // vertex. It's a simple loop in serial, but in parallel, what we have to do is:
  //	1. Copy the lower ends of the edges, converting from regular ID to candidate ID
  //	2. Sort the lower ends of the edges
  //	3. For each value, store the beginning of the range
  //	4. Compute the delta to get the degree.

  // create a sorting vector
  vtkm::cont::ArrayHandle<vtkm::Id> sortVector;
  sortVector.Allocate(nCandidates);

  // 1. Copy the lower ends of the edges, converting from regular ID to candidate ID
  if (nCandidates > 0)
  {
    RegularToCandidate regularToCandidate;
    vtkm::worklet::DispatcherMapField<RegularToCandidate> regularToCandidateDispatcher(
      regularToCandidate);

    regularToCandidateDispatcher.Invoke(candidates,         // input
                                        joinTree.mergeArcs, // input (whole array)
                                        regularToCritical,  // input (whole array)
                                        sortVector);        // output
  }

  // 2. Sort the lower ends of the edges
  vtkm::cont::Algorithm::Sort(sortVector);

  // 3. For each value, store the beginning & end of the range (in parallel)
  //	The 0th element is guaranteed to be NO_VERTEX_ASSIGNED, & can be skipped
  //	Otherwise, if the i-1th element is different, we are the offset for the subrange
  // 	and store into the ith place
  vtkm::cont::ArrayHandleCounting<vtkm::Id> subsetIndexArray(1, 1, nCandidates - 1);
  if (nCandidates > 0)
  {
    SubrangeOffset subRangeOffset;
    vtkm::worklet::DispatcherMapField<SubrangeOffset> subrangeOffsetDispatcher(subRangeOffset);

    subrangeOffsetDispatcher.Invoke(subsetIndexArray, // index
                                    sortVector,       // input
                                    upCandidate);     // output
  }

  // 4. Compute the delta to get the degree.
  if (nCandidates > 0)
  {
    DegreeDelta degreeDelta(nCandidates);
    vtkm::worklet::DispatcherMapField<DegreeDelta> degreeDeltaDispatcher(degreeDelta);

    degreeDeltaDispatcher.Invoke(subsetIndexArray, // input
                                 sortVector,       // input (whole array)
                                 upCandidate);     // output (whole array)
  }

  // Now repeat the same steps for the downdegree
  // 1. Copy the upper ends of the edges, converting from regular ID to candidate ID
  if (nCandidates > 0)
  {
    RegularToCriticalDown regularToCriticalDown;
    vtkm::worklet::DispatcherMapField<RegularToCriticalDown> regularToCriticalDownDispatcher(
      regularToCriticalDown);

    regularToCriticalDownDispatcher.Invoke(candidates,          // input
                                           splitTree.mergeArcs, // input (whole array)
                                           regularToCritical,   // input (whole array)
                                           sortVector);         // output
  }

  // 2. Sort the lower ends of the edges
  vtkm::cont::Algorithm::Sort(sortVector);

  // 3. For each value, store the beginning & end of the range (in parallel)
  //	The 0th element is guaranteed to be NO_VERTEX_ASSIGNED, & can be skipped
  //	Otherwise, if the i-1th element is different, we are the offset for the subrange
  // 	and store into the ith place
  if (nCandidates > 0)
  {
    SubrangeOffset subRangeOffset;
    vtkm::worklet::DispatcherMapField<SubrangeOffset> subrangeOffsetDispatcher(subRangeOffset);

    subrangeOffsetDispatcher.Invoke(subsetIndexArray, // index
                                    sortVector,       // input
                                    downCandidate);   // output
  }

  // 4. Compute the delta to get the degree.
  if (nCandidates > 0)
  {
    DegreeDelta degreeDelta(nCandidates);
    vtkm::worklet::DispatcherMapField<DegreeDelta> degreeDeltaDispatcher(degreeDelta);

    degreeDeltaDispatcher.Invoke(subsetIndexArray, // index
                                 sortVector,       // input
                                 downCandidate);   // in out
  }

  // create an index vector for whether the vertex is to be kept
  vtkm::cont::ArrayHandle<vtkm::Id> isSupernode;
  isSupernode.Allocate(nCandidates);

  // fill the vector in
  if (nCandidates > 0)
  {
    FillSupernodes fillSupernodes;
    vtkm::worklet::DispatcherMapField<FillSupernodes> fillSupernodesDispatcher(fillSupernodes);

    fillSupernodesDispatcher.Invoke(upCandidate,   // input
                                    downCandidate, // input
                                    isSupernode);  // output
  }

  // do a compaction to find the new index for each
  // We end with 0 in position 0, and need one extra position to find the new size
  vtkm::cont::ArrayHandle<vtkm::Id> supernodeID;
  vtkm::cont::Algorithm::ScanExclusive(isSupernode, supernodeID);

  // size is the position of the last element + the size of the last element (0/1)
  vtkm::Id nSupernodes = vtkm::cont::ArrayGetValue(nCandidates - 1, supernodeID) +
    vtkm::cont::ArrayGetValue(nCandidates - 1, isSupernode);

  // allocate memory for our arrays
  supernodes.ReleaseResources();
  updegree.ReleaseResources();
  downdegree.ReleaseResources();

  supernodes.Allocate(nSupernodes);
  updegree.Allocate(nSupernodes);
  downdegree.Allocate(nSupernodes);

  // now copy over the positions to compact
  if (nCandidates > 0)
  {
    CopySupernodes copySupernodes;
    vtkm::worklet::DispatcherMapField<CopySupernodes> copySupernodesDispatcher(copySupernodes);

    copySupernodesDispatcher.Invoke(isSupernode,       // input
                                    candidates,        // input
                                    supernodeID,       // input
                                    upCandidate,       // input
                                    downCandidate,     // input
                                    regularToCritical, // output (whole array)
                                    supernodes,        // output (whole array)
                                    updegree,          // output (whole array)
                                    downdegree);       // output (whole array)
  }

  // now we call the merge tree again to reset the merge arcs
  joinTree.ComputeAugmentedArcs(supernodes);
  splitTree.ComputeAugmentedArcs(supernodes);

  // next we create the working arrays of merge arcs
  nSupernodes = supernodes.GetNumberOfValues();
  vtkm::cont::ArrayHandleIndex supernodeIndexArray(nSupernodes);
  joinArcs.ReleaseResources();
  splitArcs.ReleaseResources();
  joinArcs.Allocate(nSupernodes);
  splitArcs.Allocate(nSupernodes);

  // and copy them across, setting IDs for both ends
  SetJoinAndSplitArcs setJoinAndSplitArcs;
  vtkm::worklet::DispatcherMapField<SetJoinAndSplitArcs> setJoinAndSplitArcsDispatcher(
    setJoinAndSplitArcs);

  setJoinAndSplitArcsDispatcher.Invoke(supernodes,          // input
                                       joinTree.mergeArcs,  // input (whole array)
                                       splitTree.mergeArcs, // input (whole array)
                                       regularToCritical,   // input (whole array)
                                       joinArcs,            // output
                                       splitArcs);          // output

  vtkm::cont::ArrayHandleConstant<vtkm::Id> newsuperarcs(NO_VERTEX_ASSIGNED, nSupernodes);
  superarcs.ReleaseResources();
  vtkm::cont::Algorithm::Copy(newsuperarcs, superarcs);

  // create the active supernode vector
  activeSupernodes.ReleaseResources();
  activeSupernodes.Allocate(nSupernodes);
  vtkm::cont::ArrayHandleIndex supernodeSeq(nSupernodes);
  vtkm::cont::Algorithm::Copy(supernodeSeq, activeSupernodes);

#ifdef DEBUG_PRINT
  DebugPrint("Supernodes Found");
#endif
} // FindSupernodes()

// transfers leaves from join/split trees to contour tree
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::TransferLeaves()
{
  FindLeaves findLeaves;
  vtkm::worklet::DispatcherMapField<FindLeaves> findLeavesDispatcher(findLeaves);

  findLeavesDispatcher.Invoke(activeSupernodes, // input
                              updegree,         // input (whole array)
                              downdegree,       // input (whole array)
                              joinArcs,         // input (whole array)
                              splitArcs,        // input (whole array)
                              superarcs);       // i/o (whole array)
#ifdef DEBUG_PRINT
  DebugPrint("Leaves Transferred");
#endif
} // TransferLeaves()

// collapses regular edges along leaf superarcs
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::CollapseRegular(bool isJoin)
{
  // we'll have a vector for tracking outwards
  vtkm::Id nSupernodes = supernodes.GetNumberOfValues();
  vtkm::cont::ArrayHandleConstant<vtkm::Id> nullArray(0, nSupernodes);
  vtkm::cont::ArrayHandle<vtkm::Id> outbound;
  outbound.Allocate(nSupernodes);
  vtkm::cont::ArrayCopy(nullArray, outbound);

  // and a reference for the inwards array and to the degrees
  vtkm::cont::ArrayHandle<vtkm::Id> inbound;
  vtkm::cont::ArrayHandle<vtkm::Id> indegree;
  vtkm::cont::ArrayHandle<vtkm::Id> outdegree;
  if (isJoin)
  {
    vtkm::cont::ArrayCopy(joinArcs, inbound);
    vtkm::cont::ArrayCopy(downdegree, indegree);
    vtkm::cont::ArrayCopy(updegree, outdegree);
  }
  else
  {
    vtkm::cont::ArrayCopy(splitArcs, inbound);
    vtkm::cont::ArrayCopy(updegree, indegree);
    vtkm::cont::ArrayCopy(downdegree, outdegree);
  }

  // loop to copy join/split
  CopyJoinSplit copyJoinSplit;
  vtkm::worklet::DispatcherMapField<CopyJoinSplit> copyJoinSplitDispatcher(copyJoinSplit);

  copyJoinSplitDispatcher.Invoke(activeSupernodes, // input
                                 inbound,          // input (whole array)
                                 indegree,         // input (whole array)
                                 outdegree,        // input (whole array)
                                 outbound);        // output (whole array)

  // Compute the number of log steps required in this pass
  vtkm::Id nLogSteps = 1;
  vtkm::Id nActiveSupernodes = activeSupernodes.GetNumberOfValues();
  for (vtkm::Id shifter = nActiveSupernodes; shifter != 0; shifter >>= 1)
    nLogSteps++;

  // loop to find the now-regular vertices and collapse past them without altering
  // the existing join & split arcs
  for (vtkm::Id iteration = 0; iteration < nLogSteps; iteration++)
  {
    UpdateOutbound updateOutbound;
    vtkm::worklet::DispatcherMapField<UpdateOutbound> updateOutboundDispatcher(updateOutbound);

    updateOutboundDispatcher.Invoke(activeSupernodes, // input
                                    outbound);        // i/o (whole array)
  }

  // at this point, the outbound vector chains everything outwards to the leaf
  // any vertices on the last outbound leaf superarc point to the leaf

  // Now, any regular leaf vertex points out to a leaf, so the condition we test is
  // a. outbound is not -1 (i.e. vertex is regular)
  // b. superarc[outbound] is not -1 (i.e. outbound is a leaf)
  SetSupernodeInward setSupernodeInward;
  vtkm::worklet::DispatcherMapField<SetSupernodeInward> setSupernodeInwardDispatcher(
    setSupernodeInward);

  setSupernodeInwardDispatcher.Invoke(activeSupernodes, // input
                                      inbound,          // input (whole array)
                                      outbound,         // input (whole array)
                                      indegree,         // input (whole array)
                                      outdegree,        // input (whole array)
                                      superarcs);       // i/o   (whole array)
  outbound.ReleaseResources();

#ifdef DEBUG_PRINT
  DebugPrint(isJoin ? "Upper Regular Nodes Collapsed" : "Lower Regular Nodes Collapsed");
#endif
} // CollapseRegular()

// compresses trees to remove transferred vertices
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::CompressTrees()
{
  // Compute the number of log steps required in this pass
  vtkm::Id nActiveSupernodes = activeSupernodes.GetNumberOfValues();
  vtkm::Id nLogSteps = 1;
  for (vtkm::Id shifter = nActiveSupernodes; shifter != 0; shifter >>= 1)
    nLogSteps++;

  // loop to update the merge trees
  for (vtkm::Id logStep = 0; logStep < nLogSteps; logStep++)
  {
    SkipVertex skipVertex;
    vtkm::worklet::DispatcherMapField<SkipVertex> skipVertexDispatcher(skipVertex);

    skipVertexDispatcher.Invoke(activeSupernodes, // input
                                superarcs,        // input (whole array)
                                joinArcs,         // i/o (whole array)
                                splitArcs);       // i/o (whole array)
  }

#ifdef DEBUG_PRINT
  DebugPrint("Trees Compressed");
#endif
} // CompressTrees()

// compresses active set of supernodes
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::CompressActiveSupernodes()
{
  // copy only if the superarc is not set
  vtkm::cont::ArrayHandle<vtkm::Id> noSuperarcArray;
  noSuperarcArray.Allocate(activeSupernodes.GetNumberOfValues());

  VertexAssigned vertexAssigned(false);
  vtkm::worklet::DispatcherMapField<VertexAssigned> vertexAssignedDispatcher(vertexAssigned);

  vertexAssignedDispatcher.Invoke(activeSupernodes, superarcs, noSuperarcArray);

  vtkm::cont::ArrayHandle<vtkm::Id> compressSupernodes;
  vtkm::cont::Algorithm::CopyIf(activeSupernodes, noSuperarcArray, compressSupernodes);

  activeSupernodes.ReleaseResources();
  vtkm::cont::ArrayCopy(compressSupernodes, activeSupernodes);

#ifdef DEBUG_PRINT
  DebugPrint("Active Supernodes Compressed");
#endif
} // CompressActiveSupernodes()

// recomputes the degree of each supernode from the join & split trees
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::FindDegrees()
{
  if (activeSupernodes.GetNumberOfValues() == 0)
    return;

  vtkm::Id nActiveSupernodes = activeSupernodes.GetNumberOfValues();
  ResetDegrees resetDegrees;
  vtkm::worklet::DispatcherMapField<ResetDegrees> resetDegreesDispatcher(resetDegrees);

  resetDegreesDispatcher.Invoke(activeSupernodes, // input
                                updegree,         // output (whole array)
                                downdegree);      // output (whole array)

  // create a temporary sorting array
  vtkm::cont::ArrayHandle<vtkm::Id> sortVector;
  sortVector.Allocate(nActiveSupernodes);
  vtkm::cont::ArrayHandleIndex activeSupernodeIndexArray(nActiveSupernodes);

  // 1. Copy the neighbours for each active edge
  if (nActiveSupernodes > 0)
  {
    CopyNeighbors copyNeighbors;
    vtkm::worklet::DispatcherMapField<CopyNeighbors> copyNeighborsDispatcher(copyNeighbors);

    copyNeighborsDispatcher.Invoke(activeSupernodeIndexArray, // input
                                   activeSupernodes,          // input (whole array)
                                   joinArcs,                  // input (whole array)
                                   sortVector);               // output
  }

  // 2. Sort the neighbours
  vtkm::cont::Algorithm::Sort(sortVector);

  // 3. For each value, store the beginning & end of the range (in parallel)
  //	The 0th element is guaranteed to be NO_VERTEX_ASSIGNED, & can be skipped
  //	Otherwise, if the i-1th element is different, we are the offset for the subrange
  // 	and store into the ith place
  vtkm::cont::ArrayHandleCounting<vtkm::Id> subsetIndexArray(1, 1, nActiveSupernodes - 1);
  if (nActiveSupernodes > 1)
  {
    DegreeSubrangeOffset degreeSubrangeOffset;
    vtkm::worklet::DispatcherMapField<DegreeSubrangeOffset> degreeSubrangeOffsetDispatcher(
      degreeSubrangeOffset);

    degreeSubrangeOffsetDispatcher.Invoke(subsetIndexArray, // input
                                          sortVector,       // input (whole array)
                                          updegree);        // output (whole array)
  }

  // 4. Compute the delta to get the degree.
  if (nActiveSupernodes > 1)
  {
    DegreeDelta degreeDelta(nActiveSupernodes);
    vtkm::worklet::DispatcherMapField<DegreeDelta> degreeDeltaDispatcher(degreeDelta);

    degreeDeltaDispatcher.Invoke(subsetIndexArray, // input
                                 sortVector,       // input
                                 updegree);        // in out
  }

  // Now repeat the same steps for the downdegree
  // 1. Copy the neighbours for each active edge
  if (nActiveSupernodes > 0)
  {
    CopyNeighbors copyNeighbors;
    vtkm::worklet::DispatcherMapField<CopyNeighbors> copyNeighborsDispatcher(copyNeighbors);

    copyNeighborsDispatcher.Invoke(activeSupernodeIndexArray, // input
                                   activeSupernodes,          // input (whole array)
                                   splitArcs,                 // input (whole array)
                                   sortVector);               // output
  }

  // 2. Sort the neighbours
  vtkm::cont::Algorithm::Sort(sortVector);

  // 3. For each value, store the beginning & end of the range (in parallel)
  //	The 0th element is guaranteed to be NO_VERTEX_ASSIGNED, & can be skipped
  //	Otherwise, if the i-1th element is different, we are the offset for the subrange
  // 	and store into the ith place
  if (nActiveSupernodes > 1)
  {
    DegreeSubrangeOffset degreeSubrangeOffset;
    vtkm::worklet::DispatcherMapField<DegreeSubrangeOffset> degreeSubrangeOffsetDispatcher(
      degreeSubrangeOffset);

    degreeSubrangeOffsetDispatcher.Invoke(subsetIndexArray, // input
                                          sortVector,       // input (whole array)
                                          downdegree);      // output (whole array)
  }

  // 4. Compute the delta to get the degree.
  if (nActiveSupernodes > 1)
  {
    DegreeDelta degreeDelta(nActiveSupernodes);
    vtkm::worklet::DispatcherMapField<DegreeDelta> degreeDeltaDispatcher(degreeDelta);

    degreeDeltaDispatcher.Invoke(subsetIndexArray, // input
                                 sortVector,       // input (whole array)
                                 downdegree);      // in out (whole array)
  }
#ifdef DEBUG_PRINT
  DebugPrint("Degrees Recomputed");
#endif
} // FindDegrees()

// small class for storing the contour arcs
class EdgePair
{
public:
  vtkm::Id low, high;

  // constructor - defaults to -1
  EdgePair(vtkm::Id Low = -1, vtkm::Id High = -1)
    : low(Low)
    , high(High)
  {
  }
};

// comparison operator <
bool operator<(const EdgePair LHS, const EdgePair RHS)
{
  if (LHS.low < RHS.low)
    return true;
  if (LHS.low > RHS.low)
    return false;
  if (LHS.high < RHS.high)
    return true;
  if (LHS.high > RHS.high)
    return false;
  return false;
}

struct SaddlePeakSort
{
  VTKM_EXEC_CONT bool operator()(const vtkm::Pair<vtkm::Id, vtkm::Id>& a,
                                 const vtkm::Pair<vtkm::Id, vtkm::Id>& b) const
  {
    if (a.first < b.first)
      return true;
    if (a.first > b.first)
      return false;
    if (a.second < b.second)
      return true;
    if (a.second > b.second)
      return false;
    return false;
  }
};

// sorted print routine
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::CollectSaddlePeak(
  vtkm::cont::ArrayHandle<vtkm::Pair<vtkm::Id, vtkm::Id>>& saddlePeak)
{
  // Collect the valid saddle peak pairs
  std::vector<vtkm::Pair<vtkm::Id, vtkm::Id>> superarcVector;
  const auto supernodePortal = supernodes.ReadPortal();
  const auto superarcPortal = superarcs.ReadPortal();
  for (vtkm::Id supernode = 0; supernode < supernodes.GetNumberOfValues(); supernode++)
  {
    // ID of regular node
    const vtkm::Id regularID = supernodePortal.Get(supernode);

    // retrieve ID of target supernode
    const vtkm::Id superTo = superarcPortal.Get(supernode);
    ;

    // if this is true, it is the last pruned vertex
    if (superTo == NO_VERTEX_ASSIGNED)
    {
      continue;
    }

    // retrieve the regular ID for it
    const vtkm::Id regularTo = supernodePortal.Get(superTo);

    // how we print depends on which end has lower ID
    if (regularID < regularTo)
    { // from is lower
      // extra test to catch duplicate edge
      if (superarcPortal.Get(superTo) != supernode)
      {
        superarcVector.push_back(vtkm::make_Pair(regularID, regularTo));
      }
    } // from is lower
    else
    {
      superarcVector.push_back(vtkm::make_Pair(regularTo, regularID));
    }
  } // per vertex

  // Setting saddlePeak reference to the make_ArrayHandle directly does not work
  vtkm::cont::ArrayHandle<vtkm::Pair<vtkm::Id, vtkm::Id>> tempArray =
    vtkm::cont::make_ArrayHandle(superarcVector, vtkm::CopyFlag::Off);

  // now sort it
  vtkm::cont::Algorithm::Sort(tempArray, SaddlePeakSort());
  vtkm::cont::Algorithm::Copy(tempArray, saddlePeak);

#ifdef DEBUG_PRINT
  const vtkm::Id arcVecSize = static_cast<vtkm::Id>(superarcVector.size());
  for (vtkm::Id superarc = 0; superarc < arcVecSize; superarc++)
  {
    std::cout << std::setw(PRINT_WIDTH) << saddlePeak.WritePortal().Get(superarc).first << " ";
    std::cout << std::setw(PRINT_WIDTH) << saddlePeak.WritePortal().Get(superarc).second
              << std::endl;
  }
#endif
} // CollectSaddlePeak()

// debug routine
template <typename T, typename StorageType>
void ContourTree<T, StorageType>::DebugPrint(const char* message)
{
  std::cout << "---------------------------" << std::endl;
  std::cout << std::string(message) << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl;

  // print out the supernode arrays
  vtkm::Id nSupernodes = supernodes.GetNumberOfValues();
  PrintHeader(nSupernodes);

  PrintIndices("Supernodes", supernodes);

  vtkm::cont::ArrayHandle<vtkm::Id> supervalues;
  vtkm::cont::ArrayCopy(PermuteValueType(supernodes, values), supervalues);
  PrintValues("Value", supervalues);

  PrintIndices("Up degree", updegree);
  PrintIndices("Down degree", downdegree);
  PrintIndices("Join arc", joinArcs);
  PrintIndices("Split arc", splitArcs);
  PrintIndices("Superarcs", superarcs);
  std::cout << std::endl;

  // print out the active supernodes
  vtkm::Id nActiveSupernodes = activeSupernodes.GetNumberOfValues();
  PrintHeader(nActiveSupernodes);

  PrintIndices("Active Supernodes", activeSupernodes);

  vtkm::cont::ArrayHandle<vtkm::Id> activeUpdegree;
  vtkm::cont::ArrayCopy(PermuteIndexType(activeSupernodes, updegree), activeUpdegree);
  PrintIndices("Active Up Degree", activeUpdegree);

  vtkm::cont::ArrayHandle<vtkm::Id> activeDowndegree;
  vtkm::cont::ArrayCopy(PermuteIndexType(activeSupernodes, downdegree), activeDowndegree);
  PrintIndices("Active Down Degree", activeDowndegree);

  vtkm::cont::ArrayHandle<vtkm::Id> activeJoinArcs;
  vtkm::cont::ArrayCopy(PermuteIndexType(activeSupernodes, joinArcs), activeJoinArcs);
  PrintIndices("Active Join Arcs", activeJoinArcs);

  vtkm::cont::ArrayHandle<vtkm::Id> activeSplitArcs;
  vtkm::cont::ArrayCopy(PermuteIndexType(activeSupernodes, splitArcs), activeSplitArcs);
  PrintIndices("Active Split Arcs", activeSplitArcs);

  vtkm::cont::ArrayHandle<vtkm::Id> activeSuperarcs;
  vtkm::cont::ArrayCopy(PermuteIndexType(activeSupernodes, superarcs), activeSuperarcs);
  PrintIndices("Active Superarcs", activeSuperarcs);
  std::cout << std::endl;
} // DebugPrint()
}
}
}

#endif
