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

#ifndef vtk_m_worklet_contourtree_augmented_contourtree_maker_inc_augment_merge_tree_set_augmented_merge_arcs_h
#define vtk_m_worklet_contourtree_augmented_contourtree_maker_inc_augment_merge_tree_set_augmented_merge_arcs_h

#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree_augmented
{
namespace contourtree_maker_inc
{

// Worklet for computing the augemented merge join/split tree arcs needed for computing the contour tree
class AugmentMergeTrees_SetAugmentedMergeArcs : public vtkm::worklet::WorkletMapField
{
public:
  typedef void ControlSignature(
    WholeArrayIn contourTreeActiveSupernodes,   // (input) active supernodes from the contour tree
    WholeArrayIn mergetreeSuperparents,         // (input)
    WholeArrayIn mergetreeSuperarcs,            // (input)
    WholeArrayIn newMergetreeID,                // (input)
    WholeArrayOut augmentedMergetreeSuperarcs); // (output)
  typedef void ExecutionSignature(_1, InputIndex, _2, _3, _4, _5);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  AugmentMergeTrees_SetAugmentedMergeArcs() {}

  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC void operator()(const InFieldPortalType& activeSupernodesPortal,
                            const vtkm::Id supernode,
                            const InFieldPortalType& mergetreeSuperparentsPortal,
                            const InFieldPortalType& mergetreeSuperarcsPortal,
                            const InFieldPortalType& newMergetreeIDPortal,
                            const OutFieldPortalType& augmentedMergetreeSuperarcsPortal) const
  {
    vtkm::Id supernodeID = activeSupernodesPortal.Get(supernode);
    vtkm::Id mergetreeSuperparent = mergetreeSuperparentsPortal.Get(supernodeID);
    // work out whether we're the "lowest" in the group
    bool lastMergetreeSupernode = false;
    // the 0'th one is always the last
    if (supernode == 0)
    {
      lastMergetreeSupernode = true;
    }
    // otherwise, check whether it's the same as the adjacent one
    else
    {
      lastMergetreeSupernode =
        (mergetreeSuperparent !=
         mergetreeSuperparentsPortal.Get(activeSupernodesPortal.Get(supernode - 1)));
    }
    // if our flag is set, we need to use the end of the superarc or infinity
    if (lastMergetreeSupernode)
    { // last supernode
      // there are two possibilities:
      // 1. the final mergetree superarc pointing to -infinity
      // 2. an ordinary mergetree superarc
      // we therefore retrieve the superarc to test
      vtkm::Id mergetreeSuperarc = mergetreeSuperarcsPortal.Get(mergetreeSuperparent);
      // if it is flagged as -infinity, preserve it
      if (NoSuchElement(mergetreeSuperarc))
      {
        augmentedMergetreeSuperarcsPortal.Set(supernodeID, (vtkm::Id)NO_SUCH_ELEMENT);
      }
      // otherwise, we use the new Mergetree ID of the mergetree superarc
      else
      {
        augmentedMergetreeSuperarcsPortal.Set(
          supernodeID, newMergetreeIDPortal.Get(MaskedIndex(mergetreeSuperarc)));
      }
    } // last supernode
    else
      // otherwise, converts supernode ID of neighbour & use it
      augmentedMergetreeSuperarcsPortal.Set(supernodeID, activeSupernodesPortal.Get(supernode - 1));

    // In serial this worklet implements the following operation
    /*
      for (vtkm::Id supernode = 0; supernode < nSupernodes; supernode++)
        { // per supernode
          // convert the sorting index to a supernode index
          vtkm::Id supernodeID = activeSupernodes[supernode];
          // retrieve the join superparent
          vtkm::Id joinSuperparent = joinSuperparents[supernodeID];
          // work out whether we're the "lowest" in the group
          bool lastJoinSupernode = false;
          // the 0'th one is always the last
          if (supernode == 0) lastJoinSupernode = true;
          // otherwise, check whether it's the same as the adjacent one
          else lastJoinSupernode = (joinSuperparent != joinSuperparents[activeSupernodes[supernode-1]]);

          // if our flag is set, we need to use the end of the superarc or infinity
          if (lastJoinSupernode)
                  { // last supernode
                  // there are two possibilities:
                  // 1. the final join superarc pointing to -infinity
                  // 2. an ordinary join superarc
                  // we therefore retrieve the superarc to test
                  vtkm::Id joinSuperarc = joinTree.superarcs[joinSuperparent];
                  // if it is flagged as -infinity, preserve it
                  if (NoSuchElement(joinSuperarc))
                          augmentedJoinSuperarcs[supernodeID] = NO_SUCH_ELEMENT;
                  else
                  // otherwise, we use the new Join ID of the join superarc
                          augmentedJoinSuperarcs[supernodeID] = newJoinID[MaskedIndex(joinSuperarc)];
                  } // last supernode
          else
                  // otherwise, converts supernode ID of neighbour & use it
                  augmentedJoinSuperarcs[supernodeID] = activeSupernodes[supernode-1];
        } // per supernode
      */
  }

}; // AugmentMergeTrees_InitNewJoinSplitIDAndSuperparents.h

} // namespace contourtree_maker_inc
} // namespace contourtree_augmented
} // namespace worklet
} // namespace vtkm

#endif
