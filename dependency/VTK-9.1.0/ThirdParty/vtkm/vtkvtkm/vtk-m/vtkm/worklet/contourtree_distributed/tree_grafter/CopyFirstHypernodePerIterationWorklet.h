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

#ifndef vtk_m_worklet_contourtree_distributed_tree_grafter_copy_first_hypernode_per_iteration_worklet_h
#define vtk_m_worklet_contourtree_distributed_tree_grafter_copy_first_hypernode_per_iteration_worklet_h


#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree_distributed
{
namespace tree_grafter
{

/// Worklet implementing the copy of the first supernode per iteration in TreeGrafter::CopyIterationDetails
class CopyFirstHypernodePerIterationWorklet : public vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(
    FieldIn
      newHypernode, // input. Set to ArrayHandleCounting<vtkm::Id>(nOldHypernodes, 1, nTotalHypernodes-nOldHypernodes)
    WholeArrayIn hierarchicalTreeHypernodes,     // input
    WholeArrayIn hierarchicalTreeWhichIteration, //input
    WholeArrayInOut
      hierarchicalTreeFirstHypernodePerIteration //output. should be set to hierarchicalTree.firstHypernodePerIteration[theRound]. // TODO: is it sufficient to use WholeArrayOut?
  );

  using ExecutionSignature = void(_1, _2, _3, _4);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  CopyFirstHypernodePerIterationWorklet(vtkm::Id numOldHypernodes)
    : NumOldHypernodes(numOldHypernodes)
  {
  }

  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC void operator()(
    const vtkm::Id& newHypernode,
    const InFieldPortalType& hierarchicalTreeHypernodesPortal,
    const InFieldPortalType& hierarchicalTreeWhichIterationPortal,
    const OutFieldPortalType& hierarchicalTreeFirstHypernodePerIterationPortal) const
  { // operator ()
    // per new hypernode
    vtkm::Id supernodeId = hierarchicalTreeHypernodesPortal.Get(newHypernode);
    vtkm::Id wasTransferred = vtkm::worklet::contourtree_augmented::MaskedIndex(
      hierarchicalTreeWhichIterationPortal.Get(supernodeId));

    // the first one defines the zeroth iteration: changes in iteration number define the others
    if (newHypernode == this->NumOldHypernodes)
    { // LHE
      hierarchicalTreeFirstHypernodePerIterationPortal.Set(0, newHypernode);
    } // LHE
    else if (wasTransferred !=
             vtkm::worklet::contourtree_augmented::MaskedIndex(
               hierarchicalTreeWhichIterationPortal.Get(
                 hierarchicalTreeHypernodesPortal.Get(newHypernode - 1))))
    { // other breakpoint
      hierarchicalTreeFirstHypernodePerIterationPortal.Set(wasTransferred, newHypernode);
    } // other breakpoint

    // In serial this worklet implements the following operation
    /*
    for (indexType newHypernode = nOldHypernodes; newHypernode < nTotalHypernodes; newHypernode++)
    { // per new hypernode
      indexType supernodeID = hierarchicalTree.hypernodes[newHypernode];
      indexType wasTransferred = maskedIndex(hierarchicalTree.whichIteration[supernodeID]);

      // the first one defines the zeroth iteration: changes in iteration number define the others
      if (newHypernode == nOldHypernodes)
        { // LHE
        hierarchicalTree.firstHypernodePerIteration[theRound][0] = newHypernode;
        } // LHE
      else if (wasTransferred != maskedIndex(hierarchicalTree.whichIteration[hierarchicalTree.hypernodes[newHypernode-1]]))
        { // other breakpoint
        hierarchicalTree.firstHypernodePerIteration[theRound][wasTransferred] = newHypernode;
        } // other breakpoint
    } // per new hypernode
    */
  } // operator ()

private:
  vtkm::Id NumOldHypernodes;

}; // CopyNewHypernodes

} // namespace tree_grafter
} // namespace contourtree_distributed
} // namespace worklet
} // namespace vtkm

#endif
