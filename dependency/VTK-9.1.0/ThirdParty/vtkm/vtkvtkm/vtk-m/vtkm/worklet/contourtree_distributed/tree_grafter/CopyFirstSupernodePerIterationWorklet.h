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

#ifndef vtk_m_worklet_contourtree_distributed_tree_grafter_copy_first_supernode_per_iteration_worklet_h
#define vtk_m_worklet_contourtree_distributed_tree_grafter_copy_first_supernode_per_iteration_worklet_h


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
class CopyFirstSupernodePerIterationWorklet : public vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(
    FieldIn
      newSupernode, // fancy ArrayHandleCounting(nOldSupernodes, 1, nTotalSupernodes - nOldSupernodes)
    WholeArrayIn hierarchicalTreeWhichIteration, // input
    WholeArrayInOut
      hierarchicalTreeFirstSupernodePerIteration // output. Should be set to hierarchicalTree.firstSupernodePerIteration[theRound]/ // TODO: is it sufficient to use WholeArrayOut?
  );

  using ExecutionSignature = void(_1, _2, _3);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  CopyFirstSupernodePerIterationWorklet(vtkm::Id numOldSupernodes)
    : NumOldSupernodes(numOldSupernodes)
  {
  }

  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC void operator()(
    const vtkm::Id& newSupernode,
    const InFieldPortalType& hierarchicalTreeWhichIterationPortal,
    const OutFieldPortalType& hierarchicalTreeFirstSupernodePerIterationPortal

  ) const
  { // operator ()
    // per new supernode
    // the LHE is the first new supernode: it just stores
    if (newSupernode == this->NumOldSupernodes)
    {
      hierarchicalTreeFirstSupernodePerIterationPortal.Set(0, newSupernode);
    }
    // otherwise we only write if we detect a change in iterations
    else
    { // not LHE
      if (vtkm::worklet::contourtree_augmented::MaskedIndex(
            hierarchicalTreeWhichIterationPortal.Get(newSupernode)) !=
          vtkm::worklet::contourtree_augmented::MaskedIndex(
            hierarchicalTreeWhichIterationPortal.Get(newSupernode - 1)))
      {
        hierarchicalTreeFirstSupernodePerIterationPortal.Set(
          vtkm::worklet::contourtree_augmented::MaskedIndex(
            hierarchicalTreeWhichIterationPortal.Get(newSupernode)),
          newSupernode);
      }
    } // not LHE

    // In serial this worklet implements the following operation
    /*
    for (indexType newSupernode = nOldSupernodes; newSupernode < nTotalSupernodes; newSupernode++)
    { // per new supernode
      // the LHE is the first new supernode: it just stores
      if (newSupernode == nOldSupernodes)
        hierarchicalTree.firstSupernodePerIteration[theRound][0] = newSupernode;
      // otherwise we only write if we detect a change in iterations
      else
        { // not LHE
        if (maskedIndex(hierarchicalTree.whichIteration[newSupernode]) != maskedIndex(hierarchicalTree.whichIteration[newSupernode-1]))
          hierarchicalTree.firstSupernodePerIteration[theRound][maskedIndex(hierarchicalTree.whichIteration[newSupernode])] = newSupernode;
        } // not LHE
    } // per new supernode
    */
  } // operator ()

private:
  vtkm::Id NumOldSupernodes;

}; // CopyNewHypernodes

} // namespace tree_grafter
} // namespace contourtree_distributed
} // namespace worklet
} // namespace vtkm

#endif
