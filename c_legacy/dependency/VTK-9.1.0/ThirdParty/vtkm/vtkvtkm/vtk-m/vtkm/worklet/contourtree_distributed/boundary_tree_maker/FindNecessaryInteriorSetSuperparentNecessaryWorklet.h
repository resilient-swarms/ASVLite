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

#ifndef vtk_m_worklet_contourtree_distributed_bract_maker_find_necessary_interior_supernodes_set_superparent_necessary_worklet_h
#define vtk_m_worklet_contourtree_distributed_bract_maker_find_necessary_interior_supernodes_set_superparent_necessary_worklet_h

#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree_distributed
{
namespace bract_maker
{

/// Worklet to transfer the dependent counts for hyperarcs
/// Part of the BoundaryRestrictedAugmentedContourTree.PropagateBoundaryCounts function
class FindNecessaryInteriorSetSuperparentNecessaryWorklet : public vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(FieldIn boundaryIndices,   // (input)
                                WholeArrayIn superparents, // (input)
                                WholeArrayIn superarcs,    // (input)
                                WholeArrayOut isNecessary  // (output)
  );
  using ExecutionSignature = void(_1, _2, _3, _4);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  FindNecessaryInteriorSetSuperparentNecessaryWorklet() {}

  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id& boundaryIndex,
                            const InFieldPortalType superparentsPortal,
                            const InFieldPortalType superarcsPortal,
                            const OutFieldPortalType& isNecessaryPortal) const
  {
    // per boundary node
    // find the superparent
    vtkm::Id superparent =
      vtkm::worklet::contourtree_augmented::MaskedIndex(superparentsPortal.Get(boundaryIndex));
    // make it necessary
    isNecessaryPortal.Set(superparent, true);
    // retrieve the supertarget
    vtkm::Id supertarget = superarcsPortal.Get(superparent);
    // check for NULL ness & set the target as well if it exists
    if (vtkm::worklet::contourtree_augmented::NoSuchElement(supertarget))
    {
      return;
    }
    // set the target as well
    isNecessaryPortal.Set(vtkm::worklet::contourtree_augmented::MaskedIndex(supertarget), true);
    // In serial this worklet implements the following operation
    /*
    for (indexType boundaryNode = 0; boundaryNode < boundaryIndices.size(); boundaryNode++)
      { // per boundary node

      // find the superparent
      indexType superparent = maskedIndex(contourTree->superparents[boundaryIndices[boundaryNode]]);
      // make it necessary
      isNecessary[superparent] = true;
      // retrieve the supertarget
      indexType supertarget = contourTree->superarcs[superparent];
      // check for NULL ness & set the target as well if it exists
      if (noSuchElement(supertarget))
        continue;
      // set the target as well
      isNecessary[maskedIndex(supertarget)] = true;
      } // per boundary node
    */
  }
}; // FindNecessaryInteriorSetSuperparentNecessaryWorklet


} // namespace bract_maker
} // namespace contourtree_distributed
} // namespace worklet
} // namespace vtkm

#endif
