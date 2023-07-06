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

#ifndef vtk_m_worklet_contourtree_distributed_tree_grafter_find_critical_points_find_leafs_worklet_h
#define vtk_m_worklet_contourtree_distributed_tree_grafter_find_critical_points_find_leafs_worklet_h


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

// Worklet used in TreeGrafter.FindCriticalPoints to flag the leafs
class FindCriticalPointsFindLeafsWorklet : public vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(
    FieldIn
      activeSuperarcs, // input iteration index. loop to one less than ContourTree->Supernodes.GetNumberOfValues()
    WholeArrayIn interiorForstIsNecessary, // input
    WholeArrayIn upNeighbour,              // input
    WholeArrayIn downNeighbour,            // input
    WholeArrayInOut supernodeType          // output
  );

  using ExecutionSignature = void(_1, _2, _3, _4, _5);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  FindCriticalPointsFindLeafsWorklet() {}

  template <typename InFieldPortalType, typename InOutFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::worklet::contourtree_augmented::EdgePair& activeSuperarc,
                            const InFieldPortalType& isNecessaryPortal,
                            const InFieldPortalType& upNeighbourPortal,
                            const InFieldPortalType& downNeighbourPortal,
                            const InOutFieldPortalType& supernodeTypePortal) const
  { // operator ()
    // flag the leaves
    // per active superarc
    vtkm::Id lowEnd = activeSuperarc.first;
    vtkm::Id highEnd = activeSuperarc.second;

    // if the low end thinks it ascends to the high end and does
    // not have a descent, then it must be a lower leaf, unless it's
    // necessary (i.e. an attachment)
    if ((supernodeTypePortal.Get(lowEnd) !=
         (vtkm::Id)vtkm::worklet::contourtree_augmented::IS_SADDLE) &&
        (upNeighbourPortal.Get(lowEnd) == highEnd) &&
        vtkm::worklet::contourtree_augmented::NoSuchElement(downNeighbourPortal.Get(lowEnd)) &&
        (!isNecessaryPortal.Get(lowEnd)))
    {
      supernodeTypePortal.Set(lowEnd,
                              (vtkm::Id)vtkm::worklet::contourtree_augmented::IS_LOWER_LEAF);
    }

    // symmetrically for the high end
    if ((supernodeTypePortal.Get(highEnd) !=
         (vtkm::Id)vtkm::worklet::contourtree_augmented::IS_SADDLE) &&
        (downNeighbourPortal.Get(highEnd) == lowEnd) &&
        vtkm::worklet::contourtree_augmented::NoSuchElement(upNeighbourPortal.Get(highEnd)) &&
        (!isNecessaryPortal.Get(highEnd)))
    {
      supernodeTypePortal.Set(highEnd,
                              (vtkm::Id)vtkm::worklet::contourtree_augmented::IS_UPPER_LEAF);
    }

    // In serial this worklet implements the following operation
    /*
    // flag the leaves
        for (indexType activeSuper = 0; activeSuper < activeSuperarcs.size(); activeSuper++)
          { // per active superarc
          indexType lowEnd = activeSuperarcs[activeSuper].low, highEnd = activeSuperarcs[activeSuper].high;

          // if the low end thinks it ascends to the high end and does not have a descent, then it must be a lower leaf, unless it's necessary (ie an attachment)
          if ((supernodeType[lowEnd] != IS_SADDLE) && (upNeighbour[lowEnd] == highEnd) && noSuchElement(downNeighbour[lowEnd]) && (!residue->isNecessary[lowEnd]))
              supernodeType[lowEnd] = IS_LOWER_LEAF;

          // symmetrically for the high end
          if ((supernodeType[highEnd] != IS_SADDLE) && (downNeighbour[highEnd] == lowEnd) && noSuchElement(upNeighbour[highEnd]) && (!residue->isNecessary[highEnd]))
              supernodeType[highEnd] = IS_UPPER_LEAF;
          } // per active superarc

    */
  } // operator ()

}; // FindCriticalPointsFindLeafsWorklet

} // namespace tree_grafter
} // namespace contourtree_distributed
} // namespace worklet
} // namespace vtkm

#endif
