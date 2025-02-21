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

#ifndef vtk_m_worklet_contourtree_augmented_active_graph_set_arcs_connect_nodes_h
#define vtk_m_worklet_contourtree_augmented_active_graph_set_arcs_connect_nodes_h

#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree_augmented
{
namespace active_graph_inc
{

// Worklet for computing the sort indices from the sort order
class SetArcsConnectNodes : public vtkm::worklet::WorkletMapField
{
public:
  typedef void ControlSignature(
    WholeArrayInOut treeArcs, // (input/output) tree,arcs
    WholeArrayIn
      nodes, // (input) nodes of tree sorted by tree.superparents using SuperArcNodeComparator
    WholeArrayIn treeSuperparents, // (input) tree superparents
    WholeArrayIn treeSuperarcs,    // (input) tree superarcs
    WholeArrayIn treeSupernodes);  // (input) tree supernodes

  typedef void ExecutionSignature(_1, InputIndex, _2, _3, _4, _5);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  SetArcsConnectNodes() {}

  template <typename InFieldPortalType, typename InOutFieldPortalType>
  VTKM_EXEC void operator()(const InOutFieldPortalType& treeArcsPortal,
                            const vtkm::Id node,
                            const InFieldPortalType& nodesPortal,
                            const InFieldPortalType& treeSuperparentsPortal,
                            const InFieldPortalType& treeSuperarcsPortal,
                            const InFieldPortalType& treeSupernodesPortal) const
  {
    // per node
    vtkm::Id nodeID = nodesPortal.Get(node);
    // work out whether we have the first node on the superarc
    vtkm::Id superparent = treeSuperparentsPortal.Get(nodeID);
    if (node == 0)
    { // left edge
      vtkm::Id superarc = treeSuperarcsPortal.Get(superparent);
      // explicit check for global minimum
      if (NoSuchElement(superarc))
        treeArcsPortal.Set(nodeID, (vtkm::Id)NO_SUCH_ELEMENT);
      else
        treeArcsPortal.Set(nodeID, treeSupernodesPortal.Get(treeSuperarcsPortal.Get(superparent)));
    } // left edge
    else if (superparent != treeSuperparentsPortal.Get(nodesPortal.Get(node - 1)))
    { // any other transition
      vtkm::Id superarc = treeSuperarcsPortal.Get(superparent);
      // explicit check for global minimum
      if (NoSuchElement(superarc))
        treeArcsPortal.Set(nodeID, (vtkm::Id)NO_SUCH_ELEMENT);
      else
        treeArcsPortal.Set(nodeID, treeSupernodesPortal.Get(treeSuperarcsPortal.Get(superparent)));
    } // any other transition
    else
    {
      treeArcsPortal.Set(nodeID, nodesPortal.Get(node - 1));
    }

    // In serial this worklet implements the following operation
    /*
      for (indexType node = 0; node < tree.Arcs.size(); node++)
        { // per node
          indexType nodeID = nodes[node];
          // work out whether we have the first node on the superarc
          indexType superparent = tree.superparents[nodeID];
          if (node == 0)
            { // left edge
              indexType superarc = tree.superarcs[superparent];
              // explicit check for global minimum
              if (NoSuchElement(superarc))
                      tree.Arcs[nodeID] = NO_SUCH_ELEMENT;
              else
                      tree.Arcs[nodeID] = tree.Supernodes[tree.superarcs[superparent]];
            } // left edge
          else if (superparent != tree.Superparents[nodes[node-1]])
            { // any other transition
              indexType superarc = tree.superarcs[superparent];
              // explicit check for global minimum
              if (NoSuchElement(superarc))
                      tree.Arcs[nodeID] = NO_SUCH_ELEMENT;
              else
                      tree.Arcs[nodeID] = tree.Supernodes[tree.superarcs[superparent]];
            } // any other transition
          else
            tree.Arcs[nodeID] = nodes[node-1];
        } // per node
        */
  }

}; // SetArcsConnectNodes

} // namespace active_graph_inc
} // namespace contourtree_augmented
} // namespace worklet
} // namespace vtkm

#endif
