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


#ifndef vtk_m_worklet_contourtree_augmented_active_graph_transfer_saddle_starts_set_new_outdegree_for_saddles_h
#define vtk_m_worklet_contourtree_augmented_active_graph_transfer_saddle_starts_set_new_outdegree_for_saddles_h

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


// Worklet to update all of the edges so that the far end resets to the result of the ascent in the previous step
class TransferSaddleStartsSetNewOutdegreeForSaddles : public vtkm::worklet::WorkletMapField
{
public:
  typedef void ControlSignature(FieldIn activeVertices,      // (input) active edges
                                WholeArrayIn firstEdge,      // (input) first edge
                                WholeArrayIn outdegree,      // (input) outdegree
                                WholeArrayIn activeEdges,    // (input) active edges
                                WholeArrayIn hyperarcs,      // (input) hyperarcs
                                WholeArrayIn edgeFar,        // (input) edgeFar
                                WholeArrayOut newOutdegree); // (output) new outdegree
  typedef void ExecutionSignature(_1, InputIndex, _2, _3, _4, _5, _6, _7);
  using InputDomain = _1;

  // Default Constructor
  VTKM_EXEC_CONT
  TransferSaddleStartsSetNewOutdegreeForSaddles() {}

  template <typename OutFieldPortalType, typename InFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id& vertexId,
                            const vtkm::Id vertex,
                            const InFieldPortalType& firstEdgePortal,
                            const InFieldPortalType& outdegreePortal,
                            const InFieldPortalType& activeEdgesPortal,
                            const InFieldPortalType& hyperarcsPortal,
                            const InFieldPortalType& edgeFarPortal,
                            const OutFieldPortalType& newOutdegreePortal) const
  {
    // first start found
    vtkm::Id firstExt = (vtkm::Id)NO_SUCH_ELEMENT;
    bool isGenuineSaddle = false;

    // loop through edges
    vtkm::Id firstE = firstEdgePortal.Get(vertexId);
    for (vtkm::Id edge = firstE; edge < firstE + outdegreePortal.Get(vertexId); edge++)
    { // per edge
      // retrieve the edge ID and the far end of the edge
      vtkm::Id edgeId = activeEdgesPortal.Get(edge);
      vtkm::Id nbrFar = MaskedIndex(hyperarcsPortal.Get(edgeFarPortal.Get(edgeId)));

      // skip looping edges
      if (nbrFar == vertexId)
        continue;

      // test for first one found
      if (firstExt == (vtkm::Id)NO_SUCH_ELEMENT)
        firstExt = nbrFar;
      else // otherwise, check for whether we have an actual merge saddle
        if (firstExt != nbrFar)
      { // first non-matching
        isGenuineSaddle = true;
        break;
      } // first non-matching
    }   // per edge

    // if it's not a genuine saddle, ignore it
    if (!isGenuineSaddle)
    {
      newOutdegreePortal.Set(vertex, 0);
    }
    else
    {
      newOutdegreePortal.Set(vertex, outdegreePortal.Get(vertexId));
    }


    // In serial this worklet implements the following operation
    /*
         for (indexType vertex = 0; vertex < activeVertices.size(); vertex++)
            { // per vertex
            // retrieve the actual vertex ID
            indexType vertexID = activeVertices[vertex];

            // first start found
            indexType firstExt = NO_SUCH_ELEMENT;
            bool isGenuineSaddle = false;

            // loop through edges
            indexType firstE = firstEdge[vertexID];
            for (indexType edge = firstE; edge < firstE + outdegree[vertexID]; edge++)
                    { // per edge
                    // retrieve the edge ID and the far end of the edge
                    indexType edgeID = activeEdges[edge];
                    indexType nbrFar = MaskedIndex(hyperarcs[edgeFar[edgeID]]);

                    // skip looping edges
                    if (nbrFar == vertexID)
                            continue;

                    // test for first one found
                    if (firstExt == NO_SUCH_ELEMENT)
                            firstExt = nbrFar;
                    else // otherwise, check for whether we have an actual merge saddle
                            if (firstExt != nbrFar)
                                    { // first non-matching
                                    isGenuineSaddle = true;
                                    break;
                                    } // first non-matching
                    } // per edge

            // if it's not a genuine saddle, ignore it
            if (!isGenuineSaddle)
                    {
                    newOutdegree[vertex] = 0;
                    }
            else
                    {
                    newOutdegree[vertex]  = outdegree[vertexID];
                    }
            } // per vertex*/

  } // operator

}; //  TransferSaddleStartsSetNewOutdegreeForSaddles


} // namespace active_graph_inc
} // namespace contourtree_augmented
} // namespace worklet
} // namespace vtkm

#endif
