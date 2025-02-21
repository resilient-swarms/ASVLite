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
// This functor identifies for each vertex which edges to keep. For arbitrary meshes,
// this should use reductions. For regular meshes, this way is faster due to low bounded
// updegree.
//
// Any vector needed by the functor for lookup purposes will be passed as a parameter to
// the constructor and saved, with the actual function call being the operator ()
//
// Vectors marked I/O are intrinsically risky unless there is an algorithmic guarantee
// that the read/writes are completely independent - which for our case actually occurs
// The I/O vectors should therefore be justified in comments both here & in caller
//
//=======================================================================================

#ifndef vtkm_worklet_contourtree_vertex_degree_updater_h
#define vtkm_worklet_contourtree_vertex_degree_updater_h

#include <vtkm/worklet/WorkletMapField.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree
{

// Worklet for setting initial chain maximum value
class VertexDegreeUpdater : public vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(FieldIn vertexID,         // (input) active vertices
                                WholeArrayIn activeEdges, // (input) active edges
                                WholeArrayIn edgeFar,     // (input) high ends of edges
                                WholeArrayIn firstEdge, // (input) first edge for each active vertex
                                WholeArrayIn prunesTo,  // (input) where vertex is pruned to
                                WholeArrayIn outdegree, // (input) updegree of vertex
                                WholeArrayInOut chainExtemum, // (i/o) chain extemum for vertices
                                FieldOut newOutdegree);       // (output) new updegree of vertex
  using ExecutionSignature = _8(_1, _2, _3, _4, _5, _6, _7);
  using InputDomain = _1;

  // chainMaximum is safe for I/O here because:
  // 		we have previously eliminated maxima from the active vertex list
  //		our lookup uses the chainMaximum of the edgeHigh, which is guaranteed to
  //		be a maximum
  //		therefore, the chainMaximum entries edited are *NEVER* also accessed & v.v.

  // Constructor
  VTKM_EXEC_CONT
  VertexDegreeUpdater() {}

  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC vtkm::Id operator()(const vtkm::Id& vertexID,
                                const InFieldPortalType& activeEdges,
                                const InFieldPortalType& edgeFar,
                                const InFieldPortalType& firstEdge,
                                const InFieldPortalType& prunesTo,
                                const InFieldPortalType& outdegree,
                                const OutFieldPortalType& chainExtremum) const
  {
    vtkm::Id newOutdegree = 0;

    // retrieve actual vertex ID & first edge
    vtkm::Id edgeFirst = firstEdge.Get(vertexID);

    // also reset the chain maximum to the vertex' ID
    chainExtremum.Set(vertexID, vertexID);

    // walk through the vertex' edges
    for (vtkm::Id edge = 0; edge < outdegree.Get(vertexID); edge++)
    {
      vtkm::Id edgeIndex = edgeFirst + edge;
      vtkm::Id edgeID = activeEdges.Get(edgeIndex);

      // retrieve the vertex ID for the high end & update for pruning
      vtkm::Id highEnd = prunesTo.Get(chainExtremum.Get(edgeFar.Get(edgeID)));

      // we want to ignore edges that lead back to this vertex
      if (highEnd == vertexID)
        continue;

      // if we survived, increment the outdegree
      newOutdegree++;
    } // per edge
    return newOutdegree;
  }
}; // VertexDegreeUpdater
}
}
}

#endif
