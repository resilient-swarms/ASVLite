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
// This functor replaces a parallel loop examining neighbours - again, for arbitrary
// meshes, it needs to be a reduction, but for regular meshes, it's faster this way.
//
// Any vector needed by the functor for lookup purposes will be passed as a parameter to
// the constructor and saved, with the actual function call being the operator ()
//
// Vectors marked I/O are intrinsically risky unless there is an algorithmic guarantee
// that the read/writes are completely independent - which for our case actually occurs
// The I/O vectors should therefore be justified in comments both here & in caller
//
//=======================================================================================

#ifndef vtkm_worklet_contourtree_mesh2d_dem_saddle_starter_h
#define vtkm_worklet_contourtree_mesh2d_dem_saddle_starter_h

#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree/Mesh2D_DEM_Triangulation_Macros.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree
{

// Worklet for setting initial chain maximum value
class Mesh2D_DEM_SaddleStarter : public vtkm::worklet::WorkletMapField
{
public:
  using PairType = vtkm::List<vtkm::Pair<vtkm::Id, vtkm::Id>>;

  using ControlSignature = void(FieldIn vertex,          // (input) index into active vertices
                                FieldIn outDegFirstEdge, // (input) out degree/first edge of vertex
                                FieldIn valueIndex,      // (input) index into regular graph
                                WholeArrayIn linkMask,   // (input) neighbors of vertex
                                WholeArrayIn arcArray,   // (input) chain extrema per vertex
                                WholeArrayIn inverseIndex,  // (input) permutation of index
                                WholeArrayOut edgeNear,     // (output) low end of edges
                                WholeArrayOut edgeFar,      // (output) high end of edges
                                WholeArrayOut activeEdges); // (output) active edge list
  using ExecutionSignature = void(_1, _2, _3, _4, _5, _6, _7, _8, _9);
  using InputDomain = _1;

  vtkm::Id nRows; // (input) number of rows in 2D
  vtkm::Id nCols; // (input) number of cols in 2D
  bool ascending; // (input) ascending or descending (join or split)

  // Constructor
  VTKM_EXEC_CONT
  Mesh2D_DEM_SaddleStarter(vtkm::Id NRows, vtkm::Id NCols, bool Ascending)
    : nRows(NRows)
    , nCols(NCols)
    , ascending(Ascending)
  {
  }

  // operator() routine that executes the loop
  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id& vertex,
                            const vtkm::Pair<vtkm::Id, vtkm::Id>& outDegFirstEdge,
                            const vtkm::Id& valueIndex,
                            const InFieldPortalType& linkMask,
                            const InFieldPortalType& arcArray,
                            const InFieldPortalType& inverseIndex,
                            const OutFieldPortalType& edgeNear,
                            const OutFieldPortalType& edgeFar,
                            const OutFieldPortalType& activeEdges) const
  {
    vtkm::Id outdegree = outDegFirstEdge.first;
    vtkm::Id firstEdge = outDegFirstEdge.second;
    // skip local extrema
    if (outdegree == 0)
      return;

    // get the saddle mask for the vertex
    vtkm::Id nbrMask = linkMask.Get(valueIndex);

    // get the row and column
    vtkm::Id row = VERTEX_ROW(valueIndex, nCols);
    vtkm::Id col = VERTEX_COL(valueIndex, nCols);

    // we now know which edges are starts, so we count to get the outdegree
    vtkm::Id outDegree = 0;
    vtkm::Id farEnds[MAX_OUTDEGREE];

    // copy into the temporary array
    if ((nbrMask & 0x30) == 0x20)
      farEnds[outDegree++] = inverseIndex.Get(arcArray.Get(VERTEX_ID(row - 1, col, nCols)));
    if ((nbrMask & 0x18) == 0x10)
      farEnds[outDegree++] = inverseIndex.Get(arcArray.Get(VERTEX_ID(row - 1, col - 1, nCols)));
    if ((nbrMask & 0x0C) == 0x08)
      farEnds[outDegree++] = inverseIndex.Get(arcArray.Get(VERTEX_ID(row, col - 1, nCols)));
    if ((nbrMask & 0x06) == 0x04)
      farEnds[outDegree++] = inverseIndex.Get(arcArray.Get(VERTEX_ID(row + 1, col, nCols)));
    if ((nbrMask & 0x03) == 0x02)
      farEnds[outDegree++] = inverseIndex.Get(arcArray.Get(VERTEX_ID(row + 1, col + 1, nCols)));
    if ((nbrMask & 0x21) == 0x01)
      farEnds[outDegree++] = inverseIndex.Get(arcArray.Get(VERTEX_ID(row, col + 1, nCols)));

    // now we check them against each other
    if ((outDegree == 2) && (farEnds[0] == farEnds[1]))
    { // outDegree 2 & both match
      // treat as a regular point
      outDegree = 1;
    } // outDegree 2 & both match
    else if (outDegree == 3)
    { // outDegree 3
      if (farEnds[0] == farEnds[1])
      { // first two match
        if (farEnds[0] == farEnds[2])
        { // triple match
          // all match - treat as regular point
          outDegree = 1;
        } // triple match
        else
        { // first two match, but not third
          // copy third down one place
          farEnds[1] = farEnds[2];
          // and reset the count
          outDegree = 2;
        } //
      }   // first two match
      else if ((farEnds[0] == farEnds[2]) || (farEnds[1] == farEnds[2]))
      { // second one matches either of the first two
        // decrease the count, keeping 0 & 1
        outDegree = 2;
      } // second one matches either of the first two
    }   // outDegree 3

    // now the farEnds array holds the far ends we can reach
    for (vtkm::Id edge = 0; edge < outDegree; edge++)
    {
      // compute the edge index in the edge arrays
      vtkm::Id edgeID = firstEdge + edge;

      // now set the near and far ends and save the edge itself
      edgeNear.Set(edgeID, vertex);
      edgeFar.Set(edgeID, farEnds[edge]);
      activeEdges.Set(edgeID, edgeID);
    } // per start
  }   // operator()

}; // Mesh2D_DEM_SaddleStarter
}
}
}

#endif
