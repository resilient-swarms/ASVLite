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

#ifndef vtkm_worklet_contourtree_mesh3d_dem_vertex_outdegree_starter_h
#define vtkm_worklet_contourtree_mesh3d_dem_vertex_outdegree_starter_h

#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree/LinkComponentCaseTable3D.h>
#include <vtkm/worklet/contourtree/Mesh3D_DEM_Triangulation_Macros.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree
{

// Worklet for setting initial chain maximum value
class Mesh3D_DEM_VertexOutdegreeStarter : public vtkm::worklet::WorkletMapField
{
public:
  using ControlSignature = void(FieldIn vertex,              // (input) index into active vertices
                                FieldIn nbrMask,             // (input) neighbor mask
                                WholeArrayIn arcArray,       // (input) chain extrema
                                WholeArrayIn neighbourTable, // (input) table for neighbour offsets
                                WholeArrayIn caseTable,      // (input) case table for neighbours
                                FieldOut outdegree,          // (output) outdegree
                                FieldOut isCritical);        // (output) whether critical
  using ExecutionSignature = void(_1, _2, _3, _4, _5, _6, _7);
  using InputDomain = _1;

  vtkm::Id nRows;   // (input) number of rows in 3D
  vtkm::Id nCols;   // (input) number of cols in 3D
  vtkm::Id nSlices; // (input) number of cols in 3D
  bool ascending;   // (input) ascending or descending (join or split tree)

  // Constructor
  VTKM_EXEC_CONT
  Mesh3D_DEM_VertexOutdegreeStarter(vtkm::Id NRows,
                                    vtkm::Id NCols,
                                    vtkm::Id NSlices,
                                    bool Ascending)
    : nRows(NRows)
    , nCols(NCols)
    , nSlices(NSlices)
    , ascending(Ascending)
  {
  }

  //template<typename InFieldPortalType>
  template <typename InFieldPortalType, typename NeighbourTableType, typename CaseTableType>
  VTKM_EXEC void operator()(const vtkm::Id& vertex,
                            const vtkm::Id& nbrMask,
                            const InFieldPortalType& arcArray,
                            const NeighbourTableType& neighbourTable,
                            const CaseTableType& caseTable,
                            vtkm::Id& outdegree,
                            vtkm::Id& isCritical) const
  {
    // get the row and column
    vtkm::Id row = VERTEX_ROW_3D(vertex, nRows, nCols);
    vtkm::Id col = VERTEX_COL_3D(vertex, nRows, nCols);
    vtkm::Id slice = VERTEX_SLICE_3D(vertex, nRows, nCols);

    // we know which edges are outbound, so we count to get the outdegree
    vtkm::Id outDegree = 0;
    vtkm::Id farEnds[MAX_OUTDEGREE_3D];

    for (vtkm::Id edgeNo = 0; edgeNo < N_INCIDENT_EDGES_3D; edgeNo++)
    {
      if (caseTable.Get(nbrMask) & (1 << edgeNo))
      {
        vtkm::Id indx = edgeNo * 3;
        vtkm::Id nbrSlice = slice + neighbourTable.Get(indx);
        vtkm::Id nbrRow = row + neighbourTable.Get(indx + 1);
        vtkm::Id nbrCol = col + neighbourTable.Get(indx + 2);
        vtkm::Id nbr = VERTEX_ID_3D(nbrSlice, nbrRow, nbrCol, nRows, nCols);

        farEnds[outDegree++] = arcArray.Get(nbr);
      }
    }

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

    // now store the outDegree
    outdegree = outDegree;

    // and set the initial inverse index to a flag
    isCritical = (outDegree != 1) ? 1 : 0;
  }
}; // Mesh3D_DEM_VertexOutdegreeStarter

} // namespace contourtree
} // namespace worklet
} // namespace vtkm

#endif
