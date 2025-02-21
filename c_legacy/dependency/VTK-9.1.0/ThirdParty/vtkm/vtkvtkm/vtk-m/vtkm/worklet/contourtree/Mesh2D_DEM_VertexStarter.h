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

#ifndef vtkm_worklet_contourtree_mesh2d_dem_vertex_starter_h
#define vtkm_worklet_contourtree_mesh2d_dem_vertex_starter_h

#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/contourtree/Mesh2D_DEM_Triangulation_Macros.h>
#include <vtkm/worklet/contourtree/VertexValueComparator.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree
{

// Worklet for setting initial chain maximum value
template <typename T>
class Mesh2D_DEM_VertexStarter : public vtkm::worklet::WorkletMapField
{
public:
  using TagType = vtkm::List<T>;

  using ControlSignature = void(FieldIn vertex,      // (input) index of vertex
                                WholeArrayIn values, // (input) values within mesh
                                FieldOut chain,      // (output) modify the chains
                                FieldOut linkMask);  // (output) modify the mask
  using ExecutionSignature = void(_1, _2, _3, _4);
  using InputDomain = _1;

  vtkm::Id nRows; // (input) number of rows in 2D
  vtkm::Id nCols; // (input) number of cols in 2D
  bool ascending; // ascending or descending (join or split tree)

  // Constructor
  VTKM_EXEC_CONT
  Mesh2D_DEM_VertexStarter(vtkm::Id NRows, vtkm::Id NCols, bool Ascending)
    : nRows(NRows)
    , nCols(NCols)
    , ascending(Ascending)
  {
  }

  // Locate the next vertex in direction indicated
  template <typename InFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id& vertex,
                            const InFieldPortalType& values,
                            vtkm::Id& chain,
                            vtkm::Id& linkMask) const
  {
    VertexValueComparator<InFieldPortalType> lessThan(values);
    vtkm::Id row = VERTEX_ROW(vertex, nCols);
    vtkm::Id col = VERTEX_COL(vertex, nCols);

    vtkm::Id destination = vertex;
    vtkm::Id mask = 0;

    bool isLeft = (col == 0);
    bool isRight = (col == nCols - 1);
    bool isTop = (row == 0);
    bool isBottom = (row == nRows - 1);

    for (vtkm::Id edgeNo = 0; edgeNo < N_INCIDENT_EDGES; edgeNo++)
    { // per edge
      vtkm::Id nbr;

      switch (edgeNo)
      {
        case 5: // up
          if (isTop)
            break;
          nbr = vertex - nCols;
          if (lessThan(vertex, nbr, ascending))
            break;
          mask |= 0x20;
          destination = nbr;
          break;

        case 4: // up left
          if (isLeft || isTop)
            break;
          nbr = vertex - nCols - 1;
          if (lessThan(vertex, nbr, ascending))
            break;
          mask |= 0x10;
          destination = nbr;
          break;

        case 3: // left
          if (isLeft)
            break;
          nbr = vertex - 1;
          if (lessThan(vertex, nbr, ascending))
            break;
          mask |= 0x08;
          destination = nbr;
          break;

        case 2: // down
          if (isBottom)
            break;
          nbr = vertex + nCols;
          if (lessThan(vertex, nbr, ascending))
            break;
          mask |= 0x04;
          destination = nbr;
          break;

        case 1: // down right
          if (isBottom || isRight)
            break;
          nbr = vertex + nCols + 1;
          if (lessThan(vertex, nbr, ascending))
            break;
          mask |= 0x02;
          destination = nbr;
          break;

        case 0: // right
          if (isRight)
            break;
          nbr = vertex + 1;
          if (lessThan(vertex, nbr, ascending))
            break;
          mask |= 0x01;
          destination = nbr;
          break;
      } // switch on edgeNo
    }   // per edge

    linkMask = mask;
    chain = destination;
  } // operator()

}; // Mesh2D_DEM_VertexStarter
}
}
}

#endif
