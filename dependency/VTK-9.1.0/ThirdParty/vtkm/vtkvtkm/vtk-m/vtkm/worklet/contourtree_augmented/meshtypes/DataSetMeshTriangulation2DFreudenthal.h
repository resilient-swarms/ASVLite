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

#ifndef vtk_m_worklet_contourtree_augmented_data_set_mesh_triangulation_2d_freudenthal_h
#define vtk_m_worklet_contourtree_augmented_data_set_mesh_triangulation_2d_freudenthal_h

#include <cstdlib>
#include <vtkm/Types.h>

#include <vtkm/worklet/contourtree_augmented/DataSetMesh.h>
#include <vtkm/worklet/contourtree_augmented/meshtypes/MeshStructureFreudenthal2D.h>
#include <vtkm/worklet/contourtree_augmented/meshtypes/mesh_boundary/ComputeMeshBoundary2D.h>
#include <vtkm/worklet/contourtree_augmented/meshtypes/mesh_boundary/MeshBoundary2D.h>

#include <vtkm/cont/ExecutionObjectBase.h>

namespace vtkm
{
namespace worklet
{
namespace contourtree_augmented
{

class DataSetMeshTriangulation2DFreudenthal
  : public DataSetMesh
  , public vtkm::cont::ExecutionObjectBase
{ // class DataSetMeshTriangulation
public:
  // Constants and case tables
  m2d_freudenthal::EdgeBoundaryDetectionMasksType EdgeBoundaryDetectionMasks;
  static constexpr int MAX_OUTDEGREE = 3;

  //Mesh dependent helper functions
  void SetPrepareForExecutionBehavior(bool getMax);

  MeshStructureFreudenthal2D PrepareForExecution(vtkm::cont::DeviceAdapterId device,
                                                 vtkm::cont::Token& token) const;

  DataSetMeshTriangulation2DFreudenthal(vtkm::Id2 meshSize);

  MeshBoundary2DExec GetMeshBoundaryExecutionObject() const;

  void GetBoundaryVertices(IdArrayType& boundaryVertexArray,    // output
                           IdArrayType& boundarySortIndexArray, // output
                           MeshBoundary2DExec* meshBoundaryExecObj =
                             NULL // optional input, included for consistency with ContourTreeMesh
  ) const;

private:
  bool UseGetMax; // Define the behavior ofr the PrepareForExecution function
};                // class DataSetMeshTriangulation

// creates input mesh
inline DataSetMeshTriangulation2DFreudenthal::DataSetMeshTriangulation2DFreudenthal(
  vtkm::Id2 meshSize)
  : DataSetMesh(vtkm::Id3{ meshSize[0], meshSize[1], 1 })
  , EdgeBoundaryDetectionMasks{ vtkm::cont::make_ArrayHandle(
      m2d_freudenthal::EdgeBoundaryDetectionMasks,
      m2d_freudenthal::N_INCIDENT_EDGES,
      vtkm::CopyFlag::Off) }
{
}

inline void DataSetMeshTriangulation2DFreudenthal::SetPrepareForExecutionBehavior(bool getMax)
{
  this->UseGetMax = getMax;
}

// Get VTKM execution object that represents the structure of the mesh and provides the mesh helper functions on the device
inline MeshStructureFreudenthal2D DataSetMeshTriangulation2DFreudenthal::PrepareForExecution(
  vtkm::cont::DeviceAdapterId device,
  vtkm::cont::Token& token) const
{
  return MeshStructureFreudenthal2D(vtkm::Id2{ this->MeshSize[0], this->MeshSize[1] },
                                    m2d_freudenthal::N_INCIDENT_EDGES,
                                    this->UseGetMax,
                                    this->SortIndices,
                                    this->SortOrder,
                                    this->EdgeBoundaryDetectionMasks,
                                    device,
                                    token);
}

inline MeshBoundary2DExec DataSetMeshTriangulation2DFreudenthal::GetMeshBoundaryExecutionObject()
  const
{
  return MeshBoundary2DExec(vtkm::Id2{ this->MeshSize[0], this->MeshSize[1] }, this->SortIndices);
}

inline void DataSetMeshTriangulation2DFreudenthal::GetBoundaryVertices(
  IdArrayType& boundaryVertexArray,    // output
  IdArrayType& boundarySortIndexArray, // output
  MeshBoundary2DExec*
    meshBoundaryExecObj // optional input, included for consistency with ContourTreeMesh
) const
{
  vtkm::Id numBoundary = 2 * this->MeshSize[1] + 2 * this->MeshSize[0] - 4;
  auto boundaryId = vtkm::cont::ArrayHandleIndex(numBoundary);
  ComputeMeshBoundary2D computeMeshBoundary2dWorklet;
  vtkm::cont::Invoker invoke;
  invoke(computeMeshBoundary2dWorklet,
         boundaryId,        // input
         this->SortIndices, // input
         (meshBoundaryExecObj == NULL) ? this->GetMeshBoundaryExecutionObject()
                                       : *meshBoundaryExecObj, // input
         boundaryVertexArray,                                  // output
         boundarySortIndexArray                                // output
  );
}

} // namespace contourtree_augmented
} // worklet
} // vtkm

#endif
