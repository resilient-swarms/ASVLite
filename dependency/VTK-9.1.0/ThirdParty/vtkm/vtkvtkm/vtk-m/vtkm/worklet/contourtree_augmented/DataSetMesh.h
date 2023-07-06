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
//
//      Parallel Peak Pruning v. 2.0
//
// Mesh_2D_DEM_Triangulation.h - a 2D regular mesh
//
//==============================================================================
//
// COMMENTS:
//
// This is an abstraction to separate out the mesh from the graph algorithm
// that we will be executing.
//
// In this version, we will sort the values up front, and then keep track of
// them using indices only, without looking up their values. This should
// simplify several parts of code significantly, and reduce the memory bandwidth.
// Of course, in moving to 64-bit indices, we will not necessarily see gains.
//
//==============================================================================

#ifndef vtk_m_worklet_contourtree_augmented_data_set_mesh_h
#define vtk_m_worklet_contourtree_augmented_data_set_mesh_h

#include <vtkm/cont/Algorithm.h>
#include <vtkm/cont/ArrayCopy.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/ArrayHandleIndex.h>
#include <vtkm/cont/ArrayHandlePermutation.h>
#include <vtkm/cont/Invoker.h>

#include <vtkm/worklet/contourtree_augmented/PrintVectors.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>
#include <vtkm/worklet/contourtree_augmented/data_set_mesh/IdRelabeler.h>
#include <vtkm/worklet/contourtree_augmented/data_set_mesh/SimulatedSimplicityComperator.h>
#include <vtkm/worklet/contourtree_augmented/data_set_mesh/SortIndices.h>

//Define namespace alias for the freudenthal types to make the code a bit more readable

namespace vtkm
{
namespace worklet
{
namespace contourtree_augmented
{
class DataSetMesh
{
public:
  // common mesh size parameter, use all three dimensions ofr MeshSize with third determining if 2D or 3D
  // (convention: MeshSize[2] is always >= 1, even for empty data set, so that we can detect 2D
  // data as MeshSize[2] == 1)
  vtkm::Id3 MeshSize;
  vtkm::Id NumVertices, NumLogSteps;

  // Array with the sorted order of the mesh vertices
  IdArrayType SortOrder;

  // Array with the sort index for each vertex
  // i.e. the inverse permutation for SortOrder
  IdArrayType SortIndices;

  //empty constructor
  DataSetMesh()
    : MeshSize{ 0, 0, 1 } // Always set third dimension to 1 for easy detection of 2D vs 3D
    , NumVertices(0)
    , NumLogSteps(1)
  {
  }

  // base constructor
  DataSetMesh(vtkm::Id3 meshSize)
    : MeshSize{ meshSize }
    , NumVertices{ meshSize[0] * meshSize[1] * meshSize[2] }
    // per convention meshSize[2] == 1 for 2D
    , NumLogSteps(1)

  {
    // Per convention the third dimension should be 1 (even for an empty
    // mesh) or higher to make it easier to check for 2D vs. 3D data)
    VTKM_ASSERT(MeshSize[2] >= 1);
    // TODO/FIXME: An empty mesh will likely cause a crash down the
    // road anyway, so we may want to detect that case and handle
    // it appropriately.

    // Compute the number of log-jumping steps (i.e. lg_2 (NumVertices))
    // this->NumLogSteps = 1; // already set in initializer list
    for (vtkm::Id shifter = this->NumVertices; shifter > 0; shifter >>= 1)
      this->NumLogSteps++;
  }

  virtual ~DataSetMesh() {}

  // Getter function for NumVertices
  vtkm::Id GetNumberOfVertices() const { return this->NumVertices; }

  // Sorts the data and initializes SortOrder & SortIndices
  template <typename T, typename StorageType>
  void SortData(const vtkm::cont::ArrayHandle<T, StorageType>& values);

  /// Routine to return the global IDs for a set of vertices
  /// We here return a fancy array handle to convert values on-the-fly without requiring additional memory
  /// @param[in] meshIds Array with sort Ids to be converted from local to global Ids
  /// @param[in] localToGlobalIdRelabeler This parameter is the IdRelabeler
  ///            used to transform local to global Ids. The relabeler relies on the
  ///            decomposition of the global mesh which is not know by this block.
  inline vtkm::cont::ArrayHandleTransform<
    vtkm::cont::ArrayHandlePermutation<IdArrayType, IdArrayType>,
    mesh_dem::IdRelabeler>
  GetGlobalIdsFromSortIndices(const IdArrayType& sortIds,
                              const mesh_dem::IdRelabeler* localToGlobalIdRelabeler) const
  { // GetGlobalIDsFromSortIndices()
    auto permutedSortOrder = vtkm::cont::make_ArrayHandlePermutation(sortIds, this->SortOrder);
    return vtkm::cont::ArrayHandleTransform<
      vtkm::cont::ArrayHandlePermutation<IdArrayType, IdArrayType>,
      mesh_dem::IdRelabeler>(permutedSortOrder, *localToGlobalIdRelabeler);
  } // GetGlobalIDsFromSortIndices()

  /// Routine to return the global IDs for a set of vertices
  /// We here return a fancy array handle to convert values on-the-fly without requiring additional memory
  /// SortIdArrayType must be an array if Ids. Usually this is a vtkm::worklet::contourtree_augmented::IdArrayType
  /// but in some cases it may also be a fancy array to avoid memory allocation
  /// @param[in] meshIds Array with mesh Ids to be converted from local to global Ids
  /// @param[in] localToGlobalIdRelabeler This parameter is the IdRelabeler
  ///            used to transform local to global Ids. The relabeler relies on the
  ///            decomposition of the global mesh which is not know by this block.
  template <typename MeshIdArrayType>
  inline vtkm::cont::ArrayHandleTransform<MeshIdArrayType, mesh_dem::IdRelabeler>
  GetGlobalIdsFromMeshIndices(const MeshIdArrayType& meshIds,
                              const mesh_dem::IdRelabeler* localToGlobalIdRelabeler) const
  { // GetGlobalIDsFromMeshIndices()
    return vtkm::cont::ArrayHandleTransform<MeshIdArrayType, mesh_dem::IdRelabeler>(
      meshIds, *localToGlobalIdRelabeler);
  } // GetGlobalIDsFromMeshIndices()

  //routine that dumps out the contents of the mesh
  void DebugPrint(const char* message, const char* fileName, long lineNum);

protected:
  virtual void DebugPrintExtends();
  template <typename T, typename StorageType>
  void DebugPrintValues(const vtkm::cont::ArrayHandle<T, StorageType>& values);
}; // class DataSetMesh

// Sorts the data and initialises the SortIndices & SortOrder
template <typename T, typename StorageType>
inline void DataSetMesh::SortData(const vtkm::cont::ArrayHandle<T, StorageType>& values)
{
  // Define namespace alias for mesh dem worklets
  namespace mesh_dem_worklets = vtkm::worklet::contourtree_augmented::mesh_dem;

  // Make sure that the values have the correct size
  VTKM_ASSERT(values.GetNumberOfValues() == this->NumVertices);

  // Make sure that we are not running on an empty mesh
  VTKM_ASSERT(this->NumVertices > 0);

  // Just in case, make sure that everything is cleaned up
  this->SortIndices.ReleaseResources();
  this->SortOrder.ReleaseResources();

  // allocate memory for the sort arrays
  this->SortOrder.Allocate(this->NumVertices);
  this->SortIndices.Allocate(this->NumVertices);

  // now sort the sort order vector by the values, i.e,. initialize the SortOrder member variable
  vtkm::cont::ArrayHandleIndex initVertexIds(
    this->NumVertices); // create sequence 0, 1, .. NumVertices
  vtkm::cont::ArrayCopy(initVertexIds, this->SortOrder);

  vtkm::cont::Algorithm::Sort(this->SortOrder,
                              mesh_dem::SimulatedSimplicityIndexComparator<T, StorageType>(values));

  // now set the index lookup, i.e., initialize the SortIndices member variable
  // In serial this would be
  //  for (indexType vertex = 0; vertex < NumVertices; vertex++)
  //            SortIndices[SortOrder[vertex]] = vertex;
  data_set_mesh::SortIndices sortIndicesWorklet;
  vtkm::cont::Invoker invoke;
  invoke(sortIndicesWorklet, this->SortOrder, this->SortIndices);

  // Debug print statement
  DebugPrint("Data Sorted", __FILE__, __LINE__);
  DebugPrintValues(values);
} // SortData()

// Print mesh extends
inline void DataSetMesh::DebugPrintExtends()
{
  // For compatibility with the output of the original PPP Implementation, print size
  // as NumRows, NumColumns and NumSlices (if applicable)
  PrintLabel("NumRows");
  PrintIndexType(this->MeshSize[1]);
  std::cout << std::endl;
  PrintLabel("NumColumns");
  PrintIndexType(this->MeshSize[0]);
  std::cout << std::endl;
  if (MeshSize[2] > 1)
  {
    PrintLabel("NumSlices");
    PrintIndexType(this->MeshSize[2]);
    std::cout << std::endl;
  }
} // DebugPrintExtends

inline void DataSetMesh::DebugPrint(const char* message, const char* fileName, long lineNum)
{ // DebugPrint()
#ifdef DEBUG_PRINT
  std::cout << "------------------------------------------------------" << std::endl;
  std::cout << std::setw(30) << std::left << fileName << ":" << std::right << std::setw(4)
            << lineNum << std::endl;
  std::cout << std::left << std::string(message) << std::endl;
  std::cout << "Mesh Contains:                                        " << std::endl;
  std::cout << "------------------------------------------------------" << std::endl;
  //DebugPrintExtents();
  PrintLabel("NumVertices");
  PrintIndexType(this->NumVertices);
  std::cout << std::endl;
  PrintLabel("NumLogSteps");
  PrintIndexType(this->NumLogSteps);
  std::cout << std::endl;
  PrintIndices("Sort Indices", this->SortIndices);
  PrintIndices("Sort Order", this->SortOrder);
  std::cout << std::endl;
#else
  // Avoid unused parameter warning
  (void)message;
  (void)fileName;
  (void)lineNum;
#endif
} // DebugPrint()

template <typename T, typename StorageType>
inline void DataSetMesh::DebugPrintValues(const vtkm::cont::ArrayHandle<T, StorageType>& values)
{
#ifdef DEBUG_PRINT
  if (MeshSize[0] > 0)
  {
    PrintLabelledDataBlock<T, StorageType>("Value", values, MeshSize[0]);
    PrintSortedValues("Sorted Values", values, this->SortOrder);
  }
  PrintHeader(values.GetNumberOfValues());
#else
  // Avoid unused parameter warning
  (void)values;
#endif
} // DebugPrintValues

} // namespace contourtree_augmented
} // worklet
} // vtkm

// Include specialized mesh classes providing triangulation/connectivity information
#include <vtkm/worklet/contourtree_augmented/meshtypes/DataSetMeshTriangulation2DFreudenthal.h>
#include <vtkm/worklet/contourtree_augmented/meshtypes/DataSetMeshTriangulation3DFreudenthal.h>
#include <vtkm/worklet/contourtree_augmented/meshtypes/DataSetMeshTriangulation3DMarchingCubes.h>

#endif
