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

#ifndef vtk_m_worklet_contourtree_distributed_boundary_tree_h
#define vtk_m_worklet_contourtree_distributed_boundary_tree_h

#include <vtkm/Types.h>
#include <vtkm/worklet/contourtree_augmented/PrintVectors.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>
#include <vtkm/worklet/contourtree_augmented/data_set_mesh/IdRelabeler.h>
#include <vtkm/worklet/contourtree_augmented/meshtypes/ContourTreeMesh.h>

#include <sstream>
#include <string>
#include <utility>

namespace vtkm
{
namespace worklet
{
namespace contourtree_distributed
{

/// \brief Boundary Restricted Augmented Contour Tree (BRACT)
///
/// A contour tree for boundary vertices with the interior abstracted.
/// This is primarily a data storage class. The actual constuction of
/// the BoundaryTree performed by the BoundaryTreeMaker
/// As a data store, this class primarily stores a set of arrays
/// and provides convenience functions for interacting with the
/// the data, e.g., to export the data to dot.
class BoundaryTree
{ // class BoundaryTree
public:
  // for each vertex, we store the index
  vtkm::worklet::contourtree_augmented::IdArrayType VertexIndex;

  // and the ID of the vertex it connects to (or NO_SUCH_ELEMENT)
  vtkm::worklet::contourtree_augmented::IdArrayType Superarcs;

  // Total size of the boundary (used for logging only!)
  vtkm::Id NumBoundary;

  // Total number of boundary points used (used for logging only!).
  // If the whole boundary is used then NumBoundaryUsed == NumBoundary
  // but if only the boundary critical points are being used then
  // NumBoundaryUsed >= NumBoundary
  vtkm::Id NumBoundaryUsed;

  // constructor
  BoundaryTree() {}

  // prints the contents of the BRACT for comparison with sweep and merge
  std::string Print();

  // secondary version which takes the mesh as a parameter
  template <typename Mesh, typename FieldArrayType>
  std::string PrintGlobalDot(const char* label,
                             const Mesh& mesh,
                             const FieldArrayType& fieldArray,
                             const vtkm::Id3 blockOrigin,
                             const vtkm::Id3 blockSize,
                             const vtkm::Id3 globalSize) const;

  // prints the contents of the BRACT as a dot file using global IDs (version for CT mesh)
  template <typename FieldType>
  std::string PrintGlobalDot(
    const char* label,
    vtkm::worklet::contourtree_augmented::ContourTreeMesh<FieldType>& mesh) const;

  // prints the contents of the BRACT in debug format
  void PrintContent(std::ostream& outStream) const;
  std::string DebugPrint(const char* message, const char* fileName, long lineNum) const;
  inline std::string PrintArraySizes() const;
}; // class BoundaryTree


// prints the contents of the BRACT for comparison with sweep and merge
std::string BoundaryTree::Print()
{ // Print
  // Use string steam to record text so the user can print it however they like
  std::stringstream resultStream;
  resultStream << "Boundary-Restricted Augmented Contour Tree" << std::endl;
  resultStream << "==========================================" << std::endl;
  // fill it up
  auto superarcsPortal = this->Superarcs.ReadPortal();
  auto vertexIndexPortal = this->VertexIndex.ReadPortal();
  for (vtkm::Id node = 0; node < superarcsPortal.GetNumberOfValues(); node++)
  {
    // retrieve ID of target supernode
    vtkm::Id from = vertexIndexPortal.Get(node);
    vtkm::Id to = superarcsPortal.Get(node);
    // if this is true, it is the last pruned vertex & is omitted
    if (vtkm::worklet::contourtree_augmented::NoSuchElement(to))
    {
      continue;
    }
    // print out the from & to
    resultStream << std::setw(vtkm::worklet::contourtree_augmented::PRINT_WIDTH) << from << " ";
    resultStream << std::setw(vtkm::worklet::contourtree_augmented::PRINT_WIDTH) << to << std::endl;
  }
  return resultStream.str();
} // Print

// secondary version which takes the mesh as a parameter
template <typename Mesh, typename FieldArrayType>
std::string BoundaryTree::PrintGlobalDot(const char* label,
                                         const Mesh& mesh,
                                         const FieldArrayType& fieldArray,
                                         const vtkm::Id3 blockOrigin,
                                         const vtkm::Id3 blockSize,
                                         const vtkm::Id3 globalSize) const
{ // PrintGlobalDot
  VTKM_IS_ARRAY_HANDLE(FieldArrayType);
  std::stringstream resultStream;
  // print the header information
  resultStream << "digraph BRACT" << std::endl;
  resultStream << "\t{" << std::endl;
  resultStream << "\tlabel=\"" << label << "\"\n\tlabelloc=t\n\tfontsize=30" << std::endl;
  // create a relabeler
  vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler relabeler(
    blockOrigin, blockSize, globalSize);

  // loop through all nodes
  auto vertexIndexPortal = this->VertexIndex.ReadPortal();
  auto superarcsPortal = this->Superarcs.ReadPortal();
  auto sortOrderPortal = mesh.SortOrder.ReadPortal();
  auto fieldArrayPortal = fieldArray.ReadPortal();
  for (vtkm::Id node = 0; node < this->Superarcs.GetNumberOfValues(); node++)
  {
    // now convert to mesh IDs from node IDs
    vtkm::Id from = vertexIndexPortal.Get(node);
    // find the local & global IDs & data value
    vtkm::Id fromLocal = sortOrderPortal.Get(from);
    vtkm::Id fromGlobal = relabeler(fromLocal);
    auto fromValue = fieldArrayPortal.Get(fromLocal);

    // print the vertex
    resultStream << node << " [style=filled,fillcolor="
                 << "grey"
                 << ",label=\"" << fromGlobal << "\\nv" << fromValue << "\"];" << std::endl;
  }

  for (vtkm::Id node = 0; node < this->Superarcs.GetNumberOfValues(); node++)
  {
    // retrieve ID of target supernode
    vtkm::Id to = superarcsPortal.Get(node);
    // if this is true, it is the last pruned vertex & is omitted
    if (vtkm::worklet::contourtree_augmented::NoSuchElement(to))
    {
      continue;
    }
    if (node < to)
    {
      resultStream << to << " -> " << node << std::endl;
    }
    else
    {
      resultStream << node << " -> " << to << std::endl;
    }
  }
  resultStream << "\t}" << std::endl;
  // return the result
  return resultStream.str();
} // PrintGlobalDot

// prints the contents of the BRACT as a dot file using global IDs (version for CT mesh)
template <typename FieldType>
std::string BoundaryTree::PrintGlobalDot(
  const char* label,
  vtkm::worklet::contourtree_augmented::ContourTreeMesh<FieldType>& mesh) const
{ //PrintGlobalDot
  std::stringstream resultStream;
  // print the header information
  resultStream << "digraph BRACT\n\t{\n";
  resultStream << "\tsize=\"6.5, 9\"\n\tratio=\"fill\"\n";
  resultStream << "\tlabel=\"" << label << "\"\n\tlabelloc=t\n\tfontsize=30\n" << std::endl;

  // loop through all nodes
  auto vertexIndexPortal = this->VertexIndex.ReadPortal();
  auto globalMeshIndexPortal = mesh.GlobalMeshIndex.ReadPortal();
  auto sortedValuesPortal = mesh.SortedValues.ReadPortal();
  auto superarcsPortal = this->Superarcs.ReadPortal();
  for (vtkm::Id node = 0; node < this->VertexIndex.GetNumberOfValues(); node++)
  { // per node
    // work out the node and it's value
    vtkm::Id meshIndex = vertexIndexPortal.Get(node);
    vtkm::Id from = globalMeshIndexPortal.Get(meshIndex);
    auto fromValue = sortedValuesPortal.Get(meshIndex);
    // print the vertex
    resultStream << node << " [style=filled,fillcolor="
                 << "grey"
                 << ",label=\"" << from << "\\nv" << fromValue << "\"];" << std::endl;
  } // per node


  for (vtkm::Id node = 0; node < this->Superarcs.GetNumberOfValues(); node++)
  { // per node
    // retrieve ID of target supernode
    vtkm::Id to = superarcsPortal.Get(node);
    // if this is true, it is the last pruned vertex & is omitted
    if (vtkm::worklet::contourtree_augmented::NoSuchElement(to))
    {
      continue;
    }
    if (node < to)
    {
      resultStream << to << " -> " << node << std::endl;
    }
    else
    {
      resultStream << node << " -> " << to << std::endl;
    }
  } // per node
  resultStream << "\t}" << std::endl;
  // Return the resulting strin
  return resultStream.str();
} //PrintGlobalDot

// debug routine
void BoundaryTree::PrintContent(std::ostream& outStream) const
{
  vtkm::worklet::contourtree_augmented::PrintHeader(this->VertexIndex.GetNumberOfValues(),
                                                    outStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Vertex Index", this->VertexIndex, -1, outStream);
  vtkm::worklet::contourtree_augmented::PrintIndices("Superarcs", this->Superarcs, -1, outStream);
}

inline std::string BoundaryTree::DebugPrint(const char* message,
                                            const char* fileName,
                                            long lineNum) const
{ // DebugPrint
  std::stringstream resultStream;
#ifdef DEBUG_PRINT
  resultStream << "[CUTHERE]-------------------------------------------------------" << std::endl;
  resultStream << std::setw(30) << std::left << fileName << ":" << std::right << std::setw(4)
               << lineNum << std::endl;
  resultStream << std::left << std::string(message) << std::endl;
  resultStream << "Boundary Restricted Augmented Contour Tree Contains:            " << std::endl;
  resultStream << "----------------------------------------------------------------" << std::endl;

  this->PrintContent(resultStream);

  resultStream << "---------------------------" << std::endl;
  resultStream << std::endl;
  resultStream << std::flush;
#else
  (void)message;
  (void)fileName;
  (void)lineNum;
#endif
  return resultStream.str();
} // DebugPrint

inline std::string BoundaryTree::PrintArraySizes() const
{ // PrintArraySizes
  std::stringstream arraySizeLog;
  arraySizeLog << std::setw(42) << std::left << "    #VertexIndex"
               << ": " << this->VertexIndex.GetNumberOfValues() << std::endl
               << std::setw(42) << std::left << "    #Superarcs"
               << ": " << this->Superarcs.GetNumberOfValues() << std::endl
               << std::setw(42) << std::left << "    #Boundary"
               << ": " << this->NumBoundary << std::endl
               << std::setw(42) << std::left << "    #BoundaryUsed"
               << ": " << this->NumBoundaryUsed << std::endl;
  return arraySizeLog.str();
} // PrintArraySizes



} // namespace contourtree_distributed
} // namespace worklet
} // namespace vtkm

#endif
