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
//================================================================================
//
//  Parallel Peak Pruning v. 2.0
//
//  Started June 15, 2017
//
// Copyright Hamish Carr, University of Leeds
//
// TreeResidue.h - A data structure storing the residue information for transfer
//                to the grafting phase
//
//================================================================================
//
// COMMENTS:
//
//
//================================================================================


#ifndef vtk_m_worklet_contourtree_distributed_tree_grafter_h
#define vtk_m_worklet_contourtree_distributed_tree_grafter_h

#include <vtkm/Types.h>
#include <vtkm/worklet/contourtree_augmented/PrintVectors.h>
#include <vtkm/worklet/contourtree_augmented/Types.h>

#include <vtkm/worklet/contourtree_augmented/ContourTree.h>
#include <vtkm/worklet/contourtree_distributed/HierarchicalContourTree.h>
#include <vtkm/worklet/contourtree_distributed/InteriorForest.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CollapseRegularChainsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CopyFirstHypernodePerIterationWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CopyFirstSupernodePerIterationWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CopyNewHypernodesWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CopyNewNodesSetSuperparentsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CopyNewSupernodesSetSuperchildrenWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/CopyNewSupernodesWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/FindCriticalPointsFindLeafsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/FindCriticalPointsFindSaddlesWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/FindCriticalPointsFindTerminalElementsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/FindCriticalPointsSetUpDownNeighboursWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/GetHierarchicalIdsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/GraftInteriorForestsSetTransferIterationWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/HyperNodeWhenComparator.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/IdentifyLeafHyperarcsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/InitActiceSuperarcIdWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/InitActiceSuperarcsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/ListNewNodesCopyIdsWorklet.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/NewHypernodePredicate.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/NewNodePredicate.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/PermuteComparator.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/SuperNodeWhenComparator.h>
#include <vtkm/worklet/contourtree_distributed/tree_grafter/SuperarcWasNotTransferredPredicate.h>

#include <sstream>
#include <string>
#include <utility>


namespace vtkm
{
namespace worklet
{
namespace contourtree_distributed
{

/// \brief Graft the InteriorForest (i.e., the residue of a BRACT) onto a hierarchical tree
template <typename MeshType, typename FieldType>
class TreeGrafter
{ // class TreeGrafter
public:
  // pointers to the related data structures
  MeshType* Mesh;
  vtkm::worklet::contourtree_augmented::ContourTree& ContourTree;
  vtkm::worklet::contourtree_distributed::InteriorForest* InteriorForest;

  //  arrays sized to all regular vertices - this may not be necessary, but is robust
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalTreeId;

  // flags for type of supernode
  vtkm::worklet::contourtree_augmented::IdArrayType SupernodeType;

  // new supernode Ids for each supernode
  vtkm::worklet::contourtree_augmented::IdArrayType NewSupernodeId;

  // maps supernode Ids to regular Ids in parent hierarchical tree, if any
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalRegularId;
  // does the same to supernode Ids, if any
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalSuperId;
  // and for superparents
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalSuperparent;
  // does the same for hypernode Ids, if any
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalHyperId;
  // this array tracks which superarc we insert into / belong on
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalHyperparent;
  // this one tracks what the hyperarc points to
  vtkm::worklet::contourtree_augmented::IdArrayType HierarchicalHyperarc;
  // this array is for tracking when we are transferred
  vtkm::worklet::contourtree_augmented::IdArrayType WhenTransferred;

  // upwards & downwards neighbours for collapsing superarcs
  vtkm::worklet::contourtree_augmented::IdArrayType UpNeighbour;
  vtkm::worklet::contourtree_augmented::IdArrayType DownNeighbour;

  // active supernode set used for re-constructing hyperstructure
  vtkm::worklet::contourtree_augmented::EdgePairArray ActiveSuperarcs;

  // arrays holding the nodes, supernodes and hypernodes that need to be transferred
  vtkm::worklet::contourtree_augmented::IdArrayType NewNodes;
  vtkm::worklet::contourtree_augmented::IdArrayType NewSupernodes;
  vtkm::worklet::contourtree_augmented::IdArrayType NewHypernodes;

  // variable for tracking # of iterations needed in transfer
  vtkm::Id NumTransferIterations;

  /// constructor
  TreeGrafter<MeshType, FieldType>(
    MeshType* mesh,
    vtkm::worklet::contourtree_augmented::ContourTree& contourTree,
    vtkm::worklet::contourtree_distributed::InteriorForest* interiorForest)
    : Mesh(mesh)
    , ContourTree(contourTree)
    , InteriorForest(interiorForest)
  { // constructor
  } // constructor

  /// routine to graft the InteriorForest residue from the BoundaryTree computation into the tree. Previously called GraftResidue
  /// @param[in] theRound The reducting round we are in
  /// @param[in] hierarchicalTree Reference to the hierarchical tree
  /// @param[in] meshDataValues Data values associated with the mesh. This is mesh.SortedValues in the case of
  ///            a ContourTreeMesh and the original data values in the case of a Mesh_DEM_Triangulation mesh.
  ///            Needed for GetHierarchicalIds.
  /// @param[in] localToGlobalIdRelabeler IdRelabeler for the mesh needed to call
  ///            this->Mesh->GetGlobalIdsFromMeshIndices(...) and this->Mesh->GetGlobalIdsFroSortIndices(...)
  ///            If this->Mesh is a ContourTreeMesh then the IdRelabeler is not needed and we can
  ///            simply set it to a nullptr.  Needed for  GetHierarchicalIds.  (default=nullptr).
  template <typename StorageTag>
  void GraftInteriorForests(
    vtkm::Id theRound,
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
    const vtkm::cont::ArrayHandle<FieldType, StorageTag>& meshDataValues,
    const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler =
      nullptr);


  /// Routine to convert supernode IDs from global to IDs in the existing hierarchical tree
  /// @param[in] hierarchicalTree Reference to the hierarchical tree
  /// @param[in] meshDataValues Data values associated with the mesh. This is mesh.SortedValues in the case of
  ///            a ContourTreeMesh and the original data values in the case of a Mesh_DEM_Triangulation mesh.
  /// @param[in] localToGlobalIdRelabeler IdRelabeler for the mesh needed to call
  ///            this->Mesh->GetGlobalIdsFromMeshIndices(...) and this->Mesh->GetGlobalIdsFroSortIndices(...)
  ///            If this->Mesh is a ContourTreeMesh then the IdRelabeler is not needed and we can
  ///            simply set it to a nullptr.
  template <typename StorageTag>
  void GetHierarchicalIds(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
    const vtkm::cont::ArrayHandle<FieldType, StorageTag>& meshDataValues,
    const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler =
      nullptr);

  /// sets up an active superarc set
  void InitializeActiveSuperarcs();

  /// find the critical points in what's left
  void FindCriticalPoints();

  /// pointer-double to collapse chains
  void CollapseRegularChains();

  /// routine to identify one iteration worth of leaves
  void IdentifyLeafHyperarcs();

  ///  6.  Compress arrays & repeat
  void CompressActiveArrays();

  /// Makes a list of new hypernodes, and maps their old IDs to their new ones
  void ListNewHypernodes(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree);

  /// Makes a list of new supernodes, and maps their old IDs to their new ones
  void ListNewSupernodes(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree);

  /// Makes a list of new nodes, and maps their old IDs to their new ones
  /// @param[in] hierarchicalTree Reference to the hierarchical tree
  /// @param[in] localToGlobalIdRelabeler IdRelabeler for the mesh needed to call
  ///            this->Mesh->GetGlobalIdsFromMeshIndices(...) and this->Mesh->GetGlobalIdsFroSortIndices(...)
  ///            If this->Mesh is a ContourTreeMesh then the IdRelabeler is not needed and we can
  ///            simply set it to a nullptr.
  void ListNewNodes(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
    const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler =
      nullptr);

  /// Copies in the hypernodes, now that we have correct super IDs
  void CopyNewHypernodes(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree);

  /// Copies in the supernodes, now that we have correct regular IDs
  void CopyNewSupernodes(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
    vtkm::Id theRound);

  /// Copies the regular nodes in, setting all arrays except superparents
  /// Must be called LAST since it depends on the hypernodes & supernodes that have just been added
  /// in order to resolve the superparents
  /// @param[in] hierarchicalTree Reference to the hierarchical tree
  /// @param[in] meshDataValues Data values associated with the mesh. This is mesh.SortedValues in the case of
  ///            a ContourTreeMesh and the original data values in the case of a Mesh_DEM_Triangulation mesh.
  /// @param[in] localToGlobalIdRelabeler IdRelabeler for the mesh needed to call
  ///            this->Mesh->GetGlobalIdsFromMeshIndices(...) and this->Mesh->GetGlobalIdsFroSortIndices(...)
  ///            If this->Mesh is a ContourTreeMesh then the IdRelabeler is not needed and we can
  ///            simply set it to a nullptr.
  template <typename StorageTag>
  void CopyNewNodes(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
    const vtkm::cont::ArrayHandle<FieldType, StorageTag>& meshDataValues,
    const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler =
      nullptr);

  /// Transfers the details of nodes used in each iteration
  void CopyIterationDetails(
    vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
    vtkm::Id theRound);

  /// prints the contents of the object in a standard format
  std::string DebugPrint(const char* message, const char* fileName, long lineNum);

private:
  /// Used internally to Invoke worklets
  vtkm::cont::Invoker Invoke;

  /// Internal helper function used to resize ArrayHandles since VTKm does not provide a
  /// method to grow a vector without loosing the original data values. The input array
  /// is modified or replaced
  /// @param[in] thearray The 1D array to be resized
  /// @param[in] newSize The new size the array should be changed to
  /// @param[in] fillValue The value to be used to fill the array
  template <typename ValueType>
  static void ResizeVector(vtkm::cont::ArrayHandle<ValueType>& thearray,
                           vtkm::Id newSize,
                           ValueType fillValue)
  {
    vtkm::Id oldSize = thearray.GetNumberOfValues();
    // Simply return if the size of the array does not change
    if (oldSize == newSize)
    {
      return;
    }

    // Resize the array but keep the original values
    thearray.Allocate(newSize, vtkm::CopyFlag::On);

    // Add the fill values to the array if we increased the size of the array
    if (oldSize < newSize)
    {
      vtkm::cont::Algorithm::CopySubRange(
        vtkm::cont::ArrayHandleConstant<ValueType>(fillValue, newSize - oldSize), // copy
        0,                 // start copying from first index
        newSize - oldSize, // num values to copy
        thearray,          // target array to copy to
        oldSize            // start copy to after oldSize
      );
    }
  }
}; // class TreeGrafter


// routine to graft the InteriorForest residue from the BoundaryTree computation into the tree
template <typename MeshType, typename FieldType>
template <typename StorageTag>
void TreeGrafter<MeshType, FieldType>::GraftInteriorForests(
  vtkm::Id theRound,
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
  const vtkm::cont::ArrayHandle<FieldType, StorageTag>& meshDataValues,
  const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler)
{ // GraftInteriorForests()
  // Since all supernodes represented in the bract have been dealt with, this routine needs to
  // identify which supernodes / superarcs need to be added

  // The first step is simply to find out which supernodes are already hierarchical supernodes
  // To do this, we rely on arrays from the TreeGrafter:
  //
  //  We therefore need to do the following:
  //    1.    For each supernode, search by global ID in the hierarchy to determine whether
  //          it is already present, saving the regular and super IDs if it is, NO_SUCH_ELEMENT otherwise
  //    2.    We can then test these IDs to classify:
  //        Super != NSE:          Already present.
  //        Super = NSE, Regular != NSE:  Attachment point, but it is already represented as a regular node
  //        Super = NSE, Regular = NSE:    Free supernode. No additional work required
  //    3.    Reconstruct the hyperstructure from the outside in

  //    1.    For each supernode, search by global ID in the hierarchy to determine
  //          whether it is already present, saving the regular and super IDs if it is, NO_SUCH_ELEMENT otherwise

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, "theRound: " << theRound);
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             this->DebugPrint("Before GraftResidue()", __FILE__, __LINE__));
  VTKM_LOG_S(
    vtkm::cont::LogLevel::Info,
    this->ContourTree.DebugPrint("Contour Tree Before GraftResidue()", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Hier Tree Before GraftResidue()", __FILE__, __LINE__));
  VTKM_LOG_S(
    vtkm::cont::LogLevel::Info,
    this->InteriorForest->DebugPrint("InteriorForest Before GraftResidue()", __FILE__, __LINE__));
#endif

  this->GetHierarchicalIds(hierarchicalTree, meshDataValues, localToGlobalIdRelabeler);

  //  now we need to replicate the merge phase to construct a new hyperstructure
  //  2.  Establish the active set of supernodes & superarcs
  this->InitializeActiveSuperarcs();

  // count the number of iterations
  this->NumTransferIterations = 0;

  //  Now loop to transfer one iteration at a time
  //  We stop when all that is left are attachment points (which aren't included in the active list)
  while (this->ActiveSuperarcs.GetNumberOfValues() > 0)
  { // loop to transfer
    //  3.  Use the write-collision trick to find leaves, regular nodes
    this->FindCriticalPoints();

    //  4.  Chain up/down to find hyperarcs
    this->CollapseRegularChains();

    //  5.  Test for leaves & identify hyperarcs
    // alternating betwen up and down
    // NB: it is therefore possible to have 0 leaves in an iteration, eg if there are no upper leaves to be transferred
    this->IdentifyLeafHyperarcs();

    //  6.  Compress arrays & repeat
    this->CompressActiveArrays();

    //  7.   Update the iteration count
    this->NumTransferIterations++;
  } // loop to transfer

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("Finished Transfer Iterations", __FILE__, __LINE__));
#endif

  //  Now set the transfer iteration for all attachment points
  //  If there were no supernodes to transfer, their types are all NO_SUCH_ELEMENT
  auto setTransferIterationWorklet = vtkm::worklet::contourtree_distributed::tree_grafter::
    GraftInteriorForestsSetTransferIterationWorklet(this->NumTransferIterations);
  this->Invoke(setTransferIterationWorklet,
               this->SupernodeType,       // input
               this->HierarchicalSuperId, // input
               this->WhenTransferred      // output
  );

  // and increment the number of iterations
  this->NumTransferIterations++;

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("Finished Setting Attachment Point Iterations", __FILE__, __LINE__));
#endif
  // The secondary arrays now hold all of the information we need, and we have to transfer it to the hierarchical tree
  // This used to be one single huge function, but has now been broken up for clarity (and because it simplified it)

  // Copying is easiest if we know the mapping of old IDs to new IDs for all regular, super and hyper nodes first, so we establish this
  // (this can be done in any order):
  ListNewHypernodes(hierarchicalTree);
  ListNewSupernodes(hierarchicalTree);
  ListNewNodes(hierarchicalTree, localToGlobalIdRelabeler);

  // Once we have done so, we can transfer them all to the hierarchical tree
  // WARNING! WARNING! WARNING!
  // CopyNewNodes() depends on having CopyNewHypernodes() & CopyNewSupernodes() called first!!!!
  CopyNewHypernodes(hierarchicalTree);
  CopyNewSupernodes(hierarchicalTree, theRound);
  CopyNewNodes(hierarchicalTree, meshDataValues, localToGlobalIdRelabeler);

  // Now we can copy the remaining details to the hierarchical tree, and we are done!
  CopyIterationDetails(hierarchicalTree, theRound);

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("GraftInteriorForests() Completed", __FILE__, __LINE__));
#endif
} // GraftInteriorForests()


/// routine to convert supernode IDs from global to IDs in the existing hierarchical tree
/// Side effects: This function updates:
/// - this->HierarchicalRegularId
/// - this->HierarchicalSuperId
/// - this->HierarchicalSuperparent
/// - this->HierarchicalHyperparent
/// - this->HierarchicalHyperId
///
template <typename MeshType, typename FieldType>
template <typename StorageTag>
void TreeGrafter<MeshType, FieldType>::GetHierarchicalIds(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
  const vtkm::cont::ArrayHandle<FieldType, StorageTag>& meshDataValues,
  const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler)
{ // GetHierarchicalIds()
  // HAC:  This appears to be the preferred idiom for resizing & initializing an array
  // In order for us to build a hierarchical contour tree (HCT), we need to know where in the hierarchical tree a given
  // supernode in the block's contour tree (BCT) belongs, and what its super/hyperparents are
  // The possibilities are:
  //  0.  It's not necessary (in InFo but not attachment)    cannot be in the HCT, so set arrays to NO_SUCH_ELEMENT
  //  1.  It's necessary but not in the HCT:          all of the arrays need to be set to NO_SUCH_ELEMENT
  //  2.  It's in the HCT, but only as a regular node:    regular ID needs to be set, as does superparent, others set to NO_SUCH_ELEMENT
  //  3.  It's in the HCT as a super but not hyper node:    regular/super IDs are set, super/hyperparent are set, hyperID set to NO_SUCH_ELEMENT
  //  4.  It's in the HCT as a hyper node:          all values need to be set
  //
  // The solution adopted is to set all of them to NO_SUCH_ELEMENT by default, and reset each of them as we determine them.

  // We start by resizing all of the arrays to the size of the BCT & setting everything to NO_SUCH_ELEMENT
  {
    auto tempNoSuchElementArray =
      vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                           this->ContourTree.Supernodes.GetNumberOfValues());
    vtkm::cont::Algorithm::Copy(tempNoSuchElementArray, this->HierarchicalRegularId);
    vtkm::cont::Algorithm::Copy(tempNoSuchElementArray, this->HierarchicalSuperId);
    vtkm::cont::Algorithm::Copy(tempNoSuchElementArray, this->HierarchicalSuperparent);
    vtkm::cont::Algorithm::Copy(tempNoSuchElementArray, this->HierarchicalHyperparent);
    vtkm::cont::Algorithm::Copy(tempNoSuchElementArray, this->HierarchicalHyperId);
  }

  // Now, to convert from supernode IDs in the BCT to regular IDs in the HCT, we either need to track IDs forward through the entire computation
  // or we need to be able to look them up.  We chose the latter approach, and therefore need to convert the supernode IDs into global IDs
  // create an array with all of the supernodes
  // NOTE:supernodeGlobalIds may have different type depending on the mesh we use a different smart array handle to avoid memory allocation
  /*
    vtkm::worklet::contourtree_augmented::IdArrayType supernodeGlobalIds;
    vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Supernodes.GetNumberOfValues()),
    supernodeGlobalIds);
    // and ask the mesh to fill it in
    mesh->GetGlobalIDsFromSortIndices(contourTree->supernodes, supernodeGlobalID);
  */
  auto supernodeGlobalIds =
    this->Mesh->GetGlobalIdsFromSortIndices(this->ContourTree.Supernodes, localToGlobalIdRelabeler);

  // retrieve the regular, super IDs (if present)
  auto getHierarchicalIdsWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::GetHierarchicalIdsWorklet();

  auto findRegularByGlobal = hierarchicalTree.GetFindRegularByGlobal();
  auto findSuperArcForUnknownNode = hierarchicalTree.GetFindSuperArcForUnknownNode();

  // TODO: A possible slight optimization would be to use a permutted array for the Mesh->SortedValues/Mesh->SortOrder because we in the ContourTreeMesh we can just do direct lookup. Since in the ContourTreeMesh the SortOrder is handled as a VTKm fancy array the extra cost should not be too bad in terms of memory and compute, but it would help avoid extra function calls and might help the compiler optimize things more.
  this->Invoke(getHierarchicalIdsWorklet,
               // input array
               this->ContourTree.Supernodes,

               // arrays used for reference (read-only)
               supernodeGlobalIds,
               this->Mesh->SortOrder,
               meshDataValues,
               this->InteriorForest->IsNecessary,
               this->InteriorForest->Above,
               this->InteriorForest->Below,
               hierarchicalTree.Superparents,
               hierarchicalTree.Hyperparents,
               hierarchicalTree.Regular2Supernode,
               hierarchicalTree.Super2Hypernode,
               // Execution object to use the FindRegularByGlobal and
               // FindSuperArcForUnknownNode for the hierarchical tree.
               findRegularByGlobal,
               findSuperArcForUnknownNode,

               // arrays used to write output to
               this->HierarchicalRegularId,
               this->HierarchicalSuperId,
               this->HierarchicalHyperId,
               this->HierarchicalSuperparent,
               this->HierarchicalHyperparent);

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("GetHierarchicalIDs() Complete", __FILE__, __LINE__));
#endif
} // GetHierarchicalIDs()


/// sets up an active superarc set
/// Side effects. This function updates:
/// - this->UpNeighbour
/// - this->DownNeighbour
/// - this->ActiveSuperarcs
/// - this->WhenTransferred
/// - this->SupernodeType
/// - this->HierarchicalHyperarc
///
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::InitializeActiveSuperarcs()
{ // InitializeActiveSuperarcs()
  // Resize the up/down neighbours to all supernodes (we won't use all of them, but ...)
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Supernodes.GetNumberOfValues()),
    this->UpNeighbour);
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Supernodes.GetNumberOfValues()),
    this->DownNeighbour);
#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Up & Down Resized", __FILE__, __LINE__));
#endif

  // start by working out a mapping from existing superarc ID to active superarc ID
  // the root superarc, which is guaranteed to be at the end of the array, is omitted
  vtkm::worklet::contourtree_augmented::IdArrayType activeSuperarcId;
  activeSuperarcId.Allocate(this->ContourTree.Supernodes.GetNumberOfValues() - 1);
  {
    // loop to one less, i.e. excluding null superarc from root. tempSuperarcIndex is used as our loop index for the worklet
    auto tempSuperarcIndex =
      vtkm::cont::ArrayHandleIndex(this->ContourTree.Supernodes.GetNumberOfValues() - 1);
    auto initActiceSuperarcIdWorklet =
      vtkm::worklet::contourtree_distributed::tree_grafter::InitActiceSuperarcIdWorklet();
    this->Invoke(initActiceSuperarcIdWorklet,
                 tempSuperarcIndex,                 // input iteration index,
                 this->ContourTree.Superarcs,       // input
                 this->InteriorForest->IsNecessary, // input
                 activeSuperarcId                   // output
    );
  }
  // TODO: Check that it is Ok to use the same array as input and ouput for the partial sum in VTKm
  // TODO: According to the original code this WANTS to be an exclusive_scan / prefix_sum,
  //       but it was not compiling in the orginal code, so this was workaround: see also comment in the following worklet
  //       In VTKm we could change this to a ScanExclusive but using the inclusive scan is fine too
  // compute the new indices for each:
  {
    vtkm::worklet::contourtree_augmented::IdArrayType tempASI;
    vtkm::cont::Algorithm::Copy(activeSuperarcId, tempASI);
    vtkm::cont::Algorithm::ScanInclusive(tempASI, activeSuperarcId);
    // vtkm::cont::Algorithm::ScanInclusive(activeSuperarcId , activeSuperarcId);
  }
  // the final element will hold the result
  vtkm::Id nFree = activeSuperarcId.ReadPortal().Get(activeSuperarcId.GetNumberOfValues() - 1);
  // TODO FIX nFree is 0 here. Check that this is correct. I believe it should be non-zero.
  // resize the active list accordingly
  this->ActiveSuperarcs.Allocate(nFree);

  // Initalize the active superarcs
  {
    // loop to one less, i.e. excluding null superarc from root. tempSuperarcIndex is used as our loop index for the worklet
    auto tempSuperarcIndex =
      vtkm::cont::ArrayHandleIndex(this->ContourTree.Supernodes.GetNumberOfValues() - 1);
    auto initActiceSuperarcsWorklet =
      vtkm::worklet::contourtree_distributed::tree_grafter::InitActiceSuperarcsWorklet();
    this->Invoke(initActiceSuperarcsWorklet,
                 tempSuperarcIndex,                 // input iterator variable
                 this->ContourTree.Superarcs,       // input
                 this->InteriorForest->IsNecessary, // input
                 activeSuperarcId,                  // input
                 this->ActiveSuperarcs              // output
    );
  }

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("Active Superarc Array Initialized", __FILE__, __LINE__));
#endif

  // prepare memory for our transfer arrays
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Supernodes.GetNumberOfValues()),
    this->WhenTransferred);
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Supernodes.GetNumberOfValues()),
    this->SupernodeType);
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Supernodes.GetNumberOfValues()),
    this->HierarchicalHyperarc);

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("InitializeActiveSuperarcs() Complete", __FILE__, __LINE__));
#endif
} // InitializeActiveSuperarcs()


/// find the critical points in what's left
///
/// Side effects. This function updates:
/// - this->UpNeighbour
/// - this->DownNeighbour
/// - this->SupernodeType
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::FindCriticalPoints()
{ // FindCriticalPoints()
  // allocate memory for type of supernode
  this->ResizeVector(this->SupernodeType,
                     this->ContourTree.Supernodes.GetNumberOfValues(),
                     vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  // Reset the UpNeighbour and DownNeighbour array
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->UpNeighbour.GetNumberOfValues()),
    this->UpNeighbour);
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->DownNeighbour.GetNumberOfValues()),
    this->DownNeighbour);

#ifdef DEBUG_PRINT
  // TODO: Hamish: I don't think we need this DebugPrint here.
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("Setting Up/Down Neighbours", __FILE__, __LINE__));
#endif

  // fill in the up/down neighbour arrays
  auto setUpDownNeighboursWorklet = vtkm::worklet::contourtree_distributed::tree_grafter::
    FindCriticalPointsSetUpDownNeighboursWorklet();
  this->Invoke(setUpDownNeighboursWorklet,        // worklet to invoke
               this->ActiveSuperarcs,             // input
               this->InteriorForest->IsNecessary, // input
               this->UpNeighbour,                 // output
               this->DownNeighbour,               // output
               this->SupernodeType                // output
  );

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Up/Down Neighbours Set", __FILE__, __LINE__));
#endif

  // now test whether they match what we expect: if not, we've found a saddle
  auto findSaddlesWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::FindCriticalPointsFindSaddlesWorklet();
  this->Invoke(findSaddlesWorklet,
               this->ActiveSuperarcs,             // input
               this->InteriorForest->IsNecessary, // input
               this->UpNeighbour,                 // input
               this->DownNeighbour,               // input
               this->SupernodeType                // output
  );


#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Saddles Identified", __FILE__, __LINE__));
#endif
  // flag the leaves
  auto findLeafsWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::FindCriticalPointsFindLeafsWorklet();
  this->Invoke(findLeafsWorklet,
               this->ActiveSuperarcs,             // input
               this->InteriorForest->IsNecessary, // input
               this->UpNeighbour,                 // input
               this->DownNeighbour,               // input
               this->SupernodeType                // output (and input)
  );

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Attachments Identified", __FILE__, __LINE__));
#endif

  // one more pass to set terminal flags
  auto findTerminalElementsWorklet = vtkm::worklet::contourtree_distributed::tree_grafter::
    FindCriticalPointsFindTerminalElementsWorklet();
  this->Invoke(findTerminalElementsWorklet,
               this->ActiveSuperarcs, // input
               this->SupernodeType,   // input
               this->UpNeighbour,     // output (and input)
               this->DownNeighbour    // output (and input)
  );

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("FindCriticalPoints() Complete", __FILE__, __LINE__));
#endif

} // FindCriticalPoints()


/// pointer-double to collapse chains
///
/// Side effects. This function updates:
/// - this->UpNeighbour
/// - this->DownNeighbour
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::CollapseRegularChains()
{ // CollapseRegularChains()
  // Compute the number of log steps required in this pass
  vtkm::Id nLogSteps = static_cast<vtkm::Id>(1);
  for (vtkm::Id shifter = this->ActiveSuperarcs.GetNumberOfValues(); shifter != 0; shifter >>= 1)
  {
    nLogSteps++;
  }

  // loop to find the now-regular vertices and collapse past them without altering
  // the existing join & split arcs
  for (vtkm::Id iteration = 0; iteration < nLogSteps; iteration++)
  { // per iteration
    // loop through the vertices, updating up and down
    auto collapseRegularChainsWorklet =
      vtkm::worklet::contourtree_distributed::tree_grafter::CollapseRegularChainsWorklet();
    this->Invoke(collapseRegularChainsWorklet,
                 this->ActiveSuperarcs, // input
                 this->UpNeighbour,     // output (and input)
                 this->DownNeighbour    // output (and input)
    );
  } // per iteration

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("CollapseRegularChains() Complete", __FILE__, __LINE__));
#endif

} // CollapseRegularChains()


/// routine to identify one iteration worth of leaves
///
/// Side effects. This function updates:
/// - this->WhenTransferred
/// - this->HierarchicalHyperarc
/// - this->HierarchicalHyperparent
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::IdentifyLeafHyperarcs()
{ // IdentifyLeafHyperarcs()
  // At this stage, we have:
  //  i.    hierarchicalRegularID set for any supernode stored at all in the parent
  //   ii.   hierarchicalSuperID set for any supernode that is a supernode in the parent
  //  iii.  hierarchicalHyperParent set for any attachment point
  //  iv.    supernodeType set to indicate what type of supernode
  //  v.    up/dn neighbours set for all supernodes

  // at the end of the chain collapse, the up/down neighbours define the start & end of the hyperarc
  // one end may be a leaf, in which case we can transfer the hyperarc
  // note that because we are grafting, we have a guarantee that they can't both be leaves
  // we therefore:
  // a. for leaves, determine whether up or down hyperarc, create hyperarc
  // b. for regular vertices pointing to a leaf hyperarc, set superarc / hyperparent
  // c. for other vertices, ignore
  auto identifyLeafHyperarcsWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::IdentifyLeafHyperarcsWorklet(
      this->NumTransferIterations);
  this->Invoke(identifyLeafHyperarcsWorklet,
               this->ActiveSuperarcs,         // input
               this->SupernodeType,           // input
               this->UpNeighbour,             // input
               this->DownNeighbour,           // input
               this->HierarchicalHyperparent, // output
               this->HierarchicalHyperarc,    // output
               this->WhenTransferred          // output
  );

  // Invariant:  All free supernodes (only) should now have:
  //  Hierarchical Hyperparent set to a non-hierarchical superID
  //  Hierarchical Hyperarc set to the non-hierarchical superID of the target IFF the supernode becomes a hypernode
  //  WARNING!  The other supernodes should all have the hierarchical hyperparent previously set IN HIERARCHICAL superIDs
#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("IdentifyLeafHyperarcs() Complete.", __FILE__, __LINE__));
#endif

} // IdentifyLeafHyperarcs()


///  6.  Compress arrays & repeat
///
/// Side effects. This function updates:
/// - this->ActiveSuperarcs
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::CompressActiveArrays()
{ // CompressActiveArrays()
  // create an array where we can put the compressed array
  vtkm::worklet::contourtree_augmented::EdgePairArray compressedActiveSuperarcs;
  // prediate for deciding which active superarcs to keep
  // NOTE: The original PPP used std::remove_if instead of CopyIf so the predicate inverts the logic, i.e, the predicate indictes
  //       which values to keep rather than which ones to remove
  auto superarcWasTransferredPredicate =
    vtkm::worklet::contourtree_distributed::tree_grafter::SuperarcWasNotTransferredPredicate(
      this->WhenTransferred);
  // compress the array
  vtkm::cont::Algorithm::CopyIf(
    this->ActiveSuperarcs, // compress the active superarcs
    this
      ->ActiveSuperarcs, // stencil. In reality this->WhenTransferred defines the stencil, but we need to lookup the values based on the superacrs itself
    compressedActiveSuperarcs, // array where the comprees active superarcs are stored
    superarcWasTransferredPredicate // unary predicate for deciding which active superarcs are considered true
  );
  // swap in the compressed array
  this->ActiveSuperarcs = compressedActiveSuperarcs;

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("CompressActiveArrays() Complete", __FILE__, __LINE__));
#endif
} // CompressActiveArrays()


/// Makes a list of new hypernodes, and maps their old IDs to their new ones
/// Side effects. This function updates:
/// - this->NewHypernodes
/// - this->HierarchicalHyperId
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::ListNewHypernodes(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree)
{ // ListNewHypernodes()
  //  A.  Start with the list of all supernodes in the non-hierarchical tree
  // NOTE: In contrast to the orignial code we directly initalize with iota instead of with NO_SUCH_ELEMENT first
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::ArrayHandleIndex(this->ContourTree.Supernodes.GetNumberOfValues()),
    this->NewHypernodes);

  //  B.  Remove any which already have a hyper ID in the hierarchical tree
  vtkm::worklet::contourtree_augmented::IdArrayType compressedNewHypernodes;
  // NOTE: The original code used std::remove_if. Since we use CopyIf here we need to invert the predicat and check for which ones to keep not which ones to remove
  auto notANewHypernodePredicate =
    vtkm::worklet::contourtree_distributed::tree_grafter::NewHypernodePredicate();
  vtkm::cont::Algorithm::CopyIf(
    this->NewHypernodes,        // compress the active superarcs
    this->HierarchicalHyperarc, // stencil.
    compressedNewHypernodes,    // array where the comprees new hypernodes are stored
    notANewHypernodePredicate // unary predicate for deciding which active hypernodes are considered true
  );
  this->NewHypernodes = compressedNewHypernodes; // swap in the compressed array

  //  C.  Sort them by iteration, tiebreaking on ID to make it canonical
  auto hyperNodeWhenComparator =
    vtkm::worklet::contourtree_distributed::tree_grafter::HyperNodeWhenComparator(
      this->WhenTransferred);
  vtkm::cont::Algorithm::Sort(this->NewHypernodes, hyperNodeWhenComparator);

  if (this->NewHypernodes.GetNumberOfValues() == 0)
  {
#ifdef DEBUG_PRINT
    VTKM_LOG_S(vtkm::cont::LogLevel::Info,
               "TreeGrafter::ListNewHypernodes(): No new hypernodes. Returning.");
#endif
    return;
  }

  //  D.  Use this sorted array to set the hierarchical hyper index for each supernode that is a new hypernode
  vtkm::Id nOldHypernodes = hierarchicalTree.Hypernodes.GetNumberOfValues();
  // VTKm copy can't allocate for transformed arrays, but this->HierarchicalHyperId.Allocate(nOldHypernodes) has already been allocate earlier.
  auto permutedHierarchicalHyperId =
    vtkm::cont::make_ArrayHandlePermutation(this->NewHypernodes,      // id array,
                                            this->HierarchicalHyperId // value array to copy to
    );
  auto tempNewHierarchicalHyperIdValues = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
    nOldHypernodes, 1, this->NewHypernodes.GetNumberOfValues());
  vtkm::cont::Algorithm::Copy(
    // copy nOldHypernodes + newHypernode
    tempNewHierarchicalHyperIdValues,
    // to hierarchicalHyperID[newHypernodes[newHypernode]]
    permutedHierarchicalHyperId);

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("New Hypernodes Listed", __FILE__, __LINE__));
#endif
} // ListNewHypernodes()


/// Makes a list of new supernodes, and maps their old IDs to their new ones
/// Side effects. This function updates:
/// - this->NewSupernodes
/// - this->HierarchicalSuperID
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::ListNewSupernodes(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree)
{ // ListNewSupernodes()

  //  A.  Start with the list of all supernodes in the non-hierarchical tree
  // NOTE: In contrast to the orignial code we directly initalize with iota instead of with NO_SUCH_ELEMENT first
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::ArrayHandleIndex(this->ContourTree.Supernodes.GetNumberOfValues()),
    this->NewSupernodes);
  //  B.  Remove any which are already supernodes in the hierarchical tree
  //     Only new supernodes will have had whenTransferred set, so this is easy to test
  vtkm::worklet::contourtree_augmented::IdArrayType compressedNewSupernodes;
  // NOTE: We here can reuse the NewHypernodePredicate because it does the same, only the stencil changes.
  //       I.e., the predicate applys the NoSuchElement function to the stencil value and returns it as a bool
  //       Similar to ListNewHypernodes the predicate is inverted compared to the orginal because we here use
  //       CopyIf instead of remove_if in the original code
  auto notANewSupernodePredicate =
    vtkm::worklet::contourtree_distributed::tree_grafter::NewHypernodePredicate();
  vtkm::cont::Algorithm::CopyIf(
    this->NewSupernodes,      // compress the active superarcs
    this->WhenTransferred,    // stencil.
    compressedNewSupernodes,  // array where the compressed new supernodes
    notANewSupernodePredicate // unary predicate for deciding which supernodes are considered true
  );
  this->NewSupernodes = compressedNewSupernodes; // swap in the compressed array

  if (this->NewSupernodes.GetNumberOfValues() == 0)
  {
#ifdef DEBUG_PRINT
    VTKM_LOG_S(vtkm::cont::LogLevel::Info,
               "TreeGrafter::ListNewSupernodes(): No new supernodes. Returning.");
#endif
    return;
  }

  //  C.  Sort them to match the hyperarc sort: note that the supernodes array ALWAYS holds a sort index into the nodes
  auto superNodeWhenComparator =
    vtkm::worklet::contourtree_distributed::tree_grafter::SuperNodeWhenComparator(
      this->WhenTransferred,
      this->HierarchicalHyperparent,
      this->HierarchicalHyperId,
      this->HierarchicalHyperarc,
      this->ContourTree.Supernodes,
      this->SupernodeType);
  vtkm::cont::Algorithm::Sort(this->NewSupernodes, superNodeWhenComparator);
  // D.  Now we set the hierarchical super index which we need for subsequent writes
  vtkm::Id nOldSupernodes = hierarchicalTree.Supernodes.GetNumberOfValues();
  // VTKm copy can't allocate for transformed arrays, but this->HierarchicalSuperId.Allocate(nOldHypernodes) has already been allocate earlier.
  auto permutedHierarchicalSuperId =
    vtkm::cont::make_ArrayHandlePermutation(this->NewSupernodes,        // id array,
                                            this->HierarchicalSuperId); // value array to copy to
  auto tempNewHierarchicalSuperIdValues = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
    nOldSupernodes, 1, this->NewSupernodes.GetNumberOfValues());
  vtkm::cont::Algorithm::Copy(
    // copy nOldSupernodes + newSupernode
    tempNewHierarchicalSuperIdValues,
    // to hierarchicalSuperID[newSupernodes[newSupernode]]
    permutedHierarchicalSuperId);
#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("New Supernodes Listed", __FILE__, __LINE__));
#endif
} // ListNewSupernodes()


/// Makes a list of new nodes, and maps their old IDs to their new ones
/// Side effe cts. This function updates:
/// - this->HierarchicalTreeId
/// - this->NewNodes
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::ListNewNodes(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
  const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler)
{ // ListNewNodes()
  //  A.  Initialise the array that maps regular IDs to "none"
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::make_ArrayHandleConstant(vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT,
                                         this->ContourTree.Nodes.GetNumberOfValues()),
    this->HierarchicalTreeId);

  //  B.  Set the ID correctly for every regular node
  //    They will not all be in the hierarchical tree, so NO_SUCH_ELEMENT will occur, but that is
  //    what we want in this case.  It also means we don't have to set it to NO_SUCH_ELEMENT in section
  //    A., but paranoia indicates we leave that in
  //    This section implements:
  //  for (indexType vertex = 0; vertex < contourTree->nodes.size(); vertex++)
  //  { // per vertex in the bract
  //  // now convert to a global index
  //  indexType globalID = mesh->GetGlobalIDFromMeshIndex(vertex);
  //
  //  // look that one up and store the result (NO_SUCH_ELEMENT is acceptable, but should never occur)
  //  hierarchicalTreeID[vertex] = hierarchicalTree.FindRegularByGlobal(globalID);
  //  } // per vertex in the bract
  // Convert the mesh ids for the contourtree nodes to global ids. This will also be our
  // main field array for the worklet
  auto globalIdsForBoundaryTreeMeshIndices =
    this->Mesh->template GetGlobalIdsFromMeshIndices<vtkm::cont::ArrayHandleIndex>(
      vtkm::cont::ArrayHandleIndex(this->ContourTree.Nodes.GetNumberOfValues()),
      localToGlobalIdRelabeler);
  // Get a FindRegularByGlobal execution object that we can use as an input for worklets to call the function
  auto findRegularByGlobal = hierarchicalTree.GetFindRegularByGlobal();
  // look up our gloabl ids  (NO_SUCH_ELEMENT is acceptable, but should never occur) and
  // copy the regular ids found from global ids in the this->HierarchicalTreeId array
  // NOTE: we should technically be able to just use a ArrayHandleTransrom with findRegularByGlobal and copy the values but it is not clear how to get FindRegularByGlobal to work in both the execution and control environment as ArrayHandleTransform requires ExecutionAndControlObject as base class. The implementation via a worklet is fine but could be made more elegant this way.
  auto listNewNodesCopyIdsWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::ListNewNodesCopyIdsWorklet();
  this->Invoke(listNewNodesCopyIdsWorklet,
               globalIdsForBoundaryTreeMeshIndices, // input global indices
               findRegularByGlobal,                 // input object to call FindRegularByGlobal
               this->HierarchicalTreeId);

  //  C.  Start with the list of all nodes in the non-hierarchical tree
  // NOTE: In contrast to the orignial code we directly initalize with iota instead of with NO_SUCH_ELEMENT first
  vtkm::cont::Algorithm::Copy(
    vtkm::cont::ArrayHandleIndex(this->ContourTree.Nodes.GetNumberOfValues()), this->NewNodes);

  // D.  Copy them if they don't already have an ID set in the hierarchical tree
  // create an array where we can put the compressed array
  vtkm::worklet::contourtree_augmented::IdArrayType compressedNewNodes;
  // prediate for deciding which nodes to keep.
  // NOTE: Similar to ListNewHypernodes the predicate is inverted compared
  //       to the orginal because we here use CopyIf instead of remove_if in the original code
  auto notANewNodePredicate =
    vtkm::worklet::contourtree_distributed::tree_grafter::NewNodePredicate();
  // compress the array
  vtkm::cont::Algorithm::CopyIf(
    this->NewNodes,           // compress the active superarcs
    this->HierarchicalTreeId, // stencil
    compressedNewNodes,       // array where the compressed NewNodes are stored
    notANewNodePredicate      // unary predicate for deciding which nodes are considered true
  );
  // swap in the compressed array
  this->NewNodes = compressedNewNodes;

  if (this->NewNodes.GetNumberOfValues() == 0)
  {
#ifdef DEBUG_PRINT
    VTKM_LOG_S(vtkm::cont::LogLevel::Info,
               "TreeGrafter::ListNewNodes(): No noew nodes. Returning.");
#endif
    return;
  }

  //  E.  And set their new ID for future use
  vtkm::Id nOldNodes = hierarchicalTree.RegularNodeGlobalIds.GetNumberOfValues();
  // VTKm copy can't allocate for transformed arrays, but this->HierarchicalTreeId has already been allocate earlier.
  auto permutedHierarchicalTreeId =
    vtkm::cont::make_ArrayHandlePermutation(this->NewNodes,            // id array,
                                            this->HierarchicalTreeId); // value array to copy to
  auto tempNewHierarchicalTreeIdValues =
    vtkm::cont::ArrayHandleCounting<vtkm::Id>(nOldNodes, 1, this->NewNodes.GetNumberOfValues());
  vtkm::cont::Algorithm::Copy(
    // copy nOldNodes + newNodes
    tempNewHierarchicalTreeIdValues,
    // to hierarchicalTreeId[newNodes[newNode]]
    permutedHierarchicalTreeId);

  // WARNING: FOR NOW, we assume that we don't want to sort the regular nodes, just copy them in
  //  We now have a list of all nodes needing transfer, and a mapping of their IDs
#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("New Nodes Listed", __FILE__, __LINE__));
#endif
} // ListNewNodes()


/// Copies in the hypernodes, now that we have correct super IDs
/// Side effe cts. This function updates:
/// - hierarchicalTree.Hypernodes
/// - hierarchicalTree.Hyperarcs
/// - hierarchicalTree.Superchildren
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::CopyNewHypernodes(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree)
{ // CopyNewHypernodes()
  //  A.  Resize the hyper arrays
  vtkm::Id nOldHypernodes = hierarchicalTree.Hypernodes.GetNumberOfValues();
  vtkm::Id nNewHypernodes = this->NewHypernodes.GetNumberOfValues();
  vtkm::Id totalNHypernodes = nOldHypernodes + nNewHypernodes;
  // Need to resize the vectors while keeping the original values. I.e., we must do a true resize.
  // VTKm does not provide a real resize so we need to do our own.
  {
    // Resize array to length totalNHypernodes and fill new values with NO_SUCH_ELEMENT (or 0) (while keeping original values)
    // NOTE: hierarchicalTree.Superchildren is initalized here but not used by this function
    this->ResizeVector<vtkm::Id>(hierarchicalTree.Hypernodes,
                                 totalNHypernodes,
                                 vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
    this->ResizeVector<vtkm::Id>(hierarchicalTree.Hyperarcs,
                                 totalNHypernodes,
                                 vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
    this->ResizeVector<vtkm::Id>(
      hierarchicalTree.Superchildren, totalNHypernodes, static_cast<vtkm::Id>(0));
  }
  // B.  Copy in the hypernodes & hyperarcs
  auto copyNewHypernodesWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::CopyNewHypernodesWorklet(nOldHypernodes);
  this->Invoke(copyNewHypernodesWorklet,
               this->NewHypernodes,         // input iteration index.
               this->HierarchicalSuperId,   // input
               this->HierarchicalHyperarc,  // input
               hierarchicalTree.Hypernodes, // output
               hierarchicalTree.Hyperarcs   // output
  );

  // we will get the superchildren to set the size for us
#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("New Hypernodes Copied", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("New Hypernodes Copied", __FILE__, __LINE__));
#endif

} // CopyNewHypernodes()


/// Copies in the supernodes, now that we have correct regular IDs
/// Side effe cts. This function updates:
/// - hierarchicalTree.Supernodes
/// - hierarchicalTree.Superarcs
/// - hierarchicalTree.Hyperparents
/// - hierarchicalTree.Superparents
/// - hierarchicalTree.Super2Hypernode
/// - hierarchicalTree.WhichRound
/// - hierarchicalTree.WhichIteration
/// - this->HierarchicalRegularId
/// - hierarchicalTree.Superchildren
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::CopyNewSupernodes(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
  vtkm::Id theRound)
{ // CopyNewSupernodes()
  //  A.  Resize the relevant arrays
  vtkm::Id nOldSupernodes = hierarchicalTree.Supernodes.GetNumberOfValues();
  vtkm::Id nNewSupernodes = this->NewSupernodes.GetNumberOfValues();
  vtkm::Id totalNSupernodes = nOldSupernodes + nNewSupernodes;
  // Resize array to length totalNHypernodes and fill new values with NO_SUCH_ELEMENT (while keeping original values)
  this->ResizeVector<vtkm::Id>(hierarchicalTree.Supernodes,
                               totalNSupernodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  this->ResizeVector<vtkm::Id>(hierarchicalTree.Superarcs,
                               totalNSupernodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  this->ResizeVector<vtkm::Id>(hierarchicalTree.Hyperparents,
                               totalNSupernodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  this->ResizeVector<vtkm::Id>(hierarchicalTree.Super2Hypernode,
                               totalNSupernodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  this->ResizeVector<vtkm::Id>(hierarchicalTree.WhichRound,
                               totalNSupernodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  this->ResizeVector<vtkm::Id>(hierarchicalTree.WhichIteration,
                               totalNSupernodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);

  // we will need this here, since we need to set some new superparents here for supernodes added
  vtkm::Id nOldNodes = hierarchicalTree.RegularNodeGlobalIds.GetNumberOfValues();
  vtkm::Id nNewNodes = this->NewNodes.GetNumberOfValues();
  vtkm::Id totalNNodes = nOldNodes + nNewNodes;
  this->ResizeVector<vtkm::Id>(hierarchicalTree.Superparents,
                               totalNNodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);

  // B.  Copy in the supernodes, &c.
  auto copyNewSupernodesWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::CopyNewSupernodesWorklet(theRound,
                                                                                   nOldSupernodes);
  this->Invoke(copyNewSupernodesWorklet,
               this->NewSupernodes,             // input and iteration index
               this->ContourTree.Supernodes,    // input
               this->Mesh->SortOrder,           // input
               this->HierarchicalTreeId,        // input
               this->WhenTransferred,           // input
               this->HierarchicalSuperparent,   // input
               this->HierarchicalHyperparent,   // input
               this->HierarchicalSuperId,       // input
               this->HierarchicalHyperId,       // input
               this->HierarchicalHyperarc,      // input
               hierarchicalTree.Supernodes,     // output
               hierarchicalTree.WhichRound,     // output
               hierarchicalTree.WhichIteration, // output
               hierarchicalTree.Superarcs,      // output
               this->HierarchicalRegularId,     // input/output
               hierarchicalTree.Hyperparents,   // input/output
               hierarchicalTree.Superparents    // input/output
  );

  // loop to set the number of superchildren per hyperarc
  auto copyNewSupernodesSetSuperchildrenWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::CopyNewSupernodesSetSuperchildrenWorklet(
      hierarchicalTree.Supernodes.GetNumberOfValues());
  auto newSupernodesIndex = vtkm::cont::ArrayHandleIndex(this->NewSupernodes.GetNumberOfValues());
  this->Invoke(
    copyNewSupernodesSetSuperchildrenWorklet,
    newSupernodesIndex,         // input array starting at 0 to NewSupernodes.GetNumberOfValues();
    hierarchicalTree.Superarcs, //input
    hierarchicalTree.Hyperparents, // input
    hierarchicalTree.Hypernodes,   //input
    hierarchicalTree.Superchildren // output
  );

  // now loop through the hypernodes to set their lookup index from supernodes. What we are doing here is the following:
  // for (indexType newHypernode = hierarchicalTree.hypernodes.size() - newHypernodes.size(); newHypernode < hierarchicalTree.hypernodes.size(); newHypernode++)
  //    hierarchicalTree.super2hypernode[hierarchicalTree.hypernodes[newHypernode]] = newHypernode;
  vtkm::Id startHypernodeIndex =
    hierarchicalTree.Hypernodes.GetNumberOfValues() - this->NewHypernodes.GetNumberOfValues();
  auto newHypernodeIndex = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
    startHypernodeIndex,                                                  // start index
    1,                                                                    // increment
    hierarchicalTree.Hypernodes.GetNumberOfValues() - startHypernodeIndex // number of values
  );
  auto permutedHypernodes =
    vtkm::cont::make_ArrayHandlePermutation(newHypernodeIndex, hierarchicalTree.Hypernodes);
  auto permutedSuper2hypernode =
    vtkm::cont::make_ArrayHandlePermutation(permutedHypernodes, hierarchicalTree.Super2Hypernode);
  if (newHypernodeIndex.GetNumberOfValues())
  { // TODO/FIXME: Can we detect this earlier and save computation time?
    vtkm::cont::Algorithm::Copy(newHypernodeIndex, permutedSuper2hypernode);
  }

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("New Supernodes Copied", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("New Supernodes Copied", __FILE__, __LINE__));
#endif
} // CopyNewSupernodes()



/// Copies the regular nodes in, setting all arrays except superparents
/// Must be called LAST since it depends on the hypernodes & supernodes that have just been added
/// in order to resolve the superparents
///
/// Side effects. This function updates:
/// - hierarchicalTree.RegularNodeGlobalIds
/// - hierarchicalTree.DataValues
/// - hierarchicalTree.RegularNodeSortOrder
/// - hierarchicalTree.Regular2Supernode
/// - hierarchicalTree.Superparents
template <typename MeshType, typename FieldType>
template <typename StorageTag>
void TreeGrafter<MeshType, FieldType>::CopyNewNodes(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
  const vtkm::cont::ArrayHandle<FieldType, StorageTag>& meshDataValues,
  const vtkm::worklet::contourtree_augmented::mesh_dem::IdRelabeler* localToGlobalIdRelabeler)
{ // CopyNewNodes()
  // A.  We resize the hierarchy to fit
  vtkm::Id nOldNodes = hierarchicalTree.RegularNodeGlobalIds.GetNumberOfValues();
  vtkm::Id nNewNodes = this->NewNodes.GetNumberOfValues();
  vtkm::Id totalNNodes = nOldNodes + nNewNodes;

  // A.  We start by finding & copying the global IDs for every regular node
  this->ResizeVector<vtkm::Id>(hierarchicalTree.RegularNodeGlobalIds,
                               totalNNodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  // NOTE: The original code created a separate array newNodesGloablId that was set
  // to NO_SUCH_ELEMENT first but we should only need the fancy array here and save the memory
  auto newNodesGloablId =
    this->Mesh
      ->template GetGlobalIdsFromMeshIndices<vtkm::worklet::contourtree_augmented::IdArrayType>(
        this->NewNodes, localToGlobalIdRelabeler); // this is a fancy array
  vtkm::cont::Algorithm::CopySubRange(
    newNodesGloablId,                      // array to copy
    0,                                     // start index
    newNodesGloablId.GetNumberOfValues(),  // number of values to copy (we need the whole array)
    hierarchicalTree.RegularNodeGlobalIds, // array to copy to
    nOldNodes                              // index where to start copying values to
  );

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Global IDs Copied", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Global IDs Copied", __FILE__, __LINE__));
#endif

  // B.  Next, we transfer the data values
  hierarchicalTree.DataValues.Allocate(totalNNodes, vtkm::CopyFlag::On);

  auto meshValuesPermuted = vtkm::cont::make_ArrayHandlePermutation(this->NewNodes, meshDataValues);
  // copy all of mesh->DataValue(newNodes[newNode]) to the end of hierarchicalTree.DataValues starting at nOldNodes
  vtkm::cont::Algorithm::CopySubRange(meshValuesPermuted,
                                      0,
                                      meshValuesPermuted.GetNumberOfValues(),
                                      hierarchicalTree.DataValues,
                                      nOldNodes);

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Data Values Copied", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Data Values Copied", __FILE__, __LINE__));
#endif

  // C.  Then we add the new array indices to the sort and resort it
  // Resize and initialize hierarchicalTree.RegularNodeSortOrder with NO_SUCH_ELEMENT
  // TODO: We should be able to shortcut this since the last values are set next in the CopySubrange
  this->ResizeVector<vtkm::Id>(hierarchicalTree.RegularNodeSortOrder,
                               totalNNodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  {
    // Do the following: std::iota(hierarchicalTree.regularNodeSortOrder.begin() + nOldNodes, hierarchicalTree.regularNodeSortOrder.end(), nOldNodes);
    auto tempCountingArray = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
      nOldNodes, 1, hierarchicalTree.RegularNodeSortOrder.GetNumberOfValues() - nOldNodes);
    vtkm::cont::Algorithm::CopySubRange(tempCountingArray,
                                        0,
                                        tempCountingArray.GetNumberOfValues(),
                                        hierarchicalTree.RegularNodeSortOrder,
                                        nOldNodes);
  }
  {
    auto permuteComparator =
      vtkm::worklet::contourtree_distributed::tree_grafter::PermuteComparator(
        hierarchicalTree.RegularNodeGlobalIds);
    vtkm::cont::Algorithm::Sort(hierarchicalTree.RegularNodeSortOrder, permuteComparator);
  }
#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("Sort Order Reset", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Sort Order Reset", __FILE__, __LINE__));
  // hierarchicalTree.PrintDotSuperStructure("CopyNewNodes Hierarchical Tree");
#endif

  // D. now loop through the supernodes to set their lookup index from regular IDs
  // Resize and initialize hierarchicalTree.Regular2Supernode with NO_SUCH_ELEMENT
  this->ResizeVector<vtkm::Id>(hierarchicalTree.Regular2Supernode,
                               totalNNodes,
                               vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  {
    // The code in this block does the following in serial
    // for (indexType newSupernode = hierarchicalTree.supernodes.size() - newSupernodes.size(); newSupernode < hierarchicalTree.supernodes.size(); newSupernode++)
    //    hierarchicalTree.regular2supernode[hierarchicalTree.supernodes[newSupernode]] = newSupernode;
    vtkm::Id tempStartIndex =
      hierarchicalTree.Supernodes.GetNumberOfValues() - this->NewSupernodes.GetNumberOfValues();
    vtkm::Id tempNumValues = hierarchicalTree.Supernodes.GetNumberOfValues() - tempStartIndex;
    auto tempNewSupernodeIndex = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
      tempStartIndex, 1, tempNumValues); // start, step, count
    auto regular2SupernodePermuted = vtkm::cont::make_ArrayHandlePermutation(
      vtkm::cont::make_ArrayHandlePermutation(tempNewSupernodeIndex, hierarchicalTree.Supernodes),
      hierarchicalTree.Regular2Supernode);
    if (tempNewSupernodeIndex.GetNumberOfValues())
    { // TODO/FIXME: Can we detect this earlier and save computation time?
      vtkm::cont::Algorithm::Copy(tempNewSupernodeIndex, regular2SupernodePermuted);
    }
  }

  // E.  Now we sort out the superparents
  auto copyNewNodesSetSuperparentsWorklet =
    vtkm::worklet::contourtree_distributed::tree_grafter::CopyNewNodesSetSuperparentsWorklet(
      nOldNodes);
  auto findSuperArcForUnknownNode = hierarchicalTree.GetFindSuperArcForUnknownNode();
  this->Invoke(copyNewNodesSetSuperparentsWorklet,
               this->NewNodes,                        // input and iteration index
               this->Mesh->SortIndices,               // input
               this->Mesh->SortOrder,                 // input
               this->ContourTree.Superparents,        // input
               this->ContourTree.Superarcs,           // input
               this->ContourTree.Supernodes,          // input
               this->HierarchicalRegularId,           // input
               this->HierarchicalTreeId,              // input
               hierarchicalTree.RegularNodeGlobalIds, // input
               hierarchicalTree.DataValues,           // input
               findSuperArcForUnknownNode,            // input
               hierarchicalTree.Superparents          // output
  );

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info, DebugPrint("New Nodes Copied", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("New Nodes Copied", __FILE__, __LINE__));
#endif

} // CopyNewNodes()


/// Transfers the details of nodes used in each iteration
///
/// Side effects. This function updates:
/// - hierarchicalTree.NumRegularNodesInRound
/// - hierarchicalTree.NumSupernodesInRound
/// - hierarchicalTree.NumHypernodesInRound
/// - hierarchicalTree.NumIterations
/// - hierarchicalTree.FirstSupernodePerIteration[static_cast<std::size_t>(theRound)]
/// - hierarchicalTree.FirstHypernodePerIteration[static_cast<std::size_t>(theRound)]
template <typename MeshType, typename FieldType>
void TreeGrafter<MeshType, FieldType>::CopyIterationDetails(
  vtkm::worklet::contourtree_distributed::HierarchicalContourTree<FieldType>& hierarchicalTree,
  vtkm::Id theRound)
{ // CopyIterationDetails()

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Starting CopyIterationDetails()", __FILE__, __LINE__));
#endif

  // update the round counts
  hierarchicalTree.NumRegularNodesInRound.WritePortal().Set(theRound,
                                                            this->NewNodes.GetNumberOfValues());
  hierarchicalTree.NumSupernodesInRound.WritePortal().Set(theRound,
                                                          this->NewSupernodes.GetNumberOfValues());
  hierarchicalTree.NumHypernodesInRound.WritePortal().Set(theRound,
                                                          this->NewHypernodes.GetNumberOfValues());
  // the -1 is because the last iteration is just setting attachment points
  hierarchicalTree.NumIterations.WritePortal().Set(theRound, this->NumTransferIterations - 1);

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Round Counts Updated", __FILE__, __LINE__));
#endif

  // calculate the number of old & total super and hyper nodes
  vtkm::Id nTotalSupernodes = hierarchicalTree.Supernodes.GetNumberOfValues();
  vtkm::Id nNewSupernodes = this->NewSupernodes.GetNumberOfValues();
  vtkm::Id nOldSupernodes = nTotalSupernodes - nNewSupernodes;
  vtkm::Id nTotalHypernodes = hierarchicalTree.Hypernodes.GetNumberOfValues();
  vtkm::Id nNewHypernodes = this->NewHypernodes.GetNumberOfValues();
  vtkm::Id nOldHypernodes = nTotalHypernodes - nNewHypernodes;

#ifdef DEBUG_PRINT
  // TODO: Hamish why do we need this debug print. It looks like the hierarchical tree does not change
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Node Counts Retrieved", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("About to Transfer Iteration Counts", __FILE__, __LINE__));
#endif

  // and set the per round iteration counts. There may be smarter ways of doing this, but . . .
  this->ResizeVector<vtkm::Id>(
    hierarchicalTree.FirstSupernodePerIteration[static_cast<std::size_t>(theRound)],
    this->NumTransferIterations,
    vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  {
    auto copyFirstSupernodePerIterationWorklet =
      vtkm::worklet::contourtree_distributed::tree_grafter::CopyFirstSupernodePerIterationWorklet(
        nOldSupernodes);
    auto newSupernodeIndex = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
      nOldSupernodes, 1, nTotalSupernodes - nOldSupernodes); // fancy iteration index
    this->Invoke(
      copyFirstSupernodePerIterationWorklet,
      newSupernodeIndex,               // input fancy iteration index
      hierarchicalTree.WhichIteration, // input
      hierarchicalTree.FirstSupernodePerIteration[static_cast<std::size_t>(theRound)] // output.
    );
  }

#ifdef DEBUG_PRINT_GRAFT_RESIDUE
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Supernode Iteration Counts Set", __FILE__, __LINE__));
#endif

  // Initalize hierarchicalTree.FirstHypernodePerIteration with NO_SUCH_ELEMENT
  this->ResizeVector<vtkm::Id>(
    hierarchicalTree.FirstHypernodePerIteration[static_cast<std::size_t>(theRound)],
    this->NumTransferIterations,
    vtkm::worklet::contourtree_augmented::NO_SUCH_ELEMENT);
  // copy the approbriat hierarchicalTree.FirstHypernodePerIteration values
  {
    auto copyFirstHypernodePerIterationWorklet =
      vtkm::worklet::contourtree_distributed::tree_grafter::CopyFirstHypernodePerIterationWorklet(
        nOldHypernodes);
    auto newHypernodeIndex = vtkm::cont::ArrayHandleCounting<vtkm::Id>(
      nOldHypernodes, 1, nTotalHypernodes - nOldHypernodes); // fancy iteration index
    this->Invoke(
      copyFirstHypernodePerIterationWorklet,
      newHypernodeIndex,               // input fancy iteration index
      hierarchicalTree.Hypernodes,     // input
      hierarchicalTree.WhichIteration, //input
      hierarchicalTree.FirstHypernodePerIteration[static_cast<std::size_t>(theRound)] //output
    );
  }

  // force the extra one to be one-off-the end for safety
  hierarchicalTree.FirstHypernodePerIteration[static_cast<size_t>(theRound)].WritePortal().Set(
    this->NumTransferIterations - 1, hierarchicalTree.Hypernodes.GetNumberOfValues());

#ifdef DEBUG_PRINT
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             hierarchicalTree.DebugPrint("Hypernode Iteration Counts Set", __FILE__, __LINE__));
  VTKM_LOG_S(vtkm::cont::LogLevel::Info,
             DebugPrint("Iteration Details Copied", __FILE__, __LINE__));
#endif

} // CopyIterationDetails()


// debug routine
template <typename MeshType, typename FieldType>
inline std::string TreeGrafter<MeshType, FieldType>::DebugPrint(const char* message,
                                                                const char* fileName,
                                                                long lineNum)
{ // DebugPrint
  std::stringstream resultStream;
  resultStream << std::endl;
  resultStream << "[CUTHERE]---------------------------------------------" << std::endl;
  resultStream << std::setw(30) << std::left << fileName << ":" << std::right << std::setw(4)
               << lineNum << " ";
  resultStream << std::left << std::string(message) << std::endl;

  resultStream << "------------------------------------------------------" << std::endl;
  resultStream << "Tree Grafter Contains:                                " << std::endl;
  resultStream << "------------------------------------------------------" << std::endl;

  // Regular Vertex Arrays
  vtkm::worklet::contourtree_augmented::PrintHeader(this->HierarchicalTreeId.GetNumberOfValues(),
                                                    resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "ID in Hierarchical Tree", this->HierarchicalTreeId, -1, resultStream);
  resultStream << std::endl;

  // Per Supernode Arrays
  vtkm::worklet::contourtree_augmented::PrintHeader(this->HierarchicalRegularId.GetNumberOfValues(),
                                                    resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Hierarchical Regular ID", this->HierarchicalRegularId, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Hierarchical Superparent", this->HierarchicalSuperparent, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Hierarchical Super ID", this->HierarchicalSuperId, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Hierarchical Hyperparent", this->HierarchicalHyperparent, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Hierarchical Hyper ID", this->HierarchicalHyperId, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Hierarchical Hyperarc", this->HierarchicalHyperarc, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "When Transferred", this->WhenTransferred, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Supernode Type", this->SupernodeType, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Up Neighbour", this->UpNeighbour, -1, resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "Down Neighbour", this->DownNeighbour, -1, resultStream);
  resultStream << std::endl;

  // Active Supernode Arrays
  vtkm::worklet::contourtree_augmented::PrintHeader(this->ActiveSuperarcs.GetNumberOfValues(),
                                                    resultStream);
  vtkm::worklet::contourtree_augmented::PrintEdgePairArray(
    "Active Superarcs", this->ActiveSuperarcs, -1, resultStream);

  // Arrays for transfer to hierarchical tree
  vtkm::worklet::contourtree_augmented::PrintHeader(this->NewHypernodes.GetNumberOfValues(),
                                                    resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "New Hypernodes", this->NewHypernodes, -1, resultStream);

  vtkm::worklet::contourtree_augmented::PrintHeader(this->NewSupernodes.GetNumberOfValues(),
                                                    resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices(
    "New Supernodes", this->NewSupernodes, -1, resultStream);

  vtkm::worklet::contourtree_augmented::PrintHeader(this->NewNodes.GetNumberOfValues(),
                                                    resultStream);
  vtkm::worklet::contourtree_augmented::PrintIndices("New Nodes", this->NewNodes, -1, resultStream);

  resultStream << "------------------------------------------------------" << std::endl;
  resultStream << std::endl;

  resultStream << std::flush;
  return resultStream.str();
} // DebugPrint


} // namespace contourtree_distributed
} // namespace worklet
} // namespace vtkm

#endif
