//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_exec_arg_ThreadIndicesTopologyMap_h
#define vtk_m_exec_arg_ThreadIndicesTopologyMap_h

#include <vtkm/exec/arg/ThreadIndicesBasic.h>

#include <vtkm/exec/ConnectivityPermuted.h>
#include <vtkm/exec/ConnectivityStructured.h>

namespace vtkm
{
namespace exec
{
namespace arg
{

namespace detail
{

/// Given a \c Vec of (semi) arbitrary size, inflate it to a vtkm::Id3 by padding with zeros.
///
static inline VTKM_EXEC vtkm::Id3 InflateTo3D(vtkm::Id3 index)
{
  return index;
}

/// Given a \c Vec of (semi) arbitrary size, inflate it to a vtkm::Id3 by padding with zeros.
/// \overload
static inline VTKM_EXEC vtkm::Id3 InflateTo3D(vtkm::Id2 index)
{
  return vtkm::Id3(index[0], index[1], 0);
}

/// Given a \c Vec of (semi) arbitrary size, inflate it to a vtkm::Id3 by padding with zeros.
/// \overload
static inline VTKM_EXEC vtkm::Id3 InflateTo3D(vtkm::Vec<vtkm::Id, 1> index)
{
  return vtkm::Id3(index[0], 0, 0);
}

/// Given a \c Vec of (semi) arbitrary size, inflate it to a vtkm::Id3 by padding with zeros.
/// \overload
static inline VTKM_EXEC vtkm::Id3 InflateTo3D(vtkm::Id index)
{
  return vtkm::Id3(index, 0, 0);
}

/// Given a vtkm::Id3, reduce down to an identifier of choice.
///
static inline VTKM_EXEC vtkm::Id3 Deflate(const vtkm::Id3& index, vtkm::Id3)
{
  return index;
}

/// Given a vtkm::Id3, reduce down to an identifier of choice.
/// \overload
static inline VTKM_EXEC vtkm::Id2 Deflate(const vtkm::Id3& index, vtkm::Id2)
{
  return vtkm::Id2(index[0], index[1]);
}

} // namespace detail

/// \brief Uses spaces optimizations when using MaskNone and ScatterIdentity
///
struct DefaultScatterAndMaskTag
{
};

/// \brief Used for when not using MaskNone and ScatterIdentity.
///
struct CustomScatterOrMaskTag
{
};

/// \brief Container for thread indices in a topology map
///
/// This specialization of \c ThreadIndices adds extra indices that deal with
/// topology maps. In particular, it saves the incident element indices. The
/// input and output indices from the superclass are considered to be indexing
/// the visited elements.
///
/// This class is templated on the type that stores the connectivity (such
/// as \c ConnectivityExplicit or \c ConnectivityStructured).
///
template <typename ConnectivityType, typename ScatterAndMaskMode>
class ThreadIndicesTopologyMap : public vtkm::exec::arg::ThreadIndicesBasic
{
  using Superclass = vtkm::exec::arg::ThreadIndicesBasic;

public:
  using IndicesIncidentType = typename ConnectivityType::IndicesType;
  using CellShapeTag = typename ConnectivityType::CellShapeTag;
  using Connectivity = ConnectivityType;

  VTKM_SUPPRESS_EXEC_WARNINGS
  VTKM_EXEC ThreadIndicesTopologyMap(vtkm::Id threadIndex,
                                     vtkm::Id inputIndex,
                                     vtkm::IdComponent visitIndex,
                                     vtkm::Id outputIndex,
                                     const ConnectivityType& connectivity)
    : Superclass(threadIndex, inputIndex, visitIndex, outputIndex)
    // The connectivity is stored in the invocation parameter at the given
    // input domain index. If this class is being used correctly, the type
    // of the domain will match the connectivity type used here. If there is
    // a compile error here about a type mismatch, chances are a worklet has
    // set its input domain incorrectly.
    , IndicesIncident(connectivity.GetIndices(inputIndex))
    , CellShape(connectivity.GetCellShape(inputIndex))
  {
  }

  /// \brief The indices of the incident elements.
  ///
  /// A topology map has "visited" and "incident" elements (e.g. points, cells,
  /// etc). For each worklet invocation, there is exactly one visited element,
  /// but there can be several incident elements. This method returns a Vec-like
  /// object containing the indices to the incident elements.
  ///
  VTKM_EXEC
  const IndicesIncidentType& GetIndicesIncident() const { return this->IndicesIncident; }

  /// \brief The input indices of the incident elements in pointer form.
  ///
  /// Returns the same object as GetIndicesIncident except that it returns a
  /// pointer to the internally held object rather than a reference or copy.
  /// Since the from indices can be a sizeable Vec (8 entries is common), it is
  /// best not to have a bunch a copies. Thus, you can pass around a pointer
  /// instead. However, care should be taken to make sure that this object does
  /// not go out of scope, at which time the returned pointer becomes invalid.
  ///
  VTKM_EXEC
  const IndicesIncidentType* GetIndicesIncidentPointer() const { return &this->IndicesIncident; }

  /// \brief The shape of the input cell.
  ///
  /// In topology maps that map from points to something, the indices make up
  /// the structure of a cell. Although the shape tag is not technically and
  /// index, it defines the meaning of the indices, so we put it here. (That
  /// and this class is the only convenient place to store it.)
  ///
  VTKM_EXEC
  CellShapeTag GetCellShape() const { return this->CellShape; }

private:
  IndicesIncidentType IndicesIncident;
  CellShapeTag CellShape;
};

/// \brief Specialization for CustomScatterOrMaskTag
template <typename VisitTopology, typename IncidentTopology, vtkm::IdComponent Dimension>
class ThreadIndicesTopologyMap<
  vtkm::exec::ConnectivityStructured<VisitTopology, IncidentTopology, Dimension>,
  CustomScatterOrMaskTag>
{
  using ConnectivityType =
    vtkm::exec::ConnectivityStructured<VisitTopology, IncidentTopology, Dimension>;

public:
  using IndicesIncidentType = typename ConnectivityType::IndicesType;
  using CellShapeTag = typename ConnectivityType::CellShapeTag;
  using LogicalIndexType = typename ConnectivityType::SchedulingRangeType;
  using Connectivity =
    vtkm::exec::ConnectivityStructured<VisitTopology, IncidentTopology, Dimension>;

  VTKM_EXEC ThreadIndicesTopologyMap(vtkm::Id threadIndex,
                                     vtkm::Id inputIndex,
                                     vtkm::IdComponent visitIndex,
                                     vtkm::Id outputIndex,
                                     const ConnectivityType& connectivity)
  {
    this->ThreadIndex = threadIndex;
    this->InputIndex = inputIndex;
    this->VisitIndex = visitIndex;
    this->OutputIndex = outputIndex;
    this->LogicalIndex = connectivity.FlatToLogicalToIndex(this->InputIndex);
    this->IndicesIncident = connectivity.GetIndices(this->LogicalIndex);
    this->CellShape = connectivity.GetCellShape(this->InputIndex);
  }

  VTKM_EXEC ThreadIndicesTopologyMap(const vtkm::Id3& threadIndex3D,
                                     vtkm::Id threadIndex1D,
                                     const ConnectivityType& connectivity)
  {
    // This constructor handles multidimensional indices on one-to-one input-to-output
    auto logicalIndex = detail::Deflate(threadIndex3D, LogicalIndexType());
    this->ThreadIndex = threadIndex1D;
    this->LogicalIndex = logicalIndex;
    this->IndicesIncident = connectivity.GetIndices(logicalIndex);
    this->CellShape = connectivity.GetCellShape(threadIndex1D);
  }

  VTKM_EXEC ThreadIndicesTopologyMap(const vtkm::Id3& threadIndex3D,
                                     vtkm::Id threadIndex1D,
                                     vtkm::Id inIndex,
                                     vtkm::IdComponent visitIndex,
                                     vtkm::Id outIndex,
                                     const ConnectivityType& connectivity)
  {
    // This constructor handles multidimensional indices on many-to-many input-to-output
    auto logicalIndex = detail::Deflate(threadIndex3D, LogicalIndexType());
    this->ThreadIndex = threadIndex1D;
    this->InputIndex = inIndex;
    this->VisitIndex = visitIndex;
    this->OutputIndex = outIndex;
    this->LogicalIndex = logicalIndex;
    this->IndicesIncident = connectivity.GetIndices(logicalIndex);
    this->CellShape = connectivity.GetCellShape(threadIndex1D);
  }


  /// \brief The index of the thread or work invocation.
  ///
  /// This index refers to which instance of the worklet is being invoked. Every invocation of the
  /// worklet has a unique thread index. This is also called the work index depending on the
  /// context.
  ///
  VTKM_EXEC
  vtkm::Id GetThreadIndex() const { return this->ThreadIndex; }

  /// \brief The logical index into the input domain.
  ///
  /// This is similar to \c GetIndex3D except the Vec size matches the actual
  /// dimensions of the data.
  ///
  VTKM_EXEC
  LogicalIndexType GetIndexLogical() const { return this->LogicalIndex; }

  /// \brief The index into the input domain.
  ///
  /// This index refers to the input element (array value, cell, etc.) that
  /// this thread is being invoked for. This is the typical index used during
  /// fetches.
  ///
  VTKM_EXEC
  vtkm::Id GetInputIndex() const { return this->InputIndex; }

  /// \brief The 3D index into the input domain.
  ///
  /// Overloads the implementation in the base class to return the 3D index
  /// for the input.
  ///
  VTKM_EXEC
  vtkm::Id3 GetInputIndex3D() const { return detail::InflateTo3D(this->GetIndexLogical()); }

  /// \brief The index into the output domain.
  ///
  /// This index refers to the output element (array value, cell, etc.) that
  /// this thread is creating. This is the typical index used during
  /// Fetch::Store.
  ///
  VTKM_EXEC
  vtkm::Id GetOutputIndex() const { return this->OutputIndex; }

  /// \brief The visit index.
  ///
  /// When multiple output indices have the same input index, they are
  /// distinguished using the visit index.
  ///
  VTKM_EXEC
  vtkm::IdComponent GetVisitIndex() const { return this->VisitIndex; }

  /// \brief The indices of the incident elements.
  ///
  /// A topology map has "visited" and "incident" elements (e.g. points, cells,
  /// etc). For each worklet invocation, there is exactly one visited element,
  /// but there can be several incident elements. This method returns a
  /// Vec-like object containing the indices to the incident elements.
  ///
  VTKM_EXEC
  const IndicesIncidentType& GetIndicesIncident() const { return this->IndicesIncident; }

  /// \brief The input indices of the incident elements in pointer form.
  ///
  /// Returns the same object as GetIndicesIncident except that it returns a
  /// pointer to the internally held object rather than a reference or copy.
  /// Since the from indices can be a sizeable Vec (8 entries is common), it is
  /// best not to have a bunch a copies. Thus, you can pass around a pointer
  /// instead. However, care should be taken to make sure that this object does
  /// not go out of scope, at which time the returned pointer becomes invalid.
  ///
  VTKM_EXEC
  const IndicesIncidentType* GetIndicesIncidentPointer() const { return &this->IndicesIncident; }

  /// \brief The shape of the input cell.
  ///
  /// In topology maps that map from points to something, the indices make up
  /// the structure of a cell. Although the shape tag is not technically and
  /// index, it defines the meaning of the indices, so we put it here. (That
  /// and this class is the only convenient place to store it.)
  ///
  VTKM_EXEC
  CellShapeTag GetCellShape() const { return this->CellShape; }

private:
  vtkm::Id ThreadIndex;
  vtkm::IdComponent VisitIndex;
  LogicalIndexType LogicalIndex;
  IndicesIncidentType IndicesIncident;
  CellShapeTag CellShape;
  vtkm::Id InputIndex;
  vtkm::Id OutputIndex;
};

/// \brief Specialization for DefaultScatterAndMaskTag
///
/// It does not store VisitIndex, InputIndex and OutputIndex
/// since this is used only when Scatter is set as ScatterIdentity
/// and Mask is set as MaskNone which does not performs any transformation onto the
/// indices.
///
template <typename VisitTopology, typename IncidentTopology, vtkm::IdComponent Dimension>
class ThreadIndicesTopologyMap<
  vtkm::exec::ConnectivityStructured<VisitTopology, IncidentTopology, Dimension>,
  DefaultScatterAndMaskTag>
{
  using ConnectivityType =
    vtkm::exec::ConnectivityStructured<VisitTopology, IncidentTopology, Dimension>;

public:
  using IndicesIncidentType = typename ConnectivityType::IndicesType;
  using CellShapeTag = typename ConnectivityType::CellShapeTag;
  using LogicalIndexType = typename ConnectivityType::SchedulingRangeType;
  using Connectivity = ConnectivityType;

  VTKM_EXEC ThreadIndicesTopologyMap(vtkm::Id threadIndex,
                                     vtkm::Id inputIndex,
                                     vtkm::IdComponent vtkmNotUsed(visitIndex),
                                     vtkm::Id vtkmNotUsed(outputIndex),
                                     const ConnectivityType& connectivity)
  {
    this->ThreadIndex = threadIndex;
    this->LogicalIndex = connectivity.FlatToLogicalToIndex(inputIndex);
    this->IndicesIncident = connectivity.GetIndices(this->LogicalIndex);
    this->CellShape = connectivity.GetCellShape(inputIndex);
  }




  VTKM_EXEC ThreadIndicesTopologyMap(const vtkm::Id3& threadIndex3D,
                                     vtkm::Id threadIndex1D,
                                     const ConnectivityType& connectivity)
  {
    // This constructor handles multidimensional indices on one-to-one input-to-output
    auto logicalIndex = detail::Deflate(threadIndex3D, LogicalIndexType());
    this->ThreadIndex = threadIndex1D;
    this->LogicalIndex = logicalIndex;
    this->IndicesIncident = connectivity.GetIndices(logicalIndex);
    this->CellShape = connectivity.GetCellShape(threadIndex1D);
  }

  VTKM_EXEC ThreadIndicesTopologyMap(const vtkm::Id3& threadIndex3D,
                                     vtkm::Id threadIndex1D,
                                     vtkm::Id vtkmNotUsed(inIndex),
                                     vtkm::IdComponent vtkmNotUsed(visitIndex),
                                     vtkm::Id vtkmNotUsed(outIndex),
                                     const ConnectivityType& connectivity)
  {
    // This constructor handles multidimensional indices on many-to-many input-to-output
    auto logicalIndex = detail::Deflate(threadIndex3D, LogicalIndexType());
    this->ThreadIndex = threadIndex1D;
    this->LogicalIndex = logicalIndex;
    this->IndicesIncident = connectivity.GetIndices(logicalIndex);
    this->CellShape = connectivity.GetCellShape(threadIndex1D);
  }

  /// \brief The index of the thread or work invocation.
  ///
  /// This index refers to which instance of the worklet is being invoked. Every invocation of the
  /// worklet has a unique thread index. This is also called the work index depending on the
  /// context.
  ///
  VTKM_EXEC
  vtkm::Id GetThreadIndex() const { return this->ThreadIndex; }

  /// \brief The logical index into the input domain.
  ///
  /// This is similar to \c GetIndex3D except the Vec size matches the actual
  /// dimensions of the data.
  ///
  VTKM_EXEC
  LogicalIndexType GetIndexLogical() const { return this->LogicalIndex; }

  /// \brief The index into the input domain.
  ///
  /// This index refers to the input element (array value, cell, etc.) that
  /// this thread is being invoked for. This is the typical index used during
  /// fetches.
  ///
  VTKM_EXEC
  vtkm::Id GetInputIndex() const { return this->ThreadIndex; }

  /// \brief The 3D index into the input domain.
  ///
  /// Overloads the implementation in the base class to return the 3D index
  /// for the input.
  ///
  VTKM_EXEC
  vtkm::Id3 GetInputIndex3D() const { return detail::InflateTo3D(this->GetIndexLogical()); }

  /// \brief The index into the output domain.
  ///
  /// This index refers to the output element (array value, cell, etc.) that
  /// this thread is creating. This is the typical index used during
  /// Fetch::Store.
  ///
  VTKM_EXEC
  vtkm::Id GetOutputIndex() const { return this->ThreadIndex; }

  /// \brief The visit index.
  ///
  /// When multiple output indices have the same input index, they are
  /// distinguished using the visit index.
  ///
  VTKM_EXEC
  vtkm::IdComponent GetVisitIndex() const { return 0; }

  /// \brief The indices of the incident elements.
  ///
  /// A topology map has "visited" and "incident" elements (e.g. points, cells,
  /// etc). For each worklet invocation, there is exactly one visited element,
  /// but there can be several incident elements. This method returns a
  /// Vec-like object containing the indices to the incident elements.
  ///
  VTKM_EXEC
  const IndicesIncidentType& GetIndicesIncident() const { return this->IndicesIncident; }

  /// \brief The input indices of the incident elements in pointer form.
  ///
  /// Returns the same object as GetIndicesIncident except that it returns a
  /// pointer to the internally held object rather than a reference or copy.
  /// Since the from indices can be a sizeable Vec (8 entries is common), it is
  /// best not to have a bunch a copies. Thus, you can pass around a pointer
  /// instead. However, care should be taken to make sure that this object does
  /// not go out of scope, at which time the returned pointer becomes invalid.
  ///
  VTKM_EXEC
  const IndicesIncidentType* GetIndicesIncidentPointer() const { return &this->IndicesIncident; }

  /// \brief The shape of the input cell.
  ///
  /// In topology maps that map from points to something, the indices make up
  /// the structure of a cell. Although the shape tag is not technically and
  /// index, it defines the meaning of the indices, so we put it here. (That
  /// and this class is the only convenient place to store it.)
  ///
  VTKM_EXEC
  CellShapeTag GetCellShape() const { return this->CellShape; }

private:
  vtkm::Id ThreadIndex;
  LogicalIndexType LogicalIndex;
  IndicesIncidentType IndicesIncident;
  CellShapeTag CellShape;
};


/// \brief Specialization for permuted structured connectivity types.
template <typename PermutationPortal, vtkm::IdComponent Dimension>
class ThreadIndicesTopologyMap<vtkm::exec::ConnectivityPermutedVisitCellsWithPoints<
                                 PermutationPortal,
                                 vtkm::exec::ConnectivityStructured<vtkm::TopologyElementTagCell,
                                                                    vtkm::TopologyElementTagPoint,
                                                                    Dimension>>,
                               CustomScatterOrMaskTag>
{
  using PermutedConnectivityType = vtkm::exec::ConnectivityPermutedVisitCellsWithPoints<
    PermutationPortal,
    vtkm::exec::ConnectivityStructured<vtkm::TopologyElementTagCell,
                                       vtkm::TopologyElementTagPoint,
                                       Dimension>>;
  using ConnectivityType = vtkm::exec::
    ConnectivityStructured<vtkm::TopologyElementTagCell, vtkm::TopologyElementTagPoint, Dimension>;

public:
  using IndicesIncidentType = typename ConnectivityType::IndicesType;
  using CellShapeTag = typename ConnectivityType::CellShapeTag;
  using LogicalIndexType = typename ConnectivityType::SchedulingRangeType;
  using Connectivity = vtkm::exec::ConnectivityPermutedVisitCellsWithPoints<
    PermutationPortal,
    vtkm::exec::ConnectivityStructured<vtkm::TopologyElementTagCell,
                                       vtkm::TopologyElementTagPoint,
                                       Dimension>>;

  VTKM_EXEC ThreadIndicesTopologyMap(vtkm::Id threadIndex,
                                     vtkm::Id inputIndex,
                                     vtkm::IdComponent visitIndex,
                                     vtkm::Id outputIndex,
                                     const PermutedConnectivityType& permutation)
  {
    this->ThreadIndex = threadIndex;
    this->InputIndex = inputIndex;
    this->VisitIndex = visitIndex;
    this->OutputIndex = outputIndex;

    const vtkm::Id permutedIndex = permutation.Portal.Get(this->InputIndex);
    this->LogicalIndex = permutation.Connectivity.FlatToLogicalToIndex(permutedIndex);
    this->IndicesIncident = permutation.Connectivity.GetIndices(this->LogicalIndex);
    this->CellShape = permutation.Connectivity.GetCellShape(permutedIndex);
  }

  /// \brief The index of the thread or work invocation.
  ///
  /// This index refers to which instance of the worklet is being invoked. Every invocation of the
  /// worklet has a unique thread index. This is also called the work index depending on the
  /// context.
  ///
  VTKM_EXEC
  vtkm::Id GetThreadIndex() const { return this->ThreadIndex; }

  /// \brief The logical index into the input domain.
  ///
  /// This is similar to \c GetIndex3D except the Vec size matches the actual
  /// dimensions of the data.
  ///
  VTKM_EXEC
  LogicalIndexType GetIndexLogical() const { return this->LogicalIndex; }

  /// \brief The index into the input domain.
  ///
  /// This index refers to the input element (array value, cell, etc.) that
  /// this thread is being invoked for. This is the typical index used during
  /// fetches.
  ///
  VTKM_EXEC
  vtkm::Id GetInputIndex() const { return this->InputIndex; }

  /// \brief The 3D index into the input domain.
  ///
  /// Overloads the implementation in the base class to return the 3D index
  /// for the input.
  ///
  VTKM_EXEC
  vtkm::Id3 GetInputIndex3D() const { return detail::InflateTo3D(this->GetIndexLogical()); }

  /// \brief The index into the output domain.
  ///
  /// This index refers to the output element (array value, cell, etc.) that
  /// this thread is creating. This is the typical index used during
  /// Fetch::Store.
  ///
  VTKM_EXEC
  vtkm::Id GetOutputIndex() const { return this->OutputIndex; }

  /// \brief The visit index.
  ///
  /// When multiple output indices have the same input index, they are
  /// distinguished using the visit index.
  ///
  VTKM_EXEC
  vtkm::IdComponent GetVisitIndex() const { return this->VisitIndex; }

  /// \brief The indices of the incident elements.
  ///
  /// A topology map has "visited" and "incident" elements (e.g. points, cells,
  /// etc). For each worklet invocation, there is exactly one visited element,
  /// but there can be several incident elements. This method returns a
  /// Vec-like object containing the indices to the incident elements.
  ///
  VTKM_EXEC
  const IndicesIncidentType& GetIndicesIncident() const { return this->IndicesIncident; }

  /// \brief The input indices of the incident elements in pointer form.
  ///
  /// Returns the same object as GetIndicesIncident except that it returns a
  /// pointer to the internally held object rather than a reference or copy.
  /// Since the from indices can be a sizeable Vec (8 entries is common), it is
  /// best not to have a bunch a copies. Thus, you can pass around a pointer
  /// instead. However, care should be taken to make sure that this object does
  /// not go out of scope, at which time the returned pointer becomes invalid.
  ///
  VTKM_EXEC
  const IndicesIncidentType* GetIndicesIncidentPointer() const { return &this->IndicesIncident; }

  /// \brief The shape of the input cell.
  ///
  /// In topology maps that map from points to something, the indices make up
  /// the structure of a cell. Although the shape tag is not technically and
  /// index, it defines the meaning of the indices, so we put it here. (That
  /// and this class is the only convenient place to store it.)
  ///
  VTKM_EXEC
  CellShapeTag GetCellShape() const { return this->CellShape; }

private:
  vtkm::Id ThreadIndex;
  vtkm::Id InputIndex;
  vtkm::IdComponent VisitIndex;
  vtkm::Id OutputIndex;
  LogicalIndexType LogicalIndex;
  IndicesIncidentType IndicesIncident;
  CellShapeTag CellShape;
};
}
}
} // namespace vtkm::exec::arg

#endif //vtk_m_exec_arg_ThreadIndicesTopologyMap_h
