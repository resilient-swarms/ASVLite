/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPResampleToImage.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// Hide VTK_DEPRECATED_IN_9_1_0() warning for this class
#define VTK_DEPRECATION_LEVEL 0

#include "vtkPResampleToImage.h"

#include "vtkArrayDispatch.h"
#include "vtkBoundingBox.h"
#include "vtkCharArray.h"
#include "vtkCompositeDataProbeFilter.h"
#include "vtkCompositeDataSet.h"
#include "vtkDIYUtilities.h"
#include "vtkDataArrayRange.h"
#include "vtkDataSetAttributes.h"
#include "vtkExtentTranslator.h"
#include "vtkIdList.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMultiProcessController.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkUnsignedCharArray.h"

// clang-format off
#include "vtk_diy2.h" // must include this before any diy header
#include VTK_DIY2(diy/assigner.hpp)
#include VTK_DIY2(diy/link.hpp)
#include VTK_DIY2(diy/master.hpp)
#include VTK_DIY2(diy/mpi.hpp)
#include VTK_DIY2(diy/reduce.hpp)
#include VTK_DIY2(diy/partners/swap.hpp)
#include VTK_DIY2(diy/decomposition.hpp)
// clang-format on

#include <algorithm>
#include <iterator>

vtkStandardNewMacro(vtkPResampleToImage);

vtkCxxSetObjectMacro(vtkPResampleToImage, Controller, vtkMultiProcessController);

namespace
{

//------------------------------------------------------------------------------
template <typename T, std::size_t Len>
struct Array
{
public:
  T& operator[](std::size_t idx) { return this->Data[idx]; }

  const T& operator[](std::size_t idx) const { return this->Data[idx]; }

  T* data() { return this->Data; }

  const T* data() const { return this->Data; }

  std::size_t size() const { return Len; }

private:
  T Data[Len];
};

//------------------------------------------------------------------------------
struct FieldMetaData
{
  std::string Name;
  int DataType;
  int NumComponents;
  int AttributeType;
};

inline void ExtractFieldMetaData(vtkDataSetAttributes* data, std::vector<FieldMetaData>* metadata)
{
  if (data == nullptr || data->GetNumberOfTuples() == 0)
  {
    // do not consider arrays from empty vtkDataSetAttributes.
    // see paraview/paraview#18590
    metadata->clear();
    return;
  }

  if (auto maskArray = data->GetArray("vtkValidPointMask"))
  {
    // data may not be valid if the only array is vtkValidPointMask and it's set
    // to 0.
    if (data->GetNumberOfArrays() == 1 && maskArray->GetRange(0)[0] < 1.0 &&
      maskArray->GetRange(0)[1] < 1.0)
    {
      metadata->clear();
      return;
    }
  }

  std::size_t numFields = static_cast<std::size_t>(data->GetNumberOfArrays());
  metadata->resize(numFields);

  for (std::size_t i = 0; i < numFields; ++i)
  {
    FieldMetaData& md = (*metadata)[i];
    vtkDataArray* da = data->GetArray(static_cast<int>(i));

    md.Name = da->GetName();
    md.DataType = da->GetDataType();
    md.NumComponents = da->GetNumberOfComponents();
    md.AttributeType = data->IsArrayAnAttribute(static_cast<int>(i));
  }
}

inline void InitializeFieldData(
  const std::vector<FieldMetaData>& metadata, vtkIdType numTuples, vtkDataSetAttributes* data)
{
  std::size_t numFields = metadata.size();
  for (std::size_t i = 0; i < numFields; ++i)
  {
    const FieldMetaData& md = metadata[i];
    vtkDataArray* da = vtkDataArray::CreateDataArray(md.DataType);
    da->SetName(md.Name.c_str());
    da->SetNumberOfComponents(md.NumComponents);
    da->SetNumberOfTuples(numTuples);

    double null_value = 0.0;
    for (int j = 0; j < da->GetNumberOfComponents(); ++j)
    {
      da->FillComponent(j, null_value);
    }
    data->AddArray(da);
    da->Delete();

    if (md.AttributeType >= 0)
    {
      data->SetActiveAttribute(static_cast<int>(i), md.AttributeType);
    }
  }
}

//------------------------------------------------------------------------------
struct SerializeWorklet
{
  template <typename ArrayType>
  void operator()(ArrayType* array, vtkIdType tupleIdx, diy::MemoryBuffer& buffer) const
  {
    using T = vtk::GetAPIType<ArrayType>; // Need value type for diy::save template resolution
    const auto tuple = vtk::DataArrayTupleRange(array)[tupleIdx];
    for (const T comp : tuple)
    {
      diy::save(buffer, comp);
    }
  }
};

inline void SerializeFieldData(vtkFieldData* field, vtkIdType tuple, diy::MemoryBuffer& bb)
{
  int numFields = field->GetNumberOfArrays();
  for (int i = 0; i < numFields; ++i)
  {
    vtkDataArray* da = field->GetArray(i);
    if (!vtkArrayDispatch::Dispatch::Execute(da, SerializeWorklet{}, tuple, bb))
    {
      vtkGenericWarningMacro(<< "Dispatch failed, fallback to vtkDataArray Get/Set");
      SerializeWorklet{}(da, tuple, bb);
    }
  }
}

struct DeserializeWorklet
{
  template <typename ArrayType>
  void operator()(ArrayType* array, vtkIdType tupleIdx, diy::MemoryBuffer& buffer) const
  {
    auto tuple = vtk::DataArrayTupleRange(array)[tupleIdx];
    using CompRefT = typename std::iterator_traits<decltype(tuple.begin())>::reference;
    // Need value type for diy::load template resolution
    using ValueT = typename std::iterator_traits<decltype(tuple.begin())>::value_type;
    for (CompRefT compRef : tuple)
    {
      ValueT val;
      diy::load(buffer, val);
      compRef = val;
    }
  }
};

inline void DeserializeFieldData(diy::MemoryBuffer& bb, vtkFieldData* field, vtkIdType tuple)
{
  int numFields = field->GetNumberOfArrays();
  for (int i = 0; i < numFields; ++i)
  {
    vtkDataArray* da = field->GetArray(i);
    if (!vtkArrayDispatch::Dispatch::Execute(da, DeserializeWorklet{}, tuple, bb))
    {
      vtkGenericWarningMacro(<< "Dispatch failed, fallback to vtkDataArray Get/Set");
      DeserializeWorklet{}(da, tuple, bb);
    }
  }
}

//------------------------------------------------------------------------------
// A structure representing a list of points from an ImageData. Stores the
// points' 3D indices (Indices) and serialized point data (Data) and they
// should be stored in the same order.
struct PointList
{
  typedef Array<int, 3> IndexType;

  std::vector<IndexType> Indices; // indices
  std::vector<char> Data;         // serialized data
  vtkIdType DataSize;             // size in bytes of serialized data of one point
};

inline void swap(PointList& a, PointList& b)
{
  a.Indices.swap(b.Indices);
  a.Data.swap(b.Data);
  std::swap(a.DataSize, b.DataSize);
}

inline vtkIdType ComputeSerializedFieldDataSize(const std::vector<FieldMetaData>& fieldMetaData)
{
  vtkNew<vtkDataSetAttributes> attribs;
  InitializeFieldData(fieldMetaData, 1, attribs);
  diy::MemoryBuffer bb;
  SerializeFieldData(attribs, 0, bb);
  return static_cast<vtkIdType>(bb.buffer.size());
}

//------------------------------------------------------------------------------
struct Block
{
  PointList Points;
  int Extent[6];
};

inline void* CreateBlock()
{
  return new Block;
}

inline void DestroyBlock(void* blockp)
{
  delete static_cast<Block*>(blockp);
}

//------------------------------------------------------------------------------
// Creates a PointList of all the valid points in img
inline void GetPointsFromImage(vtkImageData* img, const char* maskArrayName, PointList* points)
{
  if (img->GetNumberOfPoints() <= 0)
  {
    return;
  }

  vtkPointData* pd = img->GetPointData();
  vtkCharArray* maskArray = vtkArrayDownCast<vtkCharArray>(pd->GetArray(maskArrayName));
  char* mask = maskArray->GetPointer(0);

  // use diy's serialization facilities
  diy::MemoryBuffer bb;

  int extent[6];
  img->GetExtent(extent);
  for (int k = extent[4]; k <= extent[5]; ++k)
  {
    for (int j = extent[2]; j <= extent[3]; ++j)
    {
      for (int i = extent[0]; i <= extent[1]; ++i)
      {
        int ijk[3] = { i, j, k };
        vtkIdType id = img->ComputePointId(ijk);
        if (mask[id])
        {
          PointList::IndexType idx;
          std::copy(ijk, ijk + 3, idx.data());
          points->Indices.push_back(idx);
          SerializeFieldData(pd, id, bb);
        }
      }
    }
  }
  points->Data.swap(bb.buffer); // get the serialized data buffer
}

// Sets the points from the PointList (points) to img. 'points' is modified
// in the process.
void SetPointsToImage(
  const std::vector<FieldMetaData>& fieldMetaData, PointList& points, vtkImageData* img)
{
  vtkPointData* pd = img->GetPointData();
  InitializeFieldData(fieldMetaData, img->GetNumberOfPoints(), pd);

  diy::MemoryBuffer bb;
  bb.buffer.swap(points.Data);
  std::size_t numPoints = points.Indices.size();
  for (std::size_t i = 0; i < numPoints; ++i)
  {
    vtkIdType id = img->ComputePointId(points.Indices[i].data());
    DeserializeFieldData(bb, pd, id);
  }

  points.Indices.clear(); // reset the points structure to a valid empty state
}

inline void GetGlobalFieldMetaData(
  diy::mpi::communicator& comm, vtkDataSetAttributes* data, std::vector<FieldMetaData>* metadata)
{
  std::vector<FieldMetaData> local;
  ExtractFieldMetaData(data, &local);

  // find a process that has field meta data information (choose the process with
  // minimum rank)
  int rank = !local.empty() ? comm.rank() : comm.size();
  int source;
  diy::mpi::all_reduce(comm, rank, source, diy::mpi::minimum<int>());

  if (source < comm.size()) // at least one process has field meta data
  {
    diy::MemoryBuffer bb;
    if (comm.rank() == source)
    {
      diy::save(bb, local);
      bb.reset();
    }
    diy::mpi::broadcast(comm, bb.buffer, source);
    diy::load(bb, *metadata);
  }
}

//------------------------------------------------------------------------------
void Redistribute(
  void* blockp, const diy::ReduceProxy& srp, const diy::RegularSwapPartners& partners)
{
  Block* b = static_cast<Block*>(blockp);
  unsigned round = srp.round();

  // step 1: dequeue all the incoming points and add them to this block's vector
  diy::Master::IncomingQueues& in = *srp.incoming();
  for (diy::Master::IncomingQueues::iterator i = in.begin(); i != in.end(); ++i)
  {
    while (i->second)
    {
      PointList::IndexType idx;
      srp.dequeue(i->first, idx);
      b->Points.Indices.push_back(idx);

      std::size_t beg = b->Points.Data.size();
      b->Points.Data.resize(beg + b->Points.DataSize);
      srp.dequeue(i->first, &b->Points.Data[beg], b->Points.DataSize);
    }
  }

  // final round
  if (srp.out_link().size() == 0)
  {
    return;
  }

  // find this block's position in the group
  int groupSize = srp.out_link().size();
  int myPos = 0;
  for (; myPos < groupSize; ++myPos)
  {
    if (srp.out_link().target(myPos).gid == srp.gid())
    {
      break;
    }
  }

  // step 2: redistribute this block's points among the blocks in the group
  int axis = partners.dim(round);
  int minIdx = b->Extent[2 * axis];
  int maxIdx = b->Extent[2 * axis + 1];
  int length = (maxIdx - minIdx + 1 + groupSize - 1) / groupSize;

  PointList myPoints;
  myPoints.DataSize = b->Points.DataSize; // initialize myPoints
  std::size_t numPoints = b->Points.Indices.size();
  for (size_t i = 0; i < numPoints; ++i)
  {
    PointList::IndexType idx = b->Points.Indices[i];
    const char* data = &b->Points.Data[i * b->Points.DataSize];

    int nlocs = 1;
    int loc[2] = { (idx[axis] - minIdx) / length, 0 };

    // duplicate shared point
    if (((idx[axis] - minIdx) % length == 0) && (loc[0] != 0))
    {
      loc[1] = loc[0] - 1;
      ++nlocs;
    }

    for (int j = 0; j < nlocs; ++j)
    {
      if (loc[j] == myPos)
      {
        myPoints.Indices.push_back(idx);
        myPoints.Data.insert(myPoints.Data.end(), data, data + myPoints.DataSize);
      }
      else
      {
        srp.enqueue(srp.out_link().target(loc[j]), idx);
        srp.enqueue(srp.out_link().target(loc[j]), data, myPoints.DataSize);
      }
    }
  }
  swap(b->Points, myPoints);

  // step 3: readjust extents for next round
  b->Extent[2 * axis] = minIdx + (length * myPos);
  b->Extent[2 * axis + 1] = std::min(b->Extent[2 * axis] + length, maxIdx);
}

} // anonymous namespace

//------------------------------------------------------------------------------
vtkPResampleToImage::vtkPResampleToImage()
  : Controller(nullptr)
{
  this->SetController(vtkMultiProcessController::GetGlobalController());
}

//------------------------------------------------------------------------------
vtkPResampleToImage::~vtkPResampleToImage()
{
  this->SetController(nullptr);
}

//------------------------------------------------------------------------------
void vtkPResampleToImage::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  if (this->Controller)
  {
    this->Controller->PrintSelf(os, indent);
  }
}

//------------------------------------------------------------------------------
int vtkPResampleToImage::RequestData(
  vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  if (!this->Controller || this->Controller->GetNumberOfProcesses() == 1)
  {
    return this->Superclass::RequestData(request, inputVector, outputVector);
  }

  // get the info objects
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // get the input and output
  vtkDataObject* input = inInfo->Get(vtkDataObject::DATA_OBJECT());
  vtkImageData* output = vtkImageData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  diy::mpi::communicator comm = vtkDIYUtilities::GetCommunicator(this->Controller);

  double localBounds[6];
  ComputeDataBounds(input, localBounds);

  double samplingBounds[6];
  if (this->UseInputBounds)
  {
    vtkBoundingBox bbox(localBounds);
    vtkDIYUtilities::AllReduce(comm, bbox);

    // To avoid accidentally sampling outside the dataset due to floating point roundoff,
    // nudge the bounds inward by epsilon.
    // Note: this is same as what's done in the non-parallel version of this
    // filter i.e. vtkResampleToImage. So just doing the same here for
    // consistency.
    const double epsilon = 1.0e-6;
    bbox.ScaleAboutCenter(1.0 - epsilon);
    bbox.GetBounds(samplingBounds);
  }
  else
  {
    std::copy(this->SamplingBounds, this->SamplingBounds + 6, samplingBounds);
  }

  vtkNew<vtkImageData> mypiece;
  this->PerformResampling(input, samplingBounds, true, localBounds, mypiece);

  // Ensure every node has fields' metadata information
  std::vector<FieldMetaData> pointFieldMetaData;
  GetGlobalFieldMetaData(comm, mypiece->GetPointData(), &pointFieldMetaData);

  // perform swap-reduce partitioning on probed points to decompose the domain
  // into non-overlapping rectangular regions
  diy::RoundRobinAssigner assigner(comm.size(), comm.size());

  int* updateExtent = this->GetUpdateExtent();
  diy::DiscreteBounds domain(3);
  for (int i = 0; i < 3; ++i)
  {
    domain.min[i] = updateExtent[2 * i];
    domain.max[i] = updateExtent[2 * i + 1];
  }

  diy::Master master(comm, 1, -1, &CreateBlock, &DestroyBlock);

  diy::RegularDecomposer<diy::DiscreteBounds> decomposer(3, domain, comm.size());
  decomposer.decompose(comm.rank(), assigner, master);

  // Set up master's block
  Block* block = master.block<Block>(0);
  std::copy(updateExtent, updateExtent + 6, block->Extent);
  block->Points.DataSize = ComputeSerializedFieldDataSize(pointFieldMetaData);
  GetPointsFromImage(mypiece, this->GetMaskArrayName(), &block->Points);

  diy::RegularSwapPartners partners(decomposer, 2, false);
  diy::reduce(master, assigner, partners, &Redistribute);

  output->SetOrigin(mypiece->GetOrigin());
  output->SetSpacing(mypiece->GetSpacing());
  output->SetExtent(block->Extent);
  SetPointsToImage(pointFieldMetaData, block->Points, output);
  this->SetBlankPointsAndCells(output);

  return 1;
}

//------------------------------------------------------------------------------
namespace diy
{

namespace mpi
{
namespace detail
{

template <class T, std::size_t Len>
struct mpi_datatype<Array<T, Len>>
{
  typedef Array<T, Len> ArrayType;

  static MPI_Datatype datatype() { return get_mpi_datatype<T>(); }
  static const void* address(const ArrayType& x) { return &x[0]; }
  static void* address(ArrayType& x) { return &x[0]; }
  static int count(const ArrayType&) { return Len; }
};

}
} // namespace mpi::detail

template <>
struct Serialization<FieldMetaData>
{
  static void save(BinaryBuffer& bb, const FieldMetaData& f)
  {
    diy::save(bb, f.Name);
    diy::save(bb, f.DataType);
    diy::save(bb, f.NumComponents);
    diy::save(bb, f.AttributeType);
  }

  static void load(BinaryBuffer& bb, FieldMetaData& f)
  {
    diy::load(bb, f.Name);
    diy::load(bb, f.DataType);
    diy::load(bb, f.NumComponents);
    diy::load(bb, f.AttributeType);
  }
};

} // namespace diy
