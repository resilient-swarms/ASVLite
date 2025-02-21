//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_cont_ArrayHandleView_h
#define vtk_m_cont_ArrayHandleView_h

#include <vtkm/Assert.h>
#include <vtkm/Deprecated.h>

#include <vtkm/cont/ArrayExtractComponent.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/ArrayPortal.h>

namespace vtkm
{

namespace internal
{

struct ViewIndices
{
  vtkm::Id StartIndex = 0;
  vtkm::Id NumberOfValues = 0;

  ViewIndices() = default;

  ViewIndices(vtkm::Id start, vtkm::Id numValues)
    : StartIndex(start)
    , NumberOfValues(numValues)
  {
  }
};

template <typename TargetPortalType>
class ArrayPortalView
{
  using Writable = vtkm::internal::PortalSupportsSets<TargetPortalType>;

public:
  using ValueType = typename TargetPortalType::ValueType;

  VTKM_EXEC_CONT
  ArrayPortalView() {}

  VTKM_EXEC_CONT
  ArrayPortalView(const TargetPortalType& targetPortal, ViewIndices indices)
    : TargetPortal(targetPortal)
    , Indices(indices)
  {
  }

  template <typename OtherPortalType>
  VTKM_EXEC_CONT ArrayPortalView(const ArrayPortalView<OtherPortalType>& otherPortal)
    : TargetPortal(otherPortal.GetTargetPortal())
    , Indices(otherPortal.GetStartIndex(), otherPortal.GetNumberOfValues())
  {
  }

  VTKM_EXEC_CONT
  vtkm::Id GetNumberOfValues() const { return this->Indices.NumberOfValues; }

  VTKM_EXEC_CONT
  ValueType Get(vtkm::Id index) const
  {
    return this->TargetPortal.Get(index + this->GetStartIndex());
  }

  template <typename Writable_ = Writable,
            typename = typename std::enable_if<Writable_::value>::type>
  VTKM_EXEC_CONT void Set(vtkm::Id index, const ValueType& value) const
  {
    this->TargetPortal.Set(index + this->GetStartIndex(), value);
  }

  VTKM_EXEC_CONT
  const TargetPortalType& GetTargetPortal() const { return this->TargetPortal; }
  VTKM_EXEC_CONT
  vtkm::Id GetStartIndex() const { return this->Indices.StartIndex; }

private:
  TargetPortalType TargetPortal;
  ViewIndices Indices;
};

} // namespace internal

namespace cont
{

template <typename StorageTag>
struct VTKM_ALWAYS_EXPORT StorageTagView
{
};

namespace internal
{

namespace detail
{

template <typename T, typename ArrayOrStorage, bool IsArrayType>
struct ViewTypeArgImpl;

template <typename T, typename Storage>
struct ViewTypeArgImpl<T, Storage, false>
{
  using StorageTag = Storage;
  using ArrayHandle = vtkm::cont::ArrayHandle<T, StorageTag>;
};

template <typename T, typename Array>
struct ViewTypeArgImpl<T, Array, true>
{
  VTKM_STATIC_ASSERT_MSG((std::is_same<T, typename Array::ValueType>::value),
                         "Used array with wrong type in ArrayHandleView.");
  using StorageTag VTKM_DEPRECATED(1.6,
                                   "Use storage tag instead of array handle in StorageTagView.") =
    typename Array::StorageTag;
  using ArrayHandle VTKM_DEPRECATED(1.6,
                                    "Use storage tag instead of array handle in StorageTagView.") =
    vtkm::cont::ArrayHandle<T, typename Array::StorageTag>;
};

template <typename T, typename ArrayOrStorage>
struct ViewTypeArg
  : ViewTypeArgImpl<T,
                    ArrayOrStorage,
                    vtkm::cont::internal::ArrayHandleCheck<ArrayOrStorage>::type::value>
{
};

} // namespace detail

template <typename T, typename ST>
class Storage<T, StorageTagView<ST>>
{
  using ArrayHandleType = typename detail::ViewTypeArg<T, ST>::ArrayHandle;
  using SourceStorage = Storage<T, ST>;

public:
  VTKM_STORAGE_NO_RESIZE;

  using ReadPortalType = vtkm::internal::ArrayPortalView<typename ArrayHandleType::ReadPortalType>;
  using WritePortalType =
    vtkm::internal::ArrayPortalView<typename ArrayHandleType::WritePortalType>;

  VTKM_CONT static constexpr vtkm::IdComponent GetNumberOfBuffers()
  {
    return SourceStorage::GetNumberOfBuffers() + 1;
  }

  VTKM_CONT static vtkm::Id GetNumberOfValues(const vtkm::cont::internal::Buffer* buffers)
  {
    return buffers[0].GetMetaData<vtkm::internal::ViewIndices>().NumberOfValues;
  }

  VTKM_CONT static ReadPortalType CreateReadPortal(const vtkm::cont::internal::Buffer* buffers,
                                                   vtkm::cont::DeviceAdapterId device,
                                                   vtkm::cont::Token& token)
  {
    vtkm::internal::ViewIndices indices = buffers[0].GetMetaData<vtkm::internal::ViewIndices>();
    return ReadPortalType(SourceStorage::CreateReadPortal(buffers + 1, device, token), indices);
  }

  VTKM_CONT static WritePortalType CreateWritePortal(vtkm::cont::internal::Buffer* buffers,
                                                     vtkm::cont::DeviceAdapterId device,
                                                     vtkm::cont::Token& token)
  {
    vtkm::internal::ViewIndices indices = buffers[0].GetMetaData<vtkm::internal::ViewIndices>();
    return WritePortalType(SourceStorage::CreateWritePortal(buffers + 1, device, token), indices);
  }

  VTKM_CONT static std::vector<vtkm::cont::internal::Buffer>
  CreateBuffers(vtkm::Id startIndex, vtkm::Id numValues, const ArrayHandleType& array)
  {
    return vtkm::cont::internal::CreateBuffers(vtkm::internal::ViewIndices(startIndex, numValues),
                                               array);
  }

  VTKM_CONT static ArrayHandleType GetSourceArray(const vtkm::cont::internal::Buffer* buffers)
  {
    return ArrayHandleType(buffers + 1);
  }

  VTKM_CONT static vtkm::Id GetStartIndex(const vtkm::cont::internal::Buffer* buffers)
  {
    return buffers[0].GetMetaData<vtkm::internal::ViewIndices>().StartIndex;
  }
};

} // namespace internal

template <typename ArrayHandleType>
class ArrayHandleView
  : public vtkm::cont::ArrayHandle<typename ArrayHandleType::ValueType,
                                   StorageTagView<typename ArrayHandleType::StorageTag>>
{
  VTKM_IS_ARRAY_HANDLE(ArrayHandleType);

public:
  VTKM_ARRAY_HANDLE_SUBCLASS(
    ArrayHandleView,
    (ArrayHandleView<ArrayHandleType>),
    (vtkm::cont::ArrayHandle<typename ArrayHandleType::ValueType,
                             StorageTagView<typename ArrayHandleType::StorageTag>>));

private:
  using StorageType = vtkm::cont::internal::Storage<ValueType, StorageTag>;

public:
  VTKM_CONT
  ArrayHandleView(const ArrayHandleType& array, vtkm::Id startIndex, vtkm::Id numValues)
    : Superclass(StorageType::CreateBuffers(startIndex, numValues, array))
  {
  }

  VTKM_CONT ArrayHandleType GetSourceArray() const
  {
    return this->GetStorage().GetSourceArray(this->GetBuffers());
  }

  VTKM_CONT vtkm::Id GetStartIndex() const
  {
    return this->GetStorage().GetStartIndex(this->GetBuffers());
  }
};

template <typename ArrayHandleType>
ArrayHandleView<ArrayHandleType> make_ArrayHandleView(const ArrayHandleType& array,
                                                      vtkm::Id startIndex,
                                                      vtkm::Id numValues)
{
  VTKM_IS_ARRAY_HANDLE(ArrayHandleType);

  return ArrayHandleView<ArrayHandleType>(array, startIndex, numValues);
}

namespace internal
{

template <typename StorageTag>
struct ArrayExtractComponentImpl<StorageTagView<StorageTag>>
{
  template <typename T>
  using StrideArrayType =
    vtkm::cont::ArrayHandleStride<typename vtkm::VecTraits<T>::BaseComponentType>;

  template <typename T>
  StrideArrayType<T> operator()(
    const vtkm::cont::ArrayHandle<T, vtkm::cont::StorageTagView<StorageTag>>& src,
    vtkm::IdComponent componentIndex,
    vtkm::CopyFlag allowCopy) const
  {
    vtkm::cont::ArrayHandleView<vtkm::cont::ArrayHandle<T, StorageTag>> srcArray(src);
    StrideArrayType<T> subArray =
      ArrayExtractComponentImpl<StorageTag>{}(srcArray.GetSourceArray(), componentIndex, allowCopy);
    // Narrow the array by adjusting the size and offset.
    return StrideArrayType<T>(subArray.GetBasicArray(),
                              srcArray.GetNumberOfValues(),
                              subArray.GetStride(),
                              subArray.GetOffset() +
                                (subArray.GetStride() * srcArray.GetStartIndex()),
                              subArray.GetModulo(),
                              subArray.GetDivisor());
  }
};

} // namespace internal

}
} // namespace vtkm::cont

#endif //vtk_m_cont_ArrayHandleView_h
