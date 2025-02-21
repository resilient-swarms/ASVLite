//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_cont_ArrayHandleZip_h
#define vtk_m_cont_ArrayHandleZip_h

#include <vtkm/Pair.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/internal/ArrayPortalHelpers.h>

namespace vtkm
{
namespace exec
{
namespace internal
{

/// \brief An array portal that zips two portals together into a single value
/// for the execution environment
template <typename PortalTypeFirst, typename PortalTypeSecond>
class ArrayPortalZip
{
  using ReadableP1 = vtkm::internal::PortalSupportsGets<PortalTypeFirst>;
  using ReadableP2 = vtkm::internal::PortalSupportsGets<PortalTypeSecond>;
  using WritableP1 = vtkm::internal::PortalSupportsSets<PortalTypeFirst>;
  using WritableP2 = vtkm::internal::PortalSupportsSets<PortalTypeSecond>;

  using Readable = std::integral_constant<bool, ReadableP1::value && ReadableP2::value>;
  using Writable = std::integral_constant<bool, WritableP1::value && WritableP2::value>;

public:
  using T = typename PortalTypeFirst::ValueType;
  using U = typename PortalTypeSecond::ValueType;
  using ValueType = vtkm::Pair<T, U>;

  VTKM_SUPPRESS_EXEC_WARNINGS
  VTKM_EXEC_CONT
  ArrayPortalZip()
    : PortalFirst()
    , PortalSecond()
  {
  } //needs to be host and device so that cuda can create lvalue of these

  VTKM_CONT
  ArrayPortalZip(const PortalTypeFirst& portalfirst, const PortalTypeSecond& portalsecond)
    : PortalFirst(portalfirst)
    , PortalSecond(portalsecond)
  {
  }

  /// Copy constructor for any other ArrayPortalZip with an iterator
  /// type that can be copied to this iterator type. This allows us to do any
  /// type casting that the iterators do (like the non-const to const cast).
  ///
  template <class OtherF, class OtherS>
  VTKM_CONT ArrayPortalZip(const ArrayPortalZip<OtherF, OtherS>& src)
    : PortalFirst(src.GetFirstPortal())
    , PortalSecond(src.GetSecondPortal())
  {
  }

  VTKM_SUPPRESS_EXEC_WARNINGS
  VTKM_EXEC_CONT
  vtkm::Id GetNumberOfValues() const { return this->PortalFirst.GetNumberOfValues(); }

  VTKM_SUPPRESS_EXEC_WARNINGS
  template <typename Readable_ = Readable,
            typename = typename std::enable_if<Readable_::value>::type>
  VTKM_EXEC_CONT ValueType Get(vtkm::Id index) const noexcept
  {
    return vtkm::make_Pair(this->PortalFirst.Get(index), this->PortalSecond.Get(index));
  }

  VTKM_SUPPRESS_EXEC_WARNINGS
  template <typename Writable_ = Writable,
            typename = typename std::enable_if<Writable_::value>::type>
  VTKM_EXEC_CONT void Set(vtkm::Id index, const ValueType& value) const noexcept
  {
    this->PortalFirst.Set(index, value.first);
    this->PortalSecond.Set(index, value.second);
  }

  VTKM_EXEC_CONT
  const PortalTypeFirst& GetFirstPortal() const { return this->PortalFirst; }

  VTKM_EXEC_CONT
  const PortalTypeSecond& GetSecondPortal() const { return this->PortalSecond; }

private:
  PortalTypeFirst PortalFirst;
  PortalTypeSecond PortalSecond;
};
}
}
} // namespace vtkm::exec::internal

namespace vtkm
{
namespace cont
{

template <typename ST1, typename ST2>
struct VTKM_ALWAYS_EXPORT StorageTagZip
{
};

namespace internal
{

/// This helper struct defines the value type for a zip container containing
/// the given two array handles.
///
template <typename FirstHandleType, typename SecondHandleType>
struct ArrayHandleZipTraits
{
  /// The ValueType (a pair containing the value types of the two arrays).
  ///
  using ValueType =
    vtkm::Pair<typename FirstHandleType::ValueType, typename SecondHandleType::ValueType>;

  /// The appropriately templated tag.
  ///
  using Tag =
    StorageTagZip<typename FirstHandleType::StorageTag, typename SecondHandleType::StorageTag>;

  /// The superclass for ArrayHandleZip.
  ///
  using Superclass = vtkm::cont::ArrayHandle<ValueType, Tag>;
};

template <typename T1, typename T2, typename ST1, typename ST2>
class Storage<vtkm::Pair<T1, T2>, vtkm::cont::StorageTagZip<ST1, ST2>>
{
  using FirstStorage = Storage<T1, ST1>;
  using SecondStorage = Storage<T2, ST2>;
  using ValueType = vtkm::Pair<T1, T2>;

  template <typename BufferType>
  VTKM_CONT static BufferType* FirstArrayBuffers(BufferType* buffers)
  {
    return buffers;
  }
  template <typename BufferType>
  VTKM_CONT static BufferType* SecondArrayBuffers(BufferType* buffers)
  {
    return buffers + FirstStorage::GetNumberOfBuffers();
  }

public:
  using ReadPortalType =
    vtkm::exec::internal::ArrayPortalZip<typename FirstStorage::ReadPortalType,
                                         typename SecondStorage::ReadPortalType>;
  using WritePortalType =
    vtkm::exec::internal::ArrayPortalZip<typename FirstStorage::WritePortalType,
                                         typename SecondStorage::WritePortalType>;

  VTKM_CONT static constexpr vtkm::IdComponent GetNumberOfBuffers()
  {
    return FirstStorage::GetNumberOfBuffers() + SecondStorage::GetNumberOfBuffers();
  }

  VTKM_CONT static void ResizeBuffers(vtkm::Id numValues,
                                      vtkm::cont::internal::Buffer* buffers,
                                      vtkm::CopyFlag preserve,
                                      vtkm::cont::Token& token)
  {
    FirstStorage::ResizeBuffers(numValues, FirstArrayBuffers(buffers), preserve, token);
    SecondStorage::ResizeBuffers(numValues, SecondArrayBuffers(buffers), preserve, token);
  }

  VTKM_CONT static vtkm::Id GetNumberOfValues(const vtkm::cont::internal::Buffer* buffers)
  {
    vtkm::Id numValues = FirstStorage::GetNumberOfValues(FirstArrayBuffers(buffers));
    VTKM_ASSERT(numValues == SecondStorage::GetNumberOfValues(SecondArrayBuffers(buffers)));
    return numValues;
  }

  VTKM_CONT static ReadPortalType CreateReadPortal(const vtkm::cont::internal::Buffer* buffers,
                                                   vtkm::cont::DeviceAdapterId device,
                                                   vtkm::cont::Token& token)
  {
    return ReadPortalType(
      FirstStorage::CreateReadPortal(FirstArrayBuffers(buffers), device, token),
      SecondStorage::CreateReadPortal(SecondArrayBuffers(buffers), device, token));
  }

  VTKM_CONT static WritePortalType CreateWritePortal(vtkm::cont::internal::Buffer* buffers,
                                                     vtkm::cont::DeviceAdapterId device,
                                                     vtkm::cont::Token& token)
  {
    return WritePortalType(
      FirstStorage::CreateWritePortal(FirstArrayBuffers(buffers), device, token),
      SecondStorage::CreateWritePortal(SecondArrayBuffers(buffers), device, token));
  }
};
} // namespace internal

/// ArrayHandleZip is a specialization of ArrayHandle. It takes two delegate
/// array handle and makes a new handle that access the corresponding entries
/// in these arrays as a pair.
///
template <typename FirstHandleType, typename SecondHandleType>
class ArrayHandleZip
  : public internal::ArrayHandleZipTraits<FirstHandleType, SecondHandleType>::Superclass
{
  // If the following line gives a compile error, then the FirstHandleType
  // template argument is not a valid ArrayHandle type.
  VTKM_IS_ARRAY_HANDLE(FirstHandleType);

  // If the following line gives a compile error, then the SecondHandleType
  // template argument is not a valid ArrayHandle type.
  VTKM_IS_ARRAY_HANDLE(SecondHandleType);

public:
  VTKM_ARRAY_HANDLE_SUBCLASS(
    ArrayHandleZip,
    (ArrayHandleZip<FirstHandleType, SecondHandleType>),
    (typename internal::ArrayHandleZipTraits<FirstHandleType, SecondHandleType>::Superclass));

  VTKM_CONT
  ArrayHandleZip(const FirstHandleType& firstArray, const SecondHandleType& secondArray)
    : Superclass(vtkm::cont::internal::CreateBuffers(firstArray, secondArray))
  {
  }
};

/// A convenience function for creating an ArrayHandleZip. It takes the two
/// arrays to be zipped together.
///
template <typename FirstHandleType, typename SecondHandleType>
VTKM_CONT vtkm::cont::ArrayHandleZip<FirstHandleType, SecondHandleType> make_ArrayHandleZip(
  const FirstHandleType& first,
  const SecondHandleType& second)
{
  return ArrayHandleZip<FirstHandleType, SecondHandleType>(first, second);
}
}
} // namespace vtkm::cont

//=============================================================================
// Specializations of serialization related classes
/// @cond SERIALIZATION
namespace vtkm
{
namespace cont
{

template <typename AH1, typename AH2>
struct SerializableTypeString<vtkm::cont::ArrayHandleZip<AH1, AH2>>
{
  static VTKM_CONT const std::string& Get()
  {
    static std::string name = "AH_Zip<" + SerializableTypeString<AH1>::Get() + "," +
      SerializableTypeString<AH2>::Get() + ">";
    return name;
  }
};

template <typename T1, typename T2, typename ST1, typename ST2>
struct SerializableTypeString<
  vtkm::cont::ArrayHandle<vtkm::Pair<T1, T2>, vtkm::cont::StorageTagZip<ST1, ST2>>>
  : SerializableTypeString<vtkm::cont::ArrayHandleZip<vtkm::cont::ArrayHandle<T1, ST1>,
                                                      vtkm::cont::ArrayHandle<T2, ST2>>>
{
};
}
} // namespace vtkm::cont

namespace mangled_diy_namespace
{

template <typename AH1, typename AH2>
struct Serialization<vtkm::cont::ArrayHandleZip<AH1, AH2>>
{
private:
  using Type = typename vtkm::cont::ArrayHandleZip<AH1, AH2>;
  using BaseType = vtkm::cont::ArrayHandle<typename Type::ValueType, typename Type::StorageTag>;

public:
  static VTKM_CONT void save(BinaryBuffer& bb, const BaseType& obj)
  {
    auto storage = obj.GetStorage();
    vtkmdiy::save(bb, storage.GetFirstArray());
    vtkmdiy::save(bb, storage.GetSecondArray());
  }

  static VTKM_CONT void load(BinaryBuffer& bb, BaseType& obj)
  {
    AH1 a1;
    AH2 a2;

    vtkmdiy::load(bb, a1);
    vtkmdiy::load(bb, a2);

    obj = vtkm::cont::make_ArrayHandleZip(a1, a2);
  }
};

template <typename T1, typename T2, typename ST1, typename ST2>
struct Serialization<
  vtkm::cont::ArrayHandle<vtkm::Pair<T1, T2>, vtkm::cont::StorageTagZip<ST1, ST2>>>
  : Serialization<vtkm::cont::ArrayHandleZip<vtkm::cont::ArrayHandle<T1, ST1>,
                                             vtkm::cont::ArrayHandle<T2, ST2>>>
{
};

} // diy
/// @endcond SERIALIZATION

#endif //vtk_m_cont_ArrayHandleZip_h
