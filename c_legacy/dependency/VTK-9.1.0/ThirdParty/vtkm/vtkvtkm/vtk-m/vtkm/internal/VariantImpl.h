//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#if !defined(VTK_M_DEVICE) || !defined(VTK_M_NAMESPACE)
#error VariantImpl.h must be included from Variant.h
// Some defines to make my IDE happy.
#define VTK_M_DEVICE
#define VTK_M_NAMESPACE tmp
#endif

#include <vtkm/internal/VariantImplDetail.h>

#include <vtkm/Deprecated.h>
#include <vtkm/List.h>

#include <vtkm/internal/Assume.h>

namespace vtkm
{
namespace VTK_M_NAMESPACE
{
namespace internal
{

// Forward declaration
template <typename... Ts>
class Variant;

namespace detail
{

// --------------------------------------------------------------------------------
// Helper classes for Variant

template <typename UnionType>
struct VariantUnionToListImpl;
template <typename... Ts>
struct VariantUnionToListImpl<detail::VariantUnionTD<Ts...>>
{
  using type = vtkm::List<Ts...>;
};
template <typename... Ts>
struct VariantUnionToListImpl<detail::VariantUnionNTD<Ts...>>
{
  using type = vtkm::List<Ts...>;
};

template <typename UnionType>
using VariantUnionToList =
  typename VariantUnionToListImpl<typename std::decay<UnionType>::type>::type;

struct VariantCopyConstructFunctor
{
  template <typename T, typename UnionType>
  VTK_M_DEVICE void operator()(const T& src, UnionType& destUnion) const noexcept
  {
    constexpr vtkm::IdComponent Index = vtkm::ListIndexOf<VariantUnionToList<UnionType>, T>::value;
    // If we are using this functor, we can assume the union does not hold a valid type.
    new (&VariantUnionGet<Index>(destUnion)) T(src);
  }
};

struct VariantCopyFunctor
{
  template <typename T, typename UnionType>
  VTK_M_DEVICE void operator()(const T& src, UnionType& destUnion) const noexcept
  {
    constexpr vtkm::IdComponent Index = vtkm::ListIndexOf<VariantUnionToList<UnionType>, T>::value;
    // If we are using this functor, we can assume the union holds type T.
    this->DoCopy(
      src, VariantUnionGet<Index>(destUnion), typename std::is_copy_assignable<T>::type{});
  }

  template <typename T>
  VTK_M_DEVICE void DoCopy(const T& src, T& dest, std::true_type) const noexcept
  {
    dest = src;
  }

  template <typename T>
  VTK_M_DEVICE void DoCopy(const T& src, T& dest, std::false_type) const noexcept
  {
    if (&src != &dest)
    {
      // Do not have an assignment operator, so destroy the old object and create a new one.
      dest.~T();
      new (&dest) T(src);
    }
    else
    {
      // Objects are already the same.
    }
  }
};

struct VariantDestroyFunctor
{
  template <typename T>
  VTK_M_DEVICE void operator()(T& src) const noexcept
  {
    src.~T();
  }
};

template <typename T>
struct VariantCheckType
{
  // We are currently not allowing reference types (e.g. FooType&) or pointer types (e.g. FooType*)
  // in Variant objects. References and pointers can fail badly when things are passed around
  // devices. If you get a compiler error here, consider removing the reference or using something
  // like std::decay to remove qualifiers on the type. (We may decide to do that automatically in
  // the future.)
  VTKM_STATIC_ASSERT_MSG(!std::is_reference<T>::value,
                         "References are not allowed in VTK-m Variant.");
  VTKM_STATIC_ASSERT_MSG(!std::is_pointer<T>::value, "Pointers are not allowed in VTK-m Variant.");
};

template <typename VariantType>
struct VariantTriviallyCopyable;

template <typename... Ts>
struct VariantTriviallyCopyable<vtkm::VTK_M_NAMESPACE::internal::Variant<Ts...>>
  : AllTriviallyCopyable<Ts...>
{
};

template <typename VariantType>
struct VariantTriviallyConstructible;

template <typename... Ts>
struct VariantTriviallyConstructible<vtkm::VTK_M_NAMESPACE::internal::Variant<Ts...>>
  : AllTriviallyConstructible<Ts...>
{
};

// --------------------------------------------------------------------------------
// Variant superclass that defines its storage
template <typename... Ts>
struct VariantStorageImpl
{
  VariantUnion<Ts...> Storage;
  vtkm::IdComponent Index;

  VariantStorageImpl() = default;

  VTK_M_DEVICE VariantStorageImpl(vtkm::internal::NullType dummy)
    : Storage({ dummy })
  {
  }

  template <vtkm::IdComponent Index>
  using TypeAt = typename vtkm::ListAt<vtkm::List<Ts...>, Index>;

  VTK_M_DEVICE vtkm::IdComponent GetIndex() const noexcept { return this->Index; }
  VTK_M_DEVICE bool IsValid() const noexcept
  {
    return (this->Index >= 0) && (this->Index < static_cast<vtkm::IdComponent>(sizeof...(Ts)));
  }

  VTK_M_DEVICE void Reset() noexcept
  {
    if (this->IsValid())
    {
      this->CastAndCall(detail::VariantDestroyFunctor{});
      this->Index = -1;
    }
  }

  template <typename Functor, typename... Args>
  VTK_M_DEVICE auto CastAndCall(Functor&& f, Args&&... args) const
    noexcept(noexcept(f(std::declval<const TypeAt<0>&>(), args...)))
      -> decltype(f(std::declval<const TypeAt<0>&>(), args...))
  {
    VTKM_ASSERT(this->IsValid());
    return detail::VariantCastAndCallImpl(
      this->GetIndex(), std::forward<Functor>(f), this->Storage, std::forward<Args>(args)...);
  }

  template <typename Functor, typename... Args>
  VTK_M_DEVICE auto CastAndCall(Functor&& f, Args&&... args) noexcept(
    noexcept(f(std::declval<const TypeAt<0>&>(), args...)))
    -> decltype(f(std::declval<TypeAt<0>&>(), args...))
  {
    VTKM_ASSERT(this->IsValid());
    return detail::VariantCastAndCallImpl(
      this->GetIndex(), std::forward<Functor>(f), this->Storage, std::forward<Args>(args)...);
  }
};

// --------------------------------------------------------------------------------
// Variant superclass that helps preserve trivially copyable and trivially constructable
// properties where possible.
template <typename VariantType,
          typename TriviallyConstructible =
            typename VariantTriviallyConstructible<VariantType>::type,
          typename TriviallyCopyable = typename VariantTriviallyCopyable<VariantType>::type>
struct VariantConstructorImpl;

// Can trivially construct, deconstruct, and copy all data. (Probably all trivial classes.)
template <typename... Ts>
struct VariantConstructorImpl<vtkm::VTK_M_NAMESPACE::internal::Variant<Ts...>,
                              std::true_type,
                              std::true_type> : VariantStorageImpl<Ts...>
{
  VariantConstructorImpl() = default;
  ~VariantConstructorImpl() = default;

  VariantConstructorImpl(const VariantConstructorImpl&) = default;
  VariantConstructorImpl(VariantConstructorImpl&&) = default;
  VariantConstructorImpl& operator=(const VariantConstructorImpl&) = default;
  VariantConstructorImpl& operator=(VariantConstructorImpl&&) = default;
};

// Can trivially copy, but cannot trivially construct. Common if a class is simple but
// initializes itself.
template <typename... Ts>
struct VariantConstructorImpl<vtkm::VTK_M_NAMESPACE::internal::Variant<Ts...>,
                              std::false_type,
                              std::true_type> : VariantStorageImpl<Ts...>
{
  VTK_M_DEVICE VariantConstructorImpl()
    : VariantStorageImpl<Ts...>(vtkm::internal::NullType{})
  {
    this->Index = -1;
  }

  // Any trivially copyable class is trivially destructable.
  ~VariantConstructorImpl() = default;

  VariantConstructorImpl(const VariantConstructorImpl&) = default;
  VariantConstructorImpl(VariantConstructorImpl&&) = default;
  VariantConstructorImpl& operator=(const VariantConstructorImpl&) = default;
  VariantConstructorImpl& operator=(VariantConstructorImpl&&) = default;
};

// Cannot trivially copy. We assume we cannot trivially construct/destruct.
template <typename construct_type, typename... Ts>
struct VariantConstructorImpl<vtkm::VTK_M_NAMESPACE::internal::Variant<Ts...>,
                              construct_type,
                              std::false_type> : VariantStorageImpl<Ts...>
{
  VTK_M_DEVICE VariantConstructorImpl()
    : VariantStorageImpl<Ts...>(vtkm::internal::NullType{})
  {
    this->Index = -1;
  }
  VTK_M_DEVICE ~VariantConstructorImpl() { this->Reset(); }

  VTK_M_DEVICE VariantConstructorImpl(const VariantConstructorImpl& src) noexcept
    : VariantStorageImpl<Ts...>(vtkm::internal::NullType{})
  {
    src.CastAndCall(VariantCopyConstructFunctor{}, this->Storage);
    this->Index = src.Index;
  }

  VTK_M_DEVICE VariantConstructorImpl& operator=(const VariantConstructorImpl& src) noexcept
  {
    if (this->GetIndex() == src.GetIndex())
    {
      src.CastAndCall(detail::VariantCopyFunctor{}, this->Storage);
    }
    else
    {
      this->Reset();
      src.CastAndCall(detail::VariantCopyConstructFunctor{}, this->Storage);
      this->Index = src.Index;
    }
    return *this;
  }
};

} // namespace detail

template <typename... Ts>
class Variant : detail::VariantConstructorImpl<Variant<Ts...>>
{
  using Superclass = detail::VariantConstructorImpl<Variant<Ts...>>;

  // Type not used, but has the compiler check all the types for validity.
  using CheckTypes = vtkm::List<detail::VariantCheckType<Ts>...>;

public:
  /// Type that converts to a std::integral_constant containing the index of the given type (or
  /// -1 if that type is not in the list).
  template <typename T>
  using IndexOf = vtkm::ListIndexOf<vtkm::List<Ts...>, T>;

  /// Returns the index for the given type (or -1 if that type is not in the list).
  ///
  template <typename T>
  VTK_M_DEVICE static constexpr vtkm::IdComponent GetIndexOf()
  {
    return IndexOf<T>::value;
  }

  /// Type that converts to the type at the given index.
  ///
  template <vtkm::IdComponent Index>
  using TypeAt = typename vtkm::ListAt<vtkm::List<Ts...>, Index>;

  /// The number of types representable by this Variant.
  ///
  static constexpr vtkm::IdComponent NumberOfTypes = vtkm::IdComponent{ sizeof...(Ts) };

  /// Returns the index of the type of object this variant is storing. If no object is currently
  /// stored (i.e. the `Variant` is invalid), an invalid is returned.
  ///
  VTK_M_DEVICE vtkm::IdComponent GetIndex() const noexcept { return this->Index; }

  /// Returns true if this `Variant` is storing an object from one of the types in the template
  /// list, false otherwise.
  ///
  /// Note that if this `Variant` was not initialized with an object, the result of `IsValid`
  /// is undefined. The `Variant` could report itself as validly containing an object that
  /// is trivially constructed.
  ///
  VTK_M_DEVICE bool IsValid() const noexcept
  {
    return (this->Index >= 0) && (this->Index < NumberOfTypes);
  }

  Variant() = default;
  ~Variant() = default;
  Variant(const Variant&) = default;
  Variant(Variant&&) = default;
  Variant& operator=(const Variant&) = default;
  Variant& operator=(Variant&&) = default;

  template <typename T>
  VTK_M_DEVICE Variant(const T& src) noexcept
  {
    constexpr vtkm::IdComponent index = GetIndexOf<T>();
    // Might be a way to use an enable_if to enforce a proper type.
    VTKM_STATIC_ASSERT_MSG(index >= 0, "Attempting to put invalid type into a Variant");

    this->Index = index;
    new (&this->Get<index>()) T(src);
  }

  template <typename T>
  VTK_M_DEVICE Variant& operator=(const T& src)
  {
    if (this->GetIndex() == this->GetIndexOf<T>())
    {
      this->Get<T>() = src;
    }
    else
    {
      this->Emplace<T>(src);
    }
    return *this;
  }

  template <typename T, typename... Args>
  VTK_M_DEVICE T& Emplace(Args&&... args)
  {
    constexpr vtkm::IdComponent I = GetIndexOf<T>();
    VTKM_STATIC_ASSERT_MSG(I >= 0, "Variant::Emplace called with invalid type.");
    return this->EmplaceImpl<T, I>(std::forward<Args>(args)...);
  }

  template <typename T, typename U, typename... Args>
  VTK_M_DEVICE T& Emplace(std::initializer_list<U> il, Args&&... args)
  {
    constexpr vtkm::IdComponent I = GetIndexOf<T>();
    VTKM_STATIC_ASSERT_MSG(I >= 0, "Variant::Emplace called with invalid type.");
    return this->EmplaceImpl<T, I>(il, std::forward<Args>(args)...);
  }

  template <vtkm::IdComponent I, typename... Args>
  VTK_M_DEVICE TypeAt<I>& Emplace(Args&&... args)
  {
    VTKM_STATIC_ASSERT_MSG((I >= 0) && (I < NumberOfTypes),
                           "Variant::Emplace called with invalid index");
    return this->EmplaceImpl<TypeAt<I>, I>(std::forward<Args>(args)...);
  }

  template <vtkm::IdComponent I, typename U, typename... Args>
  VTK_M_DEVICE TypeAt<I>& Emplace(std::initializer_list<U> il, Args&&... args)
  {
    VTKM_STATIC_ASSERT_MSG((I >= 0) && (I < NumberOfTypes),
                           "Variant::Emplace called with invalid index");
    return this->EmplaceImpl<TypeAt<I>, I>(il, std::forward<Args>(args)...);
  }

private:
  template <typename T, vtkm::IdComponent I, typename... Args>
  VTK_M_DEVICE T& EmplaceImpl(Args&&... args)
  {
    this->Reset();
    this->Index = I;
    return *(new (&this->Get<I>()) T{ args... });
  }

  template <typename T, vtkm::IdComponent I, typename U, typename... Args>
  VTK_M_DEVICE T& EmplaceImpl(std::initializer_list<U> il, Args&&... args)
  {
    this->Reset();
    this->Index = I;
    return *(new (&this->Get<I>()) T(il, args...));
  }

public:
  //@{
  /// Returns the value as the type at the given index. The behavior is undefined if the
  /// variant does not contain the value at the given index.
  ///
  template <vtkm::IdComponent I>
  VTK_M_DEVICE TypeAt<I>& Get() noexcept
  {
    VTKM_ASSERT(I == this->GetIndex());
    return detail::VariantUnionGet<I>(this->Storage);
  }

  template <vtkm::IdComponent I>
  VTK_M_DEVICE const TypeAt<I>& Get() const noexcept
  {
    VTKM_ASSERT(I == this->GetIndex());
    return detail::VariantUnionGet<I>(this->Storage);
  }
  //@}

  //@{
  /// Returns the value as the given type. The behavior is undefined if the variant does not
  /// contain a value of the given type.
  ///
  template <typename T>
  VTK_M_DEVICE T& Get() noexcept
  {
    VTKM_ASSERT(this->GetIndexOf<T>() == this->GetIndex());
    return detail::VariantUnionGet<IndexOf<T>::value>(this->Storage);
  }

  template <typename T>
  VTK_M_DEVICE const T& Get() const noexcept
  {
    VTKM_ASSERT(this->GetIndexOf<T>() == this->GetIndex());
    return detail::VariantUnionGet<IndexOf<T>::value>(this->Storage);
  }
  //@}

  //@{
  /// Given a functor object, calls the functor with the contained object cast to the appropriate
  /// type. If extra \c args are given, then those are also passed to the functor after the cast
  /// object. If the functor returns a value, that value is returned from \c CastAndCall.
  ///
  /// The results are undefined if the Variant is not valid.
  ///
  template <typename Functor, typename... Args>
  VTK_M_DEVICE auto CastAndCall(Functor&& f, Args&&... args) const
    noexcept(noexcept(f(std::declval<const TypeAt<0>&>(), args...)))
      -> decltype(f(std::declval<const TypeAt<0>&>(), args...))
  {
    VTKM_ASSERT(this->IsValid());
    return detail::VariantCastAndCallImpl(
      this->GetIndex(), std::forward<Functor>(f), this->Storage, std::forward<Args>(args)...);
  }

  template <typename Functor, typename... Args>
  VTK_M_DEVICE auto CastAndCall(Functor&& f, Args&&... args) noexcept(
    noexcept(f(std::declval<const TypeAt<0>&>(), args...)))
    -> decltype(f(std::declval<TypeAt<0>&>(), args...))
  {
    VTKM_ASSERT(this->IsValid());
    return detail::VariantCastAndCallImpl(
      this->GetIndex(), std::forward<Functor>(f), this->Storage, std::forward<Args>(args)...);
  }

  /// Destroys any object the Variant is holding and sets the Variant to an invalid state. This
  /// method is not thread safe.
  ///
  VTK_M_DEVICE void Reset() noexcept
  {
    if (this->IsValid())
    {
      this->CastAndCall(detail::VariantDestroyFunctor{});
      this->Index = -1;
    }
  }
};

/// \brief Convert a ListTag to a Variant.
///
/// Depricated. Use ListAsVariant instead.
///
template <typename ListTag>
using ListTagAsVariant VTKM_DEPRECATED(
  1.6,
  "vtkm::ListTag is no longer supported. Use vtkm::List instead.") =
  vtkm::ListApply<ListTag, vtkm::VTK_M_NAMESPACE::internal::Variant>;

/// \brief Convert a `List` to a `Variant`.
///
template <typename List>
using ListAsVariant = vtkm::ListApply<List, vtkm::VTK_M_NAMESPACE::internal::Variant>;
}
}
} // namespace vtkm::VTK_M_NAMESPACE::internal

#undef VTK_M_DEVICE
#undef VTK_M_NAMESPACE
