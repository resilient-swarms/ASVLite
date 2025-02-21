//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/cont/Algorithm.h>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/ArrayHandleCompositeVector.h>
#include <vtkm/cont/ArrayHandleCounting.h>
#include <vtkm/cont/ArrayHandleSwizzle.h>

#include <vtkm/cont/testing/Testing.h>

#include <type_traits>

namespace
{

template <typename ValueType>
struct SwizzleTests
{
  static constexpr vtkm::IdComponent InSize = 4;
  using SwizzleInputArrayType = vtkm::cont::ArrayHandle<vtkm::Vec<ValueType, InSize>>;

  template <vtkm::IdComponent OutSize>
  using SwizzleArrayType = vtkm::cont::ArrayHandleSwizzle<SwizzleInputArrayType, OutSize>;

  using ReferenceComponentArrayType = vtkm::cont::ArrayHandleCounting<ValueType>;
  using ReferenceArrayType = vtkm::cont::ArrayHandleCompositeVector<ReferenceComponentArrayType,
                                                                    ReferenceComponentArrayType,
                                                                    ReferenceComponentArrayType,
                                                                    ReferenceComponentArrayType>;

  template <vtkm::IdComponent Size>
  using MapType = vtkm::Vec<vtkm::IdComponent, Size>;

  using Algo = vtkm::cont::Algorithm;

  // This is used to build a ArrayHandleSwizzle's internal array.
  ReferenceArrayType RefArray;

  void ConstructReferenceArray()
  {
    // Build the Ref array
    const vtkm::Id numValues = 32;
    ReferenceComponentArrayType c1 =
      vtkm::cont::make_ArrayHandleCounting<ValueType>(3, 2, numValues);
    ReferenceComponentArrayType c2 =
      vtkm::cont::make_ArrayHandleCounting<ValueType>(2, 3, numValues);
    ReferenceComponentArrayType c3 =
      vtkm::cont::make_ArrayHandleCounting<ValueType>(4, 4, numValues);
    ReferenceComponentArrayType c4 =
      vtkm::cont::make_ArrayHandleCounting<ValueType>(1, 3, numValues);

    this->RefArray = vtkm::cont::make_ArrayHandleCompositeVector(c1, c2, c3, c4);
  }

  SwizzleInputArrayType BuildSwizzleInputArray() const
  {
    SwizzleInputArrayType result;
    Algo::Copy(this->RefArray, result);
    return result;
  }

  template <vtkm::IdComponent OutSize>
  void SanityCheck(const MapType<OutSize>& map) const
  {
    SwizzleInputArrayType input = this->BuildSwizzleInputArray();
    auto swizzle = vtkm::cont::make_ArrayHandleSwizzle(input, map);

    VTKM_TEST_ASSERT(input.GetNumberOfValues() == swizzle.GetNumberOfValues(),
                     "Number of values in copied Swizzle array does not match input.");
  }

  template <vtkm::IdComponent OutSize>
  void ReadTest(const MapType<OutSize>& map) const
  {
    // Test that the expected values are read from an Swizzle array.
    SwizzleInputArrayType input = this->BuildSwizzleInputArray();
    auto swizzle = vtkm::cont::make_ArrayHandleSwizzle(input, map);

    // Test reading the data back in the control env:
    this->ValidateReadTest(swizzle, map);

    // Copy the extracted array in the execution environment to test reading:
    vtkm::cont::ArrayHandle<vtkm::Vec<ValueType, OutSize>> execCopy;
    Algo::Copy(swizzle, execCopy);
    this->ValidateReadTest(execCopy, map);
  }

  template <typename ArrayHandleType, vtkm::IdComponent OutSize>
  void ValidateReadTest(ArrayHandleType testArray, const MapType<OutSize>& map) const
  {
    using ReferenceVectorType = typename ReferenceArrayType::ValueType;
    using SwizzleVectorType = vtkm::Vec<ValueType, OutSize>;

    VTKM_TEST_ASSERT(map.GetNumberOfComponents() ==
                       vtkm::VecTraits<SwizzleVectorType>::NUM_COMPONENTS,
                     "Unexpected runtime component map size.");
    VTKM_TEST_ASSERT(testArray.GetNumberOfValues() == this->RefArray.GetNumberOfValues(),
                     "Number of values incorrect in Read test.");

    auto refPortal = this->RefArray.ReadPortal();
    auto testPortal = testArray.ReadPortal();

    SwizzleVectorType refVecSwizzle(vtkm::TypeTraits<SwizzleVectorType>::ZeroInitialization());
    for (vtkm::Id i = 0; i < testArray.GetNumberOfValues(); ++i)
    {
      ReferenceVectorType refVec = refPortal.Get(i);

      // Manually swizzle the reference vector using the runtime map information:
      for (vtkm::IdComponent j = 0; j < map.GetNumberOfComponents(); ++j)
      {
        refVecSwizzle[j] = refVec[map[j]];
      }

      VTKM_TEST_ASSERT(test_equal(refVecSwizzle, testPortal.Get(i), 0.),
                       "Invalid value encountered in Read test.");
    }
  }

  // Doubles everything in the input portal.
  template <typename PortalType>
  struct WriteTestFunctor : vtkm::exec::FunctorBase
  {
    PortalType Portal;

    VTKM_CONT
    WriteTestFunctor(const PortalType& portal)
      : Portal(portal)
    {
    }

    VTKM_EXEC_CONT
    void operator()(vtkm::Id index) const { this->Portal.Set(index, this->Portal.Get(index) * 2.); }
  };

  struct WriteExec
  {
    template <typename DeviceTag, typename SwizzleHandleType>
    bool operator()(DeviceTag, SwizzleHandleType& swizzle) const
    {
      vtkm::cont::Token token;
      using Portal = typename SwizzleHandleType::WritePortalType;
      WriteTestFunctor<Portal> functor(swizzle.PrepareForInPlace(DeviceTag(), token));
      Algo::Schedule(functor, swizzle.GetNumberOfValues());
      return true;
    }
  };


  template <vtkm::IdComponent OutSize>
  void WriteTest(const MapType<OutSize>& map, std::true_type) const
  {
    // Control test:
    {
      SwizzleInputArrayType input = this->BuildSwizzleInputArray();
      auto swizzle = vtkm::cont::make_ArrayHandleSwizzle(input, map);

      {
        WriteTestFunctor<typename SwizzleArrayType<OutSize>::WritePortalType> functor(
          swizzle.WritePortal());

        for (vtkm::Id i = 0; i < swizzle.GetNumberOfValues(); ++i)
        {
          functor(i);
        }
      }

      this->ValidateWriteTestArray(input, map);
    }

    // Exec test:
    {
      SwizzleInputArrayType input = this->BuildSwizzleInputArray();
      auto swizzle = vtkm::cont::make_ArrayHandleSwizzle(input, map);

      vtkm::cont::TryExecute(WriteExec{}, swizzle);
      this->ValidateWriteTestArray(input, map);
    }
  }

  template <vtkm::IdComponent OutSize>
  void WriteTest(const MapType<OutSize>&, std::false_type) const
  {
    // Array is not writable
  }

  template <vtkm::IdComponent OutSize>
  void WriteTest(const MapType<OutSize>& map) const
  {
    this->WriteTest(map, std::integral_constant<bool, OutSize == InSize>{});
  }

  // Check that the swizzled components are twice the reference value.
  template <vtkm::IdComponent OutSize>
  void ValidateWriteTestArray(SwizzleInputArrayType testArray, const MapType<OutSize>& map) const
  {
    auto refPortal = this->RefArray.ReadPortal();
    auto portal = testArray.ReadPortal();

    VTKM_TEST_ASSERT(portal.GetNumberOfValues() == refPortal.GetNumberOfValues(),
                     "Number of values in write test output do not match input.");

    for (vtkm::Id i = 0; i < portal.GetNumberOfValues(); ++i)
    {
      auto value = portal.Get(i);
      auto refValue = refPortal.Get(i);

      // Double all of the components that appear in the map to replicate the
      // test result:
      for (vtkm::IdComponent j = 0; j < map.GetNumberOfComponents(); ++j)
      {
        refValue[map[j]] *= 2;
      }

      VTKM_TEST_ASSERT(test_equal(refValue, value, 0.), "Value mismatch in Write test.");
    }
  }

  template <vtkm::IdComponent OutSize>
  void TestSwizzle(const MapType<OutSize>& map) const
  {
    this->SanityCheck(map);
    this->ReadTest(map);

    this->WriteTest(map);
  }

  void operator()()
  {
    this->ConstructReferenceArray();

    this->TestSwizzle(vtkm::make_Vec(0, 1));
    this->TestSwizzle(vtkm::make_Vec(0, 2));
    this->TestSwizzle(vtkm::make_Vec(0, 3));
    this->TestSwizzle(vtkm::make_Vec(1, 0));
    this->TestSwizzle(vtkm::make_Vec(1, 2));
    this->TestSwizzle(vtkm::make_Vec(1, 3));
    this->TestSwizzle(vtkm::make_Vec(2, 0));
    this->TestSwizzle(vtkm::make_Vec(2, 1));
    this->TestSwizzle(vtkm::make_Vec(2, 3));
    this->TestSwizzle(vtkm::make_Vec(3, 0));
    this->TestSwizzle(vtkm::make_Vec(3, 1));
    this->TestSwizzle(vtkm::make_Vec(3, 2));
    this->TestSwizzle(vtkm::make_Vec(0, 1, 2));
    this->TestSwizzle(vtkm::make_Vec(0, 1, 3));
    this->TestSwizzle(vtkm::make_Vec(0, 2, 1));
    this->TestSwizzle(vtkm::make_Vec(0, 2, 3));
    this->TestSwizzle(vtkm::make_Vec(0, 3, 1));
    this->TestSwizzle(vtkm::make_Vec(0, 3, 2));
    this->TestSwizzle(vtkm::make_Vec(1, 0, 2));
    this->TestSwizzle(vtkm::make_Vec(1, 0, 3));
    this->TestSwizzle(vtkm::make_Vec(1, 2, 0));
    this->TestSwizzle(vtkm::make_Vec(1, 2, 3));
    this->TestSwizzle(vtkm::make_Vec(1, 3, 0));
    this->TestSwizzle(vtkm::make_Vec(1, 3, 2));
    this->TestSwizzle(vtkm::make_Vec(2, 0, 1));
    this->TestSwizzle(vtkm::make_Vec(2, 0, 3));
    this->TestSwizzle(vtkm::make_Vec(2, 1, 0));
    this->TestSwizzle(vtkm::make_Vec(2, 1, 3));
    this->TestSwizzle(vtkm::make_Vec(2, 3, 0));
    this->TestSwizzle(vtkm::make_Vec(2, 3, 1));
    this->TestSwizzle(vtkm::make_Vec(3, 0, 1));
    this->TestSwizzle(vtkm::make_Vec(3, 0, 2));
    this->TestSwizzle(vtkm::make_Vec(3, 1, 0));
    this->TestSwizzle(vtkm::make_Vec(3, 1, 2));
    this->TestSwizzle(vtkm::make_Vec(3, 2, 0));
    this->TestSwizzle(vtkm::make_Vec(3, 2, 1));
    this->TestSwizzle(vtkm::make_Vec(0, 1, 2, 3));
    this->TestSwizzle(vtkm::make_Vec(0, 1, 3, 2));
    this->TestSwizzle(vtkm::make_Vec(0, 2, 1, 3));
    this->TestSwizzle(vtkm::make_Vec(0, 2, 3, 1));
    this->TestSwizzle(vtkm::make_Vec(0, 3, 1, 2));
    this->TestSwizzle(vtkm::make_Vec(0, 3, 2, 1));
    this->TestSwizzle(vtkm::make_Vec(1, 0, 2, 3));
    this->TestSwizzle(vtkm::make_Vec(1, 0, 3, 2));
    this->TestSwizzle(vtkm::make_Vec(1, 2, 0, 3));
    this->TestSwizzle(vtkm::make_Vec(1, 2, 3, 0));
    this->TestSwizzle(vtkm::make_Vec(1, 3, 0, 2));
    this->TestSwizzle(vtkm::make_Vec(1, 3, 2, 0));
    this->TestSwizzle(vtkm::make_Vec(2, 0, 1, 3));
    this->TestSwizzle(vtkm::make_Vec(2, 0, 3, 1));
    this->TestSwizzle(vtkm::make_Vec(2, 1, 0, 3));
    this->TestSwizzle(vtkm::make_Vec(2, 1, 3, 0));
    this->TestSwizzle(vtkm::make_Vec(2, 3, 0, 1));
    this->TestSwizzle(vtkm::make_Vec(2, 3, 1, 0));
    this->TestSwizzle(vtkm::make_Vec(3, 0, 1, 2));
    this->TestSwizzle(vtkm::make_Vec(3, 0, 2, 1));
    this->TestSwizzle(vtkm::make_Vec(3, 1, 0, 2));
    this->TestSwizzle(vtkm::make_Vec(3, 1, 2, 0));
    this->TestSwizzle(vtkm::make_Vec(3, 2, 0, 1));
    this->TestSwizzle(vtkm::make_Vec(3, 2, 1, 0));
  }
};

struct ArgToTemplateType
{
  template <typename ValueType>
  void operator()(ValueType) const
  {
    SwizzleTests<ValueType>()();
  }
};

void TestArrayHandleSwizzle()
{
  using TestTypes = vtkm::List<vtkm::Int32, vtkm::Int64, vtkm::Float32, vtkm::Float64>;
  vtkm::testing::Testing::TryTypes(ArgToTemplateType(), TestTypes());
}

} // end anon namespace

int UnitTestArrayHandleSwizzle(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(TestArrayHandleSwizzle, argc, argv);
}
