//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/ArrayHandleCounting.h>

#ifndef VTKM_NO_DEPRECATED_VIRTUAL
#include <vtkm/cont/ArrayHandleVirtual.h>
#include <vtkm/cont/ArrayHandleVirtual.hxx>
#endif

#include <vtkm/cont/DeviceAdapterAlgorithm.h>

#include <vtkm/cont/serial/internal/DeviceAdapterAlgorithmSerial.h>
#include <vtkm/cont/serial/internal/DeviceAdapterTagSerial.h>

#include <vtkm/cont/testing/Testing.h>

#include <vtkm/BinaryOperators.h>

#include <algorithm>

#ifndef VTKM_NO_DEPRECATED_VIRTUAL
VTKM_DEPRECATED_SUPPRESS_BEGIN

namespace UnitTestArrayHandleVirtualDetail
{

template <typename ValueType>
struct Test
{
  static constexpr vtkm::Id ARRAY_SIZE = 100;
  static constexpr vtkm::Id NUM_KEYS = 3;

  using ArrayHandle = vtkm::cont::ArrayHandle<ValueType>;
  using VirtHandle = vtkm::cont::ArrayHandleVirtual<ValueType>;
  using DeviceTag = vtkm::cont::DeviceAdapterTagSerial;
  using Algorithm = vtkm::cont::DeviceAdapterAlgorithm<DeviceTag>;

  void TestConstructors()
  {
    std::cout << "Constructors" << std::endl;

    VirtHandle nullStorage;
    VTKM_TEST_ASSERT(nullStorage.GetStorage().GetStorageVirtual() == nullptr,
                     "storage should be empty when using ArrayHandleVirtual().");

    VirtHandle fromArrayHandle{ ArrayHandle{} };
    VTKM_TEST_ASSERT(fromArrayHandle.GetStorage().GetStorageVirtual() != nullptr,
                     "storage should be empty when using ArrayHandleVirtual().");
    VTKM_TEST_ASSERT(vtkm::cont::IsType<ArrayHandle>(fromArrayHandle),
                     "ArrayHandleVirtual should contain a ArrayHandle<ValueType>.");

    VirtHandle fromVirtHandle(fromArrayHandle);
    VTKM_TEST_ASSERT(fromVirtHandle.GetStorage().GetStorageVirtual() != nullptr,
                     "storage should be empty when using ArrayHandleVirtual().");
    VTKM_TEST_ASSERT(vtkm::cont::IsType<ArrayHandle>(fromVirtHandle),
                     "ArrayHandleVirtual should contain a ArrayHandle<ValueType>.");

    VirtHandle fromNullPtrHandle(nullStorage);
    VTKM_TEST_ASSERT(fromNullPtrHandle.GetStorage().GetStorageVirtual() == nullptr,
                     "storage should be empty when constructing from a ArrayHandleVirtual that has "
                     "nullptr storage.");
    VTKM_TEST_ASSERT((vtkm::cont::IsType<ArrayHandle>(fromNullPtrHandle) == false),
                     "ArrayHandleVirtual shouldn't match any type with nullptr storage.");
  }


  void TestMoveConstructors()
  {
    std::cout << "Move constructors" << std::endl;

    //test ArrayHandle move constructor
    {
      ArrayHandle handle;
      VirtHandle virt(std::move(handle));
      VTKM_TEST_ASSERT(
        vtkm::cont::IsType<ArrayHandle>(virt),
        "ArrayHandleVirtual should be valid after move constructor ArrayHandle<ValueType>.");
    }

    //test ArrayHandleVirtual move constructor
    {
      ArrayHandle handle;
      VirtHandle virt(std::move(handle));
      VirtHandle virt2(std::move(virt));
      VTKM_TEST_ASSERT(
        vtkm::cont::IsType<ArrayHandle>(virt2),
        "ArrayHandleVirtual should be valid after move constructor ArrayHandleVirtual<ValueType>.");
    }
  }

  void TestAssignmentOps()
  {
    std::cout << "Assignment operators" << std::endl;

    //test assignment operator from ArrayHandleVirtual
    {
      VirtHandle virt;
      virt = VirtHandle{ ArrayHandle{} };
      VTKM_TEST_ASSERT(vtkm::cont::IsType<ArrayHandle>(virt),
                       "ArrayHandleVirtual should be valid after assignment op from AHV.");
    }

    //test assignment operator from ArrayHandle
    {
      VirtHandle virt = vtkm::cont::ArrayHandleCounting<ValueType>{};
      virt = ArrayHandle{};
      VTKM_TEST_ASSERT(vtkm::cont::IsType<ArrayHandle>(virt),
                       "ArrayHandleVirtual should be valid after assignment op from AH.");
    }

    //test move assignment operator from ArrayHandleVirtual
    {
      VirtHandle temp{ ArrayHandle{} };
      VirtHandle virt;
      virt = std::move(temp);
      VTKM_TEST_ASSERT(vtkm::cont::IsType<ArrayHandle>(virt),
                       "ArrayHandleVirtual should be valid after move assignment op from AHV.");
    }

    //test move assignment operator from ArrayHandle
    {
      vtkm::cont::ArrayHandleCounting<ValueType> temp;
      VirtHandle virt;
      virt = std::move(temp);
      VTKM_TEST_ASSERT(vtkm::cont::IsType<decltype(temp)>(virt),
                       "ArrayHandleVirtual should be valid after move assignment op from AH.");
    }
  }

  void TestPrepareForExecution()
  {
    std::cout << "Prepare for execution" << std::endl;

    vtkm::cont::ArrayHandle<ValueType> handle;
    handle.Allocate(ARRAY_SIZE);

    VirtHandle virt(std::move(handle));

    try
    {
      vtkm::cont::Token token;
      virt.PrepareForInput(DeviceTag(), token);
      virt.PrepareForInPlace(DeviceTag(), token);
      virt.PrepareForOutput(ARRAY_SIZE, DeviceTag(), token);
    }
    catch (vtkm::cont::ErrorBadValue&)
    {
      // un-expected failure.
      VTKM_TEST_FAIL(
        "Unexpected error when using Prepare* on an ArrayHandleVirtual with StorageAny.");
    }
  }


  void TestIsType()
  {
    std::cout << "IsType" << std::endl;

    vtkm::cont::ArrayHandle<ValueType> handle;
    VirtHandle virt(std::move(handle));

    VTKM_TEST_ASSERT(vtkm::cont::IsType<decltype(virt)>(virt),
                     "virt should by same type as decltype(virt)");
    VTKM_TEST_ASSERT(vtkm::cont::IsType<decltype(handle)>(virt),
                     "virt should by same type as decltype(handle)");

    vtkm::cont::ArrayHandle<vtkm::Vec<ValueType, 3>> vecHandle;
    VTKM_TEST_ASSERT(!vtkm::cont::IsType<decltype(vecHandle)>(virt),
                     "virt shouldn't by same type as decltype(vecHandle)");
  }

  void TestCast()
  {
    std::cout << "Cast" << std::endl;

    vtkm::cont::ArrayHandle<ValueType> handle;
    VirtHandle virt(handle);

    auto c1 = vtkm::cont::Cast<decltype(virt)>(virt);
    VTKM_TEST_ASSERT(c1 == virt, "virt should cast to VirtHandle");

    auto c2 = vtkm::cont::Cast<decltype(handle)>(virt);
    VTKM_TEST_ASSERT(c2 == handle, "virt should cast to HandleType");

    using VecHandle = vtkm::cont::ArrayHandle<vtkm::Vec<ValueType, 3>>;
    try
    {
      auto c3 = vtkm::cont::Cast<VecHandle>(virt);
      VTKM_TEST_FAIL("Cast of T to Vec<T,3> should have failed");
    }
    catch (vtkm::cont::ErrorBadType&)
    {
    }
  }

  void TestControlPortalLocking()
  {
    std::cout << "Control portal locking" << std::endl;

    // There was a bug where a control portal was not relinquished and it locked the
    // ArrayHandle from further use.

    ArrayHandle concreteArray;
    concreteArray.Allocate(ARRAY_SIZE);

    VirtHandle virtualArray(concreteArray);

    // Make sure you can write to the virtualArray and then read the data from the concreteArray
    // without the concreteArray getting locked up.
    SetPortal(virtualArray.WritePortal());
    CheckPortal(concreteArray.ReadPortal());

    // Make sure you can read from the virtualArray and the write to the concreteArray without
    // the concreteArray getting locked up.
    CheckPortal(virtualArray.ReadPortal());
    SetPortal(concreteArray.WritePortal());
  }

  void operator()()
  {
    std::cout << std::endl;
    std::cout << "### Testing for " << vtkm::cont::TypeToString<ValueType>() << std::endl;
    TestConstructors();
    TestMoveConstructors();
    TestAssignmentOps();
    TestPrepareForExecution();
    TestIsType();
    TestCast();
    TestControlPortalLocking();
  }
};

void TestArrayHandleVirtual()
{
  Test<vtkm::UInt8>()();
  Test<vtkm::Int16>()();
  Test<vtkm::Int32>()();
  Test<vtkm::Int64>()();
  Test<vtkm::Float32>()();
  Test<vtkm::Float64>()();
}

} // end namespace UnitTestArrayHandleVirtualDetail

VTKM_DEPRECATED_SUPPRESS_END
#endif //VTKM_NO_DEPRECATED_VIRTUAL

int UnitTestArrayHandleVirtual(int argc, char* argv[])
{
#ifndef VTKM_NO_DEPRECATED_VIRTUAL
  using namespace UnitTestArrayHandleVirtualDetail;
  return vtkm::cont::testing::Testing::Run(TestArrayHandleVirtual, argc, argv);
#else
  (void)argc;
  (void)argv;
  return 0;
#endif
}
