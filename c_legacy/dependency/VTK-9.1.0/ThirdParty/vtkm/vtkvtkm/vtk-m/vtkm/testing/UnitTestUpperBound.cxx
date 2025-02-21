//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/UpperBound.h>

#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/Invoker.h>

#include <vtkm/worklet/WorkletMapField.h>

#include <vtkm/cont/testing/Testing.h>

#include <vector>

namespace
{

using IdArray = vtkm::cont::ArrayHandle<vtkm::Id>;

struct TestUpperBound
{
  struct Impl : public vtkm::worklet::WorkletMapField
  {
    using ControlSignature = void(FieldIn needles, WholeArrayIn haystack, FieldOut results);
    using ExecutionSignature = _3(_1, _2);
    using InputDomain = _1;

    template <typename HaystackPortal>
    VTKM_EXEC vtkm::Id operator()(vtkm::Id needle, const HaystackPortal& haystack) const
    {
      return vtkm::UpperBound(haystack, needle);
    }
  };

  static void Run()
  {
    IdArray needles = vtkm::cont::make_ArrayHandle<vtkm::Id>({ -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 });
    IdArray haystack =
      vtkm::cont::make_ArrayHandle<vtkm::Id>({ -3, -2, -2, -2, 0, 0, 1, 1, 1, 4, 4 });
    IdArray results;

    std::vector<vtkm::Id> expected{ 0, 1, 4, 4, 6, 9, 9, 9, 11, 11 };

    vtkm::cont::Invoker invoke;
    invoke(Impl{}, needles, haystack, results);

    // Verify:
    auto resultsPortal = results.ReadPortal();
    for (vtkm::Id i = 0; i < needles.GetNumberOfValues(); ++i)
    {
      VTKM_TEST_ASSERT(resultsPortal.Get(i) == expected[static_cast<size_t>(i)]);
    }
  }
};

void RunUpperBoundTest()
{
  std::cout << "Testing upper bound." << std::endl;
  TestUpperBound::Run();
}

} // anon namespace

int UnitTestUpperBound(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(RunUpperBoundTest, argc, argv);
}
