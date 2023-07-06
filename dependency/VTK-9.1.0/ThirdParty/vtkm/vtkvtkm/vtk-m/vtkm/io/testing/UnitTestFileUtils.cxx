//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/cont/testing/Testing.h>
#include <vtkm/io/FileUtils.h>

#ifdef _MSC_VER
#include <direct.h>
#include <process.h>
#else
#include <unistd.h>
#endif

#include <random>
#include <string>

using namespace vtkm::io;

namespace
{
void TestEndsWith()
{
  VTKM_TEST_ASSERT(EndsWith("checking.val", ".val"), "Ending did not match '.val'");
  VTKM_TEST_ASSERT(EndsWith("special_char$&#*", "_char$&#*"), "Ending did not match '_char$&#*'");
  VTKM_TEST_ASSERT(!EndsWith("wrong_ending", "fing"), "Ending did not match 'fing'");
  VTKM_TEST_ASSERT(!EndsWith("too_long", "ending_too_long"),
                   "Ending did not match 'ending_too_long'");
  VTKM_TEST_ASSERT(EndsWith("empty_string", ""), "Ending did not match ''");
}

void TestGetWindowsPathSeperator()
{
  VTKM_TEST_ASSERT(GetWindowsPathSeperator("some/test/path") == '/', "/ should be the separator");
  VTKM_TEST_ASSERT(GetWindowsPathSeperator("some\\test\\path") == '\\',
                   "\\ should be the seperator");
  VTKM_TEST_ASSERT(GetWindowsPathSeperator("some\\test/path") == '/', "Always prefer / over \\");
  VTKM_TEST_ASSERT(GetWindowsPathSeperator("some/test\\path") == '/', "Always prefer / over \\");
}

void TestFilename()
{
  VTKM_TEST_ASSERT(Filename("filename.txt") == "filename.txt",
                   "Should not affect filename without dir");
  VTKM_TEST_ASSERT(Filename("test/path/filename.txt") == "filename.txt", "Should strip linux path");
  VTKM_TEST_ASSERT(Filename("test/path/dir/") == "", "Should return empty string if ends in a dir");
#ifdef _MSC_VER
  VTKM_TEST_ASSERT(Filename("C:\\windows\\path\\filename.txt") == "filename.txt",
                   "Should strip windows paths");
  VTKM_TEST_ASSERT(Filename("test\\path\\dir\\") == "",
                   "Should return empty string if ends in a dir");
#endif
}

void TestParentPath()
{
  VTKM_TEST_ASSERT(ParentPath("filename.txt") == "", "Should return empty string");
  VTKM_TEST_ASSERT(ParentPath("test/path/filename.txt") == "test/path", "Should strip Linux file");
  VTKM_TEST_ASSERT(ParentPath("test/path/dir/") == "test/path/dir",
                   "Should remove only the trailing /");
#ifdef _MSC_VER
  VTKM_TEST_ASSERT(ParentPath("C:\\windows\\path\\filename.txt") == "C:\\windows\\path",
                   "Should strip the Windows file");
  VTKM_TEST_ASSERT(ParentPath("test\\path\\dir\\") == "test\\path\\dir",
                   "Should remove only the trailing \\");
#endif
}

void TestCreateDirectoriesFromFilePath()
{
  VTKM_TEST_ASSERT(!CreateDirectoriesFromFilePath("filename.txt"),
                   "no dir to create from file path, should return false");
#ifdef _MSC_VER
  vtkm::Id pid = _getpid();
#else
  vtkm::Id pid = getpid();
#endif
  std::string baseDir;
  vtkm::cont::TryExecute(
    [](const vtkm::cont::DeviceAdapterId& device, vtkm::Id id, std::string& dir) {
      dir = "test_dir" + std::to_string(device.GetValue()) + "_id" + std::to_string(id);
      return true;
    },
    pid,
    baseDir);

  VTKM_TEST_ASSERT(CreateDirectoriesFromFilePath(baseDir + "/filename.txt"),
                   "Should create the " + baseDir + " dir");
  VTKM_TEST_ASSERT(!CreateDirectoriesFromFilePath(baseDir + "/filename.txt"),
                   baseDir + " was just created, should be false");
  VTKM_TEST_ASSERT(CreateDirectoriesFromFilePath(baseDir + "/test_1/"),
                   "Should create the 'test_1' dir");
  VTKM_TEST_ASSERT(CreateDirectoriesFromFilePath(baseDir + "/test_2/test_3/file"),
                   "should create the full path 'test_2/test_3' in " + baseDir);
#ifdef _MSC_VER
  baseDir = "win_" + baseDir;
  VTKM_TEST_ASSERT(CreateDirectoriesFromFilePath(baseDir + "\\filename.txt"),
                   "Should create the " + baseDir + " dir");
  VTKM_TEST_ASSERT(!CreateDirectoriesFromFilePath(baseDir + "\\filename.txt"),
                   baseDir + " was just created, should be false");
  VTKM_TEST_ASSERT(CreateDirectoriesFromFilePath(baseDir + "\\test_1\\"),
                   "Should create the 'test_1' dir");
  VTKM_TEST_ASSERT(CreateDirectoriesFromFilePath(baseDir + "\\test_2\\test_3\\file"),
                   "should create the full path 'test_2\\test_3' in " + baseDir);
#endif
}

void TestMergePaths()
{
  VTKM_TEST_ASSERT(MergePaths("some/path", "filename.txt") == "some/path/filename.txt",
                   "should append filename.txt " + MergePaths("some/path", "filename.txt"));
  VTKM_TEST_ASSERT(MergePaths("", "filename.txt") == "filename.txt",
                   "should just return the suffix");
  VTKM_TEST_ASSERT(MergePaths("some/path", "") == "some/path", "should just return the prefix");
  VTKM_TEST_ASSERT(MergePaths("end/in/slash/", "/start/slash") == "end/in/slash/start/slash",
                   "Should do correct slash merge");
  VTKM_TEST_ASSERT(MergePaths("", "") == "", "Empty paths, empty return string");
#ifdef _MSC_VER
  VTKM_TEST_ASSERT(MergePaths("some\\path", "filename.txt") == "some\\path\\filename.txt",
                   "should append filename.txt");
  VTKM_TEST_ASSERT(MergePaths("some\\path", "") == "some\\path", "should just return the prefix");
  VTKM_TEST_ASSERT(MergePaths("end\\in\\slash\\", "\\start\\slash") ==
                     "end\\in\\slash\\start\\slash",
                   "Should do correct slash merge");
  VTKM_TEST_ASSERT(MergePaths("bad\\combo", "bad/combo") == "bad\\combo\\bad/combo",
                   "Should use the prefix seperator");
  VTKM_TEST_ASSERT(MergePaths("bad\\combo", "/bad/combo") == "bad\\combo\\bad/combo",
                   "Should use the prefix seperator");
  VTKM_TEST_ASSERT(MergePaths("bad/combo", "\\bad\\combo") == "bad/combo/bad\\combo",
                   "Should use the prefix seperator");
#endif
}

void TestPrefixStringToFilename()
{
  VTKM_TEST_ASSERT(PrefixStringToFilename("some/path/filename.txt", "prefix-") ==
                     "some/path/prefix-filename.txt",
                   "should prefix file");
  VTKM_TEST_ASSERT(PrefixStringToFilename("/path/here.txt", "dir/prefix-") ==
                     "/path/dir/prefix-here.txt",
                   "should prepend dir+prefix");
  VTKM_TEST_ASSERT(PrefixStringToFilename("filename.txt", "prefix-") == "prefix-filename.txt",
                   "should prefix only file");
  VTKM_TEST_ASSERT(PrefixStringToFilename("some/path/", "prefix-") == "some/path/prefix-",
                   "should append to file, not dir");
  VTKM_TEST_ASSERT(PrefixStringToFilename("", "prefix-") == "prefix-",
                   "should just return the prefix-");
  VTKM_TEST_ASSERT(PrefixStringToFilename("", "") == "", "Should return empty string");
  VTKM_TEST_ASSERT(PrefixStringToFilename("some/path/filename.txt", "") == "some/path/filename.txt",
                   "should return file path");
#ifdef _MSC_VER
  VTKM_TEST_ASSERT(PrefixStringToFilename("some\\path\\filename.txt", "prefix-") ==
                     "some\\path\\prefix-filename.txt",
                   "should prefix file");
  VTKM_TEST_ASSERT(PrefixStringToFilename("\\path\\here.txt", "dir\\prefix-") ==
                     "\\path\\dir\\prefix-here.txt",
                   "should prepend dir+prefix");
  VTKM_TEST_ASSERT(PrefixStringToFilename("some\\path\\", "prefix-") == "some\\path\\prefix-",
                   "should append to file, not dir");
  VTKM_TEST_ASSERT(PrefixStringToFilename("some\\path\\filename.txt", "") ==
                     "some\\path\\filename.txt",
                   "should return file path");
#endif
}


void TestUtils()
{
  TestEndsWith();
  TestGetWindowsPathSeperator();
  TestFilename();
  TestParentPath();
  TestCreateDirectoriesFromFilePath();
  TestMergePaths();
  TestPrefixStringToFilename();
}

} // namespace

int UnitTestFileUtils(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(TestUtils, argc, argv);
}
