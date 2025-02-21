# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindMEMKIND
---------

Find memkind's headers and library.

Imported Targets
^^^^^^^^^^^^^^^^

This module defines the following :prop_tgt:`IMPORTED` targets:

``MEMKIND::MEMKIND``
  The memkind library, if found.

Result Variables
^^^^^^^^^^^^^^^^

This module will set the following variables in your project:

``MEMKIND_INCLUDE_DIRS``
  where to find memkind.h, etc.
``MEMKIND_LIBRARIES``
  the libraries to link against to use memkind.
``MEMKIND_FOUND``
  true if the memkind headers and libraries were found.

#]=======================================================================]
include(CMakeFindDependencyMacro)

# Look for the header file.
find_path(MEMKIND_INCLUDE_DIR
  NAMES memkind.h
  DOC "memkind include directory")

# Look for the library.
find_library(MEMKIND_LIBRARY
  NAMES memkind libmemkind
  DOC "memkind library")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MEMKIND
  REQUIRED_VARS MEMKIND_LIBRARY MEMKIND_INCLUDE_DIR)

# Copy the results to the output variables and target.
if(MEMKIND_FOUND)
  set(MEMKIND_LIBRARIES ${MEMKIND_LIBRARY})
  set(MEMKIND_INCLUDE_DIRS ${MEMKIND_INCLUDE_DIR})
  set(pkgconfigfile "${MEMKIND_INCLUDE_DIR}/../lib/pkgconfig/memkind.pc")
  set(MEMKIND_VERSION_MINOR 0)
  if(EXISTS "${pkgconfigfile}")
    file(STRINGS "${pkgconfigfile}" MEMKIND_VERSION_LINE REGEX "Version: ")
    string(SUBSTRING "${MEMKIND_VERSION_LINE}" 11 -1 MEMKIND_VERSION_STRING) # skip over "Version: ?.", I wouldn't expect >9  major versions
    string(FIND "${MEMKIND_VERSION_STRING}" "." minorNumLen) 
    string(SUBSTRING "${MEMKIND_VERSION_STRING}" 0 ${minorNumLen} MEMKIND_VERSION_MINOR)
  endif()

  if(NOT TARGET MEMKIND::MEMKIND)
    add_library(MEMKIND::MEMKIND UNKNOWN IMPORTED)
    set_target_properties(MEMKIND::MEMKIND PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES "C"
      IMPORTED_LOCATION "${MEMKIND_LIBRARY}"
      IMPORTED_IMPLIB "${MEMKIND_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MEMKIND_INCLUDE_DIRS}"
      MEMKIND_VERSION_MINOR "${MEMKIND_VERSION_MINOR}")
  endif()
endif()

mark_as_advanced(MEMKIND_INCLUDE_DIR MEMKIND_LIBRARY)
