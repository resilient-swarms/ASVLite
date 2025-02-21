/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkDeprecation.h

-------------------------------------------------------------------------
  Copyright 2008 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
  the U.S. Government retains certain rights in this software.
-------------------------------------------------------------------------

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef vtkDeprecation_h
#define vtkDeprecation_h

#include "vtkVersion.h"

//----------------------------------------------------------------------------
// These macros may be used to deprecate APIs in VTK. They act as attributes on
// method declarations and do not remove methods from a build based on build
// configuration.
//
// To use:
//
// In the declaration:
//
// ```cxx
// VTK_DEPRECATED_IN_9_1_0("reason for the deprecation")
// void oldApi();
// ```
//
// When selecting which version to deprecate an API in, use the newest macro
// available in this header.
//
// In the implementation:
//
// ```cxx
// // Hide VTK_DEPRECATED_IN_9_1_0() warnings for this class.
// #define VTK_DEPRECATION_LEVEL 0
//
// #include "vtkLegacy.h"
//
// void oldApi()
// {
//   // One of:
//   VTK_LEGACY_BODY(oldApi, "VTK 9.1");
//   VTK_LEGACY_REPLACED_BODY(oldApi, "VTK 9.1", newApi);
//
//   // Remaining implementation.
// }
// ```
//
// Please note the `VTK_DEPRECATED_IN_` version in the `VTK_DEPRECATION_LEVEL`
// comment so that it can be removed when that version is finally removed.
//----------------------------------------------------------------------------

// The level at which warnings should be made.
#ifndef VTK_DEPRECATION_LEVEL
// VTK defaults to deprecation of its current version.
#include "vtkVersionMacros.h"
#define VTK_DEPRECATION_LEVEL VTK_VERSION_NUMBER
#endif

// API deprecated before 8.2.0 have already been removed.
#define VTK_MINIMUM_DEPRECATION_LEVEL VTK_VERSION_CHECK(8, 2, 0)

// Force the deprecation level to be at least that of VTK's build
// configuration.
#if VTK_DEPRECATION_LEVEL < VTK_MINIMUM_DEPRECATION_LEVEL
#undef VTK_DEPRECATION_LEVEL
#define VTK_DEPRECATION_LEVEL VTK_MINIMUM_DEPRECATION_LEVEL
#endif

// Deprecation macro support for various compilers.
#if 0 && __cplusplus >= 201402L
// This is currently hard-disabled because compilers do not mix C++ attributes
// and `__attribute__` extensions together well.
#define VTK_DEPRECATION(reason) [[deprecated(reason)]]
#elif defined(VTK_WRAPPING_CXX)
// Ignore deprecation in wrapper code.
#define VTK_DEPRECATION(reason)
#elif defined(__VTK_WRAP__)
#define VTK_DEPRECATION(reason) [[vtk::deprecated(reason)]]
#else
#if defined(_WIN32) || defined(_WIN64)
#define VTK_DEPRECATION(reason) __declspec(deprecated(reason))
#elif defined(__clang__)
#if __has_extension(attribute_deprecated_with_message)
#define VTK_DEPRECATION(reason) __attribute__((__deprecated__(reason)))
#else
#define VTK_DEPRECATION(reason) __attribute__((__deprecated__))
#endif
#elif defined(__GNUC__)
#if (__GNUC__ >= 5) || ((__GNUC__ == 4) && (__GNUC_MINOR__ >= 5))
#define VTK_DEPRECATION(reason) __attribute__((__deprecated__(reason)))
#else
#define VTK_DEPRECATION(reason) __attribute__((__deprecated__))
#endif
#else
#define VTK_DEPRECATION(reason)
#endif
#endif

// APIs deprecated in the next release.
#if defined(__VTK_WRAP__)
#define VTK_DEPRECATED_IN_9_1_0(reason) [[vtk::deprecated(reason, "9.1.0")]]
#elif VTK_DEPRECATION_LEVEL >= VTK_VERSION_CHECK(9, 1, 0)
#define VTK_DEPRECATED_IN_9_1_0(reason) VTK_DEPRECATION(reason)
#else
#define VTK_DEPRECATED_IN_9_1_0(reason)
#endif

// APIs deprecated in 9.0.0.
#if defined(__VTK_WRAP__)
#define VTK_DEPRECATED_IN_9_0_0(reason) [[vtk::deprecated(reason, "9.0.0")]]
#elif VTK_DEPRECATION_LEVEL >= VTK_VERSION_CHECK(9, 0, 0)
#define VTK_DEPRECATED_IN_9_0_0(reason) VTK_DEPRECATION(reason)
#else
#define VTK_DEPRECATED_IN_9_0_0(reason)
#endif

// APIs deprecated in the older release always warn.
#if defined(__VTK_WRAP__)
#define VTK_DEPRECATED_IN_8_2_0(reason) [[vtk::deprecated(reason, "8.2.0")]]
#else
#define VTK_DEPRECATED_IN_8_2_0(reason) VTK_DEPRECATION(reason)
#endif

#endif

// VTK-HeaderTest-Exclude: vtkDeprecation.h
