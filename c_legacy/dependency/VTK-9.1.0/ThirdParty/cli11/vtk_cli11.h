/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtk_cli11.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#ifndef vtk_cli11_h
#define vtk_cli11_h

// VTK_MODULE_USE_EXTERNAL_VTK_cli11 is defined in this header,
// so include it.
#include <vtk_cli11_external.h>

#if VTK_MODULE_USE_EXTERNAL_VTK_cli11
# include <CLI/CLI.hpp>
#else

#if defined(vtk_cli11_forward_h) && defined(VTK_CLI)
// vtk_cli11_forward.h defines VTK_CLI to help mangle forward declarations.
// However that can conflict with definitions in CLI.hpp, so we undef it here,
// if the header was already included.
#undef VTK_CLI
#endif

# include <vtkcli11/CLI/CLI.hpp>
#endif // VTK_MODULE_USE_EXTERNAL_VTK_cli11

#endif
