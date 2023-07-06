/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkWrapPythonOverload.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef vtkWrapPythonOverload_h
#define vtkWrapPythonOverload_h

#include "vtkParse.h"
#include "vtkParseData.h"
#include "vtkParseHierarchy.h"

/* output the method table for all overloads of a particular method */
void vtkWrapPython_OverloadMethodDef(FILE* fp, const char* classname, ClassInfo* data,
  int* overloadMap, FunctionInfo** wrappedFunctions, int numberOfWrappedFunctions, int fnum,
  int numberOfOccurrences);

/* a master method to choose which overload to call */
void vtkWrapPython_OverloadMasterMethod(FILE* fp, const char* classname, int* overloadMap,
  int maxArgs, FunctionInfo** wrappedFunctions, int numberOfWrappedFunctions, int fnum,
  int is_vtkobject);

/* generate an int array that maps arg counts to overloads */
int* vtkWrapPython_ArgCountToOverloadMap(FunctionInfo** wrappedFunctions,
  int numberOfWrappedFunctions, int fnum, int is_vtkobject, int* nmax, int* overlap);

#endif /* vtkWrapPythonOverload_h */
/* VTK-HeaderTest-Exclude: vtkWrapPythonOverload.h */
