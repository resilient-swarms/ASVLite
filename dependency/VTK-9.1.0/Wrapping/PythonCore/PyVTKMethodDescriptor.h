/*=========================================================================

  Program:   Visualization Toolkit
  Module:    PyVTKMethodDescriptor.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*-----------------------------------------------------------------------
  The PyVTKMethodDescriptor was created in July 2015 by David Gobbi.

  Python's built-in method descriptor can only be used for non-static
  method calls.  VTK, however, has many methods where one signature of
  the method is static and another signature of the method is not. In
  order to wrap these methods, a custom method descriptor is needed.
-----------------------------------------------------------------------*/

#ifndef PyVTKMethodDescriptor_h
#define PyVTKMethodDescriptor_h

#include "vtkPython.h"
#include "vtkSystemIncludes.h"
#include "vtkWrappingPythonCoreModule.h" // For export macro

extern PyTypeObject PyVTKMethodDescriptor_Type;

#define PyVTKMethodDescriptor_Check(obj) (Py_TYPE(obj) == &PyVTKMethodDescriptor_Type)

extern "C"
{
  // Create a new method descriptor from a PyMethodDef.
  PyObject* PyVTKMethodDescriptor_New(PyTypeObject* cls, PyMethodDef* meth);
}

#endif
