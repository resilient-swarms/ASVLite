/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkGDAL.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkGDAL.h"
#include "vtkInformationIntegerVectorKey.h"
#include "vtkInformationKey.h"
#include "vtkInformationStringKey.h"

vtkGDAL::vtkGDAL() = default;

vtkGDAL::~vtkGDAL() = default;

void vtkGDAL::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

vtkInformationKeyMacro(vtkGDAL, MAP_PROJECTION, String);
vtkInformationKeyRestrictedMacro(vtkGDAL, FLIP_AXIS, IntegerVector, 3);
