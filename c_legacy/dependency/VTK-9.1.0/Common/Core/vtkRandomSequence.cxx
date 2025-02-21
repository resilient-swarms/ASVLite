/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkRandomSequence.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.
=========================================================================*/
#include "vtkRandomSequence.h"

#include <cassert>

//------------------------------------------------------------------------------
vtkRandomSequence::vtkRandomSequence() = default;

//------------------------------------------------------------------------------
vtkRandomSequence::~vtkRandomSequence() = default;

//------------------------------------------------------------------------------
void vtkRandomSequence::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
double vtkRandomSequence::GetNextValue()
{
  this->Next();
  return this->GetValue();
}
