/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkHierarchicalDataExtractDataSets.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkHierarchicalDataExtractDataSets.h"

#include "vtkObjectFactory.h"
vtkStandardNewMacro(vtkHierarchicalDataExtractDataSets);

//------------------------------------------------------------------------------
vtkHierarchicalDataExtractDataSets::vtkHierarchicalDataExtractDataSets() = default;

//------------------------------------------------------------------------------
vtkHierarchicalDataExtractDataSets::~vtkHierarchicalDataExtractDataSets() = default;

//------------------------------------------------------------------------------
void vtkHierarchicalDataExtractDataSets::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
