/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkXMLHierarchicalDataReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkXMLHierarchicalDataReader.h"

#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkXMLHierarchicalDataReader);
//------------------------------------------------------------------------------
vtkXMLHierarchicalDataReader::vtkXMLHierarchicalDataReader() = default;

//------------------------------------------------------------------------------
vtkXMLHierarchicalDataReader::~vtkXMLHierarchicalDataReader() = default;

//------------------------------------------------------------------------------
void vtkXMLHierarchicalDataReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
