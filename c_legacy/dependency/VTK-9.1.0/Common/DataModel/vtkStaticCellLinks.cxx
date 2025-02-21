/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkStaticCellLinks.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkStaticCellLinks.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkStaticCellLinks);

//------------------------------------------------------------------------------
vtkStaticCellLinks::vtkStaticCellLinks()
{
  this->Type = vtkAbstractCellLinks::STATIC_CELL_LINKS_IDTYPE;
  this->Impl = new vtkStaticCellLinksTemplate<vtkIdType>;
}

//------------------------------------------------------------------------------
vtkStaticCellLinks::~vtkStaticCellLinks()
{
  delete this->Impl;
}

//------------------------------------------------------------------------------
void vtkStaticCellLinks::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Implementation: " << this->Impl << "\n";
}
