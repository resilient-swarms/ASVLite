/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVolumetricPass.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkVolumetricPass.h"
#include "vtkObjectFactory.h"
#include <cassert>

vtkStandardNewMacro(vtkVolumetricPass);

//------------------------------------------------------------------------------
vtkVolumetricPass::vtkVolumetricPass() = default;

//------------------------------------------------------------------------------
vtkVolumetricPass::~vtkVolumetricPass() = default;

//------------------------------------------------------------------------------
void vtkVolumetricPass::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
// Description:
// Perform rendering according to a render state \p s.
// \pre s_exists: s!=0
void vtkVolumetricPass::Render(const vtkRenderState* s)
{
  assert("pre: s_exists" && s != nullptr);

  this->NumberOfRenderedProps = 0;
  this->RenderFilteredVolumetricGeometry(s);
}
