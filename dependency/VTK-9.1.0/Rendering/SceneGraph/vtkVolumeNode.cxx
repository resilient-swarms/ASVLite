/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVolumeNode.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVolumeNode.h"

#include "vtkAbstractVolumeMapper.h"
#include "vtkCellArray.h"
#include "vtkMapper.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolygon.h"
#include "vtkProperty.h"
#include "vtkVolume.h"

//============================================================================
vtkStandardNewMacro(vtkVolumeNode);

//------------------------------------------------------------------------------
vtkVolumeNode::vtkVolumeNode() = default;

//------------------------------------------------------------------------------
vtkVolumeNode::~vtkVolumeNode() = default;

//------------------------------------------------------------------------------
void vtkVolumeNode::Build(bool prepass)
{
  if (prepass)
  {
    vtkVolume* mine = vtkVolume::SafeDownCast(this->GetRenderable());
    if (!mine)
    {
      return;
    }
    if (!mine->GetMapper())
    {
      return;
    }

    this->PrepareNodes();
    this->AddMissingNode(mine->GetMapper());
    this->RemoveUnusedNodes();
  }
}

//------------------------------------------------------------------------------
void vtkVolumeNode::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
