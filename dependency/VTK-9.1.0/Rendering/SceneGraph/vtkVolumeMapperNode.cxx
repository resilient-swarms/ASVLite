/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVolumeMapperNode.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkVolumeMapperNode.h"

#include "vtkActor.h"
#include "vtkCellArray.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolygon.h"
#include "vtkProperty.h"

//============================================================================
vtkStandardNewMacro(vtkVolumeMapperNode);

//------------------------------------------------------------------------------
vtkVolumeMapperNode::vtkVolumeMapperNode() = default;

//------------------------------------------------------------------------------
vtkVolumeMapperNode::~vtkVolumeMapperNode() = default;

//------------------------------------------------------------------------------
void vtkVolumeMapperNode::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
