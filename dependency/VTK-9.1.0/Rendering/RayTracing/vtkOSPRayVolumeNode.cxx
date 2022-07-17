/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayVolumeNode.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkOSPRayVolumeNode.h"

#include "vtkAbstractVolumeMapper.h"
#include "vtkActor.h"
#include "vtkCompositeDataIterator.h"
#include "vtkCompositeDataSet.h"
#include "vtkDataArray.h"
#include "vtkInformation.h"
#include "vtkInformationIntegerKey.h"
#include "vtkInformationObjectBaseKey.h"
#include "vtkInformationStringKey.h"
#include "vtkMapper.h"
#include "vtkObjectFactory.h"
#include "vtkPiecewiseFunction.h"
#include "vtkPolyData.h"
#include "vtkVolume.h"
#include "vtkVolumeProperty.h"

#include "RTWrapper/RTWrapper.h"

//============================================================================
vtkStandardNewMacro(vtkOSPRayVolumeNode);

//------------------------------------------------------------------------------
vtkOSPRayVolumeNode::vtkOSPRayVolumeNode() = default;

//------------------------------------------------------------------------------
vtkOSPRayVolumeNode::~vtkOSPRayVolumeNode() = default;

//------------------------------------------------------------------------------
void vtkOSPRayVolumeNode::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
vtkMTimeType vtkOSPRayVolumeNode::GetMTime()
{
  vtkMTimeType mtime = this->Superclass::GetMTime();
  vtkVolume* vol = (vtkVolume*)this->GetRenderable();
  if (!vol)
  {
    return mtime;
  }

  if (vol->GetMTime() > mtime)
  {
    mtime = vol->GetMTime();
  }
  if (vol->GetProperty())
  {
    mtime = std::max(mtime, vol->GetProperty()->GetMTime());
  }
  vtkAbstractVolumeMapper* mapper = vol->GetMapper();

  if (mapper)
  {
    vtkDataObject* dobj = mapper->GetDataSetInput();
    if (dobj)
    {
      mtime = std::max(mtime, dobj->GetMTime());
    }
    if (mapper->GetMTime() > mtime)
    {
      mtime = mapper->GetMTime();
    }
    if (mapper->GetInformation()->GetMTime() > mtime)
    {
      mtime = mapper->GetInformation()->GetMTime();
    }
  }
  return mtime;
}
