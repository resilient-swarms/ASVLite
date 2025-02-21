/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayActorNode.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkOSPRayActorNode.h"

#include "vtkActor.h"
#include "vtkCompositeDataIterator.h"
#include "vtkCompositeDataSet.h"
#include "vtkDataArray.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkInformationDoubleKey.h"
#include "vtkInformationIntegerKey.h"
#include "vtkInformationObjectBaseKey.h"
#include "vtkInformationStringKey.h"
#include "vtkMapper.h"
#include "vtkObjectFactory.h"
#include "vtkPiecewiseFunction.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkTexture.h"

#include "RTWrapper/RTWrapper.h"

vtkInformationKeyMacro(vtkOSPRayActorNode, LUMINOSITY, Double);
vtkInformationKeyMacro(vtkOSPRayActorNode, ENABLE_SCALING, Integer);
vtkInformationKeyMacro(vtkOSPRayActorNode, SCALE_ARRAY_NAME, String);
vtkInformationKeyMacro(vtkOSPRayActorNode, SCALE_FUNCTION, ObjectBase);

//============================================================================
vtkStandardNewMacro(vtkOSPRayActorNode);

//------------------------------------------------------------------------------
vtkOSPRayActorNode::vtkOSPRayActorNode()
{
  this->LastMapper = nullptr;
}

//------------------------------------------------------------------------------
vtkOSPRayActorNode::~vtkOSPRayActorNode() = default;

//------------------------------------------------------------------------------
void vtkOSPRayActorNode::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
void vtkOSPRayActorNode::SetEnableScaling(int value, vtkActor* actor)
{
  if (!actor)
  {
    return;
  }
  vtkMapper* mapper = actor->GetMapper();
  if (mapper)
  {
    vtkInformation* info = mapper->GetInformation();
    info->Set(vtkOSPRayActorNode::ENABLE_SCALING(), value);
  }
}

//------------------------------------------------------------------------------
int vtkOSPRayActorNode::GetEnableScaling(vtkActor* actor)
{
  if (!actor)
  {
    return 0;
  }
  vtkMapper* mapper = actor->GetMapper();
  if (mapper)
  {
    vtkInformation* info = mapper->GetInformation();
    if (info && info->Has(vtkOSPRayActorNode::ENABLE_SCALING()))
    {
      return (info->Get(vtkOSPRayActorNode::ENABLE_SCALING()));
    }
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkOSPRayActorNode::SetScaleArrayName(const char* arrayName, vtkActor* actor)
{
  if (!actor)
  {
    return;
  }
  vtkMapper* mapper = actor->GetMapper();
  if (mapper)
  {
    vtkInformation* mapperInfo = mapper->GetInformation();
    mapperInfo->Set(vtkOSPRayActorNode::SCALE_ARRAY_NAME(), arrayName);
  }
}

//------------------------------------------------------------------------------
void vtkOSPRayActorNode::SetScaleFunction(vtkPiecewiseFunction* scaleFunction, vtkActor* actor)
{
  if (!actor)
  {
    return;
  }
  vtkMapper* mapper = actor->GetMapper();
  if (mapper)
  {
    vtkInformation* mapperInfo = mapper->GetInformation();
    mapperInfo->Set(vtkOSPRayActorNode::SCALE_FUNCTION(), scaleFunction);
  }
}

//------------------------------------------------------------------------------
void vtkOSPRayActorNode::SetLuminosity(double value, vtkProperty* property)
{
  if (!property)
  {
    return;
  }
  vtkInformation* info = property->GetInformation();
  info->Set(vtkOSPRayActorNode::LUMINOSITY(), value);
}

//------------------------------------------------------------------------------
double vtkOSPRayActorNode::GetLuminosity(vtkProperty* property)
{
  if (!property)
  {
    return 0.0;
  }
  vtkInformation* info = property->GetInformation();
  if (info && info->Has(vtkOSPRayActorNode::LUMINOSITY()))
  {
    double retval = info->Get(vtkOSPRayActorNode::LUMINOSITY());
    return retval;
  }
  return 0.0;
}

//------------------------------------------------------------------------------
vtkMTimeType vtkOSPRayActorNode::GetMTime()
{
  vtkMTimeType mtime = this->Superclass::GetMTime();
  vtkActor* act = (vtkActor*)this->GetRenderable();
  if (!act)
  {
    return mtime;
  }
  if (act->GetMTime() > mtime)
  {
    mtime = act->GetMTime();
  }
  if (vtkProperty* prop = act->GetProperty())
  {
    if (prop->GetMTime() > mtime)
    {
      mtime = prop->GetMTime();
    }
    if (prop->GetInformation()->GetMTime() > mtime)
    {
      mtime = prop->GetInformation()->GetMTime();
    }
  }
  vtkDataObject* dobj = nullptr;
  vtkPolyData* poly = nullptr;
  vtkMapper* mapper = act->GetMapper();
  vtkTexture* texture = act->GetTexture();
  if (mapper)
  {
    // if (act->GetRedrawMTime() > mtime)
    //  {
    //  mtime = act->GetRedrawMTime();
    // }
    if (mapper->GetMTime() > mtime)
    {
      mtime = mapper->GetMTime();
    }
    if (mapper->GetInformation()->GetMTime() > mtime)
    {
      mtime = mapper->GetInformation()->GetMTime();
    }
    if (mapper != this->LastMapper)
    {
      this->MapperChangedTime.Modified();
      mtime = this->MapperChangedTime;
      this->LastMapper = mapper;
    }
    vtkPiecewiseFunction* pwf = vtkPiecewiseFunction::SafeDownCast(
      mapper->GetInformation()->Get(vtkOSPRayActorNode::SCALE_FUNCTION()));
    if (pwf)
    {
      if (pwf->GetMTime() > mtime)
      {
        mtime = pwf->GetMTime();
      }
    }
    if (mapper->GetNumberOfInputPorts() > 0)
    {
      dobj = mapper->GetInputDataObject(0, 0);
      poly = vtkPolyData::SafeDownCast(dobj);
    }
  }
  if (poly)
  {
    if (poly->GetMTime() > mtime)
    {
      mtime = poly->GetMTime();
    }
  }
  else if (dobj)
  {
    vtkCompositeDataSet* comp = vtkCompositeDataSet::SafeDownCast(dobj);
    if (comp)
    {
      vtkCompositeDataIterator* dit = comp->NewIterator();
      dit->SkipEmptyNodesOn();
      while (!dit->IsDoneWithTraversal())
      {
        poly = vtkPolyData::SafeDownCast(comp->GetDataSet(dit));
        if (poly)
        {
          if (poly->GetMTime() > mtime)
          {
            mtime = poly->GetMTime();
          }
        }
        dit->GoToNextItem();
      }
      dit->Delete();
    }
  }
  if (texture)
  {
    if (texture->GetMTime() > mtime)
    {
      mtime = texture->GetMTime();
    }
    if (texture->GetInput() && texture->GetInput()->GetMTime() > mtime)
    {
      mtime = texture->GetInput()->GetMTime();
    }
  }
  return mtime;
}
