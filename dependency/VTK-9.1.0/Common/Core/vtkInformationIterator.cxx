/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkInformationIterator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkInformationIterator.h"

#include "vtkInformation.h"
#include "vtkInformationInternals.h"
#include "vtkInformationKey.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkInformationIterator);

class vtkInformationIteratorInternals
{
public:
  vtkInformationInternals::MapType::iterator Iterator;
};

//------------------------------------------------------------------------------
vtkInformationIterator::vtkInformationIterator()
{
  this->Internal = new vtkInformationIteratorInternals;
  this->Information = nullptr;
  this->ReferenceIsWeak = false;
}

//------------------------------------------------------------------------------
vtkInformationIterator::~vtkInformationIterator()
{
  if (this->ReferenceIsWeak)
  {
    this->Information = nullptr;
  }
  if (this->Information)
  {
    this->Information->Delete();
  }
  delete this->Internal;
}

//------------------------------------------------------------------------------
void vtkInformationIterator::SetInformation(vtkInformation* inf)
{
  if (this->ReferenceIsWeak)
  {
    this->Information = nullptr;
  }
  this->ReferenceIsWeak = false;
  vtkSetObjectBodyMacro(Information, vtkInformation, inf);
}

//------------------------------------------------------------------------------
void vtkInformationIterator::SetInformationWeak(vtkInformation* inf)
{
  if (!this->ReferenceIsWeak)
  {
    this->SetInformation(nullptr);
  }

  this->ReferenceIsWeak = true;

  if (this->Information != inf)
  {
    this->Information = inf;
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkInformationIterator::GoToFirstItem()
{
  if (!this->Information)
  {
    vtkErrorMacro("No information has been set.");
    return;
  }
  this->Internal->Iterator = this->Information->Internal->Map.begin();
}

//------------------------------------------------------------------------------
void vtkInformationIterator::GoToNextItem()
{
  if (!this->Information)
  {
    vtkErrorMacro("No information has been set.");
    return;
  }

  ++this->Internal->Iterator;
}

//------------------------------------------------------------------------------
int vtkInformationIterator::IsDoneWithTraversal()
{
  if (!this->Information)
  {
    vtkErrorMacro("No information has been set.");
    return 1;
  }

  if (this->Internal->Iterator == this->Information->Internal->Map.end())
  {
    return 1;
  }
  return 0;
}

//------------------------------------------------------------------------------
vtkInformationKey* vtkInformationIterator::GetCurrentKey()
{
  if (this->IsDoneWithTraversal())
  {
    return nullptr;
  }

  return this->Internal->Iterator->first;
}

//------------------------------------------------------------------------------
void vtkInformationIterator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Information: ";
  if (this->Information)
  {
    os << endl;
    this->Information->PrintSelf(os, indent.GetNextIndent());
  }
  else
  {
    os << "(none)" << endl;
  }
}
