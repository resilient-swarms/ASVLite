/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPeriodicTable.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

  =========================================================================*/

// Hide VTK_DEPRECATED_IN_9_1_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkPeriodicTable.h"

#include "vtkAbstractArray.h"
#include "vtkBlueObeliskData.h"
#include "vtkColor.h"
#include "vtkDebugLeaks.h"
#include "vtkFloatArray.h"
#include "vtkLookupTable.h"
#include "vtkObjectFactory.h"
#include "vtkStdString.h"
#include "vtkStringArray.h"
#include "vtkUnsignedShortArray.h"

#include <cassert>
#include <cctype>
#include <cstring>
#include <string>

// Setup static variables
vtkNew<vtkBlueObeliskData> vtkPeriodicTable::BlueObeliskData;

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPeriodicTable);

//------------------------------------------------------------------------------
vtkPeriodicTable::vtkPeriodicTable()
{
  vtkPeriodicTable::BlueObeliskData->LockWriteMutex();

  if (!vtkPeriodicTable::BlueObeliskData->IsInitialized())
  {
    vtkPeriodicTable::BlueObeliskData->Initialize();
  }

  vtkPeriodicTable::BlueObeliskData->UnlockWriteMutex();
}

//------------------------------------------------------------------------------
vtkPeriodicTable::~vtkPeriodicTable() = default;

//------------------------------------------------------------------------------
void vtkPeriodicTable::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "BlueObeliskData:\n";
  vtkPeriodicTable::BlueObeliskData->PrintSelf(os, indent.GetNextIndent());
}

//------------------------------------------------------------------------------
unsigned short vtkPeriodicTable::GetNumberOfElements()
{
  return vtkPeriodicTable::BlueObeliskData->GetNumberOfElements();
}

//------------------------------------------------------------------------------
const char* vtkPeriodicTable::GetSymbol(unsigned short atomicNum)
{
  if (atomicNum > this->GetNumberOfElements())
  {
    vtkWarningMacro("Atomic number out of range ! Using 0 instead of " << atomicNum);
    atomicNum = 0;
  }

  return vtkPeriodicTable::BlueObeliskData->GetSymbols()->GetValue(atomicNum).c_str();
}

//------------------------------------------------------------------------------
const char* vtkPeriodicTable::GetElementName(unsigned short atomicNum)
{
  if (atomicNum > this->GetNumberOfElements())
  {
    vtkWarningMacro("Atomic number out of range ! Using 0 instead of " << atomicNum);
    atomicNum = 0;
  }

  return vtkPeriodicTable::BlueObeliskData->GetNames()->GetValue(atomicNum).c_str();
}

//------------------------------------------------------------------------------
unsigned short vtkPeriodicTable::GetAtomicNumber(const vtkStdString& str)
{
  return this->GetAtomicNumber(str.c_str());
}

//------------------------------------------------------------------------------
unsigned short vtkPeriodicTable::GetAtomicNumber(const char* str)
{
  // If the string is null or the BODR object is not initialized, just
  // return 0.
  if (!str)
  {
    return 0;
  }

  // First attempt to just convert the string to an integer. If this
  // works, return the integer
  int atoi_num = atoi(str);
  if (atoi_num > 0 && atoi_num <= static_cast<int>(this->GetNumberOfElements()))
  {
    return static_cast<unsigned short>(atoi_num);
  }

  // Convert str to lowercase (see note about casts in
  // https://en.cppreference.com/w/cpp/string/byte/tolower)
  std::string lowerStr(str);
  std::transform(lowerStr.cbegin(), lowerStr.cend(), lowerStr.begin(),
    [](unsigned char c) -> char { return static_cast<char>(std::tolower(c)); });

  // Cache pointers:
  vtkStringArray* lnames = vtkPeriodicTable::BlueObeliskData->GetLowerNames();
  vtkStringArray* lsymbols = vtkPeriodicTable::BlueObeliskData->GetLowerSymbols();
  const unsigned short numElements = this->GetNumberOfElements();

  // Compare with other lowercase strings
  for (unsigned short ind = 0; ind <= numElements; ++ind)
  {
    if (lnames->GetValue(ind) == lowerStr || lsymbols->GetValue(ind) == lowerStr)
    {
      return ind;
    }
  }

  // Manually test some non-standard names:
  // - Deuterium
  if (lowerStr == "d" || lowerStr == "deuterium")
  {
    return 1;
  }
  // - Tritium
  else if (lowerStr == "t" || lowerStr == "tritium")
  {
    return 1;
  }
  // - Aluminum (vs. Aluminium)
  else if (lowerStr == "aluminum")
  {
    return 13;
  }

  return 0;
}

//------------------------------------------------------------------------------
float vtkPeriodicTable::GetCovalentRadius(unsigned short atomicNum)
{
  if (atomicNum > this->GetNumberOfElements())
  {
    vtkWarningMacro("Atomic number out of range ! Using 0 instead of " << atomicNum);
    atomicNum = 0;
  }

  return vtkPeriodicTable::BlueObeliskData->GetCovalentRadii()->GetValue(atomicNum);
}

//------------------------------------------------------------------------------
float vtkPeriodicTable::GetVDWRadius(unsigned short atomicNum)
{
  if (atomicNum > this->GetNumberOfElements())
  {
    vtkWarningMacro("Atomic number out of range ! Using 0 instead of " << atomicNum);
    atomicNum = 0;
  }

  return vtkPeriodicTable::BlueObeliskData->GetVDWRadii()->GetValue(atomicNum);
}

//------------------------------------------------------------------------------
float vtkPeriodicTable::GetMaxVDWRadius()
{
  float maxRadius = 0;
  for (unsigned short i = 0; i < this->GetNumberOfElements(); i++)
  {
    maxRadius = std::max(maxRadius, this->GetVDWRadius(i));
  }
  return maxRadius;
}

//------------------------------------------------------------------------------
void vtkPeriodicTable::GetDefaultLUT(vtkLookupTable* lut)
{
  const unsigned short numColors = this->GetNumberOfElements() + 1;
  vtkFloatArray* colors = vtkPeriodicTable::BlueObeliskData->GetDefaultColors();
  lut->SetNumberOfColors(numColors);
  lut->SetIndexedLookup(true);
  float rgb[3];
  for (vtkIdType i = 0; static_cast<unsigned int>(i) < numColors; ++i)
  {
    colors->GetTypedTuple(i, rgb);
    lut->SetTableValue(i, rgb[0], rgb[1], rgb[2]);
    lut->SetAnnotation(i, this->GetSymbol(static_cast<unsigned short>(i)));
  }
}

//------------------------------------------------------------------------------
void vtkPeriodicTable::GetDefaultRGBTuple(unsigned short atomicNum, float rgb[3])
{
  vtkPeriodicTable::BlueObeliskData->GetDefaultColors()->GetTypedTuple(atomicNum, rgb);
}

//------------------------------------------------------------------------------
vtkColor3f vtkPeriodicTable::GetDefaultRGBTuple(unsigned short atomicNum)
{
  vtkColor3f result;
  vtkPeriodicTable::BlueObeliskData->GetDefaultColors()->GetTypedTuple(atomicNum, result.GetData());
  return result;
}
