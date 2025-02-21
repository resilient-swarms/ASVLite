/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkXMLDataElement.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkXMLDataElement.h"

#include "vtkObjectFactory.h"

#include <cctype>
#include <sstream>
using std::istringstream;
using std::ostringstream;
#include <string>
#include <vtksys/FStream.hxx>
#include <vtksys/SystemTools.hxx>
using std::string;
#include <locale> // C++ locale

vtkStandardNewMacro(vtkXMLDataElement);

//------------------------------------------------------------------------------
vtkXMLDataElement::vtkXMLDataElement()
{
  this->Name = nullptr;
  this->Id = nullptr;
  this->Parent = nullptr;

  this->NumberOfAttributes = 0;
  this->AttributesSize = 5;
  this->AttributeNames = new char*[this->AttributesSize];
  this->AttributeValues = new char*[this->AttributesSize];

  this->NumberOfNestedElements = 0;
  this->NestedElementsSize = 10;
  this->NestedElements = new vtkXMLDataElement*[this->NestedElementsSize];

  this->InlineDataPosition = 0;
  this->XMLByteIndex = 0;
  this->AttributeEncoding = VTK_ENCODING_UTF_8;

  this->CharacterDataWidth = -1;

  this->CharacterDataBlockSize = 2048;
  this->CharacterDataBufferSize = 2048;
  this->EndOfCharacterData = 1;
  this->CharacterData = static_cast<char*>(malloc(this->CharacterDataBlockSize));
  this->CharacterData[0] = '\0';

  this->IgnoreCharacterData = 0;
}

//------------------------------------------------------------------------------
vtkXMLDataElement::~vtkXMLDataElement()
{
  this->SetName(nullptr);
  this->SetId(nullptr);

  this->RemoveAllAttributes();
  delete[] this->AttributeNames;
  delete[] this->AttributeValues;

  this->RemoveAllNestedElements();
  delete[] this->NestedElements;

  free(this->CharacterData);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::RemoveAttribute(const char* name)
{
  if (!name || !name[0])
  {
    return;
  }

  // Find the attribute

  int i, j;
  for (i = 0; i < this->NumberOfAttributes; ++i)
  {
    if (!strcmp(this->AttributeNames[i], name))
    {
      // delete the contents for the attribute being removed.
      delete[] this->AttributeNames[i];
      delete[] this->AttributeValues[i];
      this->AttributeValues[i] = this->AttributeNames[i] = nullptr;

      // Shift the other attributes
      for (j = i; j < this->NumberOfAttributes - 1; ++j)
      {
        this->AttributeNames[j] = this->AttributeNames[j + 1];
        this->AttributeValues[j] = this->AttributeValues[j + 1];
      }

      // set the last ones as nullptr, since the pointers have been moved.
      this->AttributeNames[this->NumberOfAttributes - 1] = nullptr;
      this->AttributeValues[this->NumberOfAttributes - 1] = nullptr;

      --this->NumberOfAttributes;
      // this->AttributesSize is unchanged
      return;
    }
  }
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::RemoveAllAttributes()
{
  for (int i = 0; i < this->NumberOfAttributes; ++i)
  {
    delete[] this->AttributeNames[i];
    delete[] this->AttributeValues[i];
  }
  this->NumberOfAttributes = 0;
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::RemoveAllNestedElements()
{
  for (int i = 0; i < this->NumberOfNestedElements; ++i)
  {
    this->NestedElements[i]->UnRegister(this);
  }
  this->NumberOfNestedElements = 0;
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetName(const char* _arg)
{
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting Name to "
                << (_arg ? _arg : "(null)"));

  if (this->Name == nullptr && _arg == nullptr)
  {
    return;
  }
  if (this->Name && _arg && (!strcmp(this->Name, _arg)))
  {
    return;
  }
  delete[] this->Name;
  this->IgnoreCharacterData = 0;
  if (_arg)
  {
    // NOTE: Tags that have specialized character data
    // handlers can set this flag to improve performance.
    if (strstr(_arg, "DataArray"))
    {
      this->IgnoreCharacterData = 1;
    }
    size_t n = strlen(_arg) + 1;
    char* cp1 = new char[n];
    const char* cp2 = (_arg);
    this->Name = cp1;
    do
    {
      *cp1++ = *cp2++;
    } while (--n);
  }
  else
  {
    this->Name = nullptr;
  }
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetCharacterData(const char* data, int length)
{
  // Sanity check.
  if (length < 0)
  {
    vtkWarningMacro("Negative values for length are not allowed, setting to 0!");
    length = 0;
  }
  // Mark end of buffer.
  this->EndOfCharacterData = length + 1;
  // Size buffer in units of blocks.
  this->CharacterDataBufferSize = this->CharacterDataBlockSize;
  while (this->CharacterDataBufferSize < this->EndOfCharacterData)
  {
    this->CharacterDataBufferSize += this->CharacterDataBlockSize;
  }
  // Allocate the buffer.
  this->CharacterData =
    static_cast<char*>(realloc(this->CharacterData, this->CharacterDataBufferSize));
  // Copy the data passed in.
  if (data && length > 0)
  {
    memmove(this->CharacterData, data, length);
  }
  this->CharacterData[length] = 0;
  // Mark us changed.
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetAttribute(const char* name, const char* value)
{
  if (!name || !name[0] || !value || !value[0])
  {
    return;
  }

  int i;

  // Set an existing attribute

  for (i = 0; i < this->NumberOfAttributes; ++i)
  {
    if (!strcmp(this->AttributeNames[i], name))
    {
      delete[] this->AttributeValues[i];
      this->AttributeValues[i] = new char[strlen(value) + 1];
      strcpy(this->AttributeValues[i], value);
      return;
    }
  }

  // Or add an attribute

  if (this->NumberOfAttributes == this->AttributesSize)
  {
    int newSize = this->AttributesSize * 2;
    char** newAttributeNames = new char*[newSize];
    char** newAttributeValues = new char*[newSize];
    for (i = 0; i < this->NumberOfAttributes; ++i)
    {
      newAttributeNames[i] = new char[strlen(this->AttributeNames[i]) + 1];
      strcpy(newAttributeNames[i], this->AttributeNames[i]);
      delete[] this->AttributeNames[i];
      newAttributeValues[i] = new char[strlen(this->AttributeValues[i]) + 1];
      strcpy(newAttributeValues[i], this->AttributeValues[i]);
      delete[] this->AttributeValues[i];
    }
    delete[] this->AttributeNames;
    delete[] this->AttributeValues;
    this->AttributeNames = newAttributeNames;
    this->AttributeValues = newAttributeValues;
    this->AttributesSize = newSize;
  }

  i = this->NumberOfAttributes++;
  this->AttributeNames[i] = new char[strlen(name) + 1];
  strcpy(this->AttributeNames[i], name);
  this->AttributeValues[i] = new char[strlen(value) + 1];
  strcpy(this->AttributeValues[i], value);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::AddNestedElement(vtkXMLDataElement* element)
{
  if (!element)
  {
    return;
  }

  if (this->NumberOfNestedElements == this->NestedElementsSize)
  {
    int i;
    int newSize = this->NestedElementsSize * 2;
    vtkXMLDataElement** newNestedElements = new vtkXMLDataElement*[newSize];
    for (i = 0; i < this->NumberOfNestedElements; ++i)
    {
      newNestedElements[i] = this->NestedElements[i];
    }
    delete[] this->NestedElements;
    this->NestedElements = newNestedElements;
    this->NestedElementsSize = newSize;
  }

  int index = this->NumberOfNestedElements++;
  this->NestedElements[index] = element;
  element->Register(this);
  element->SetParent(this);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::RemoveNestedElement(vtkXMLDataElement* element)
{
  if (!element)
  {
    return;
  }

  int i, j;
  for (i = 0; i < this->NumberOfNestedElements; ++i)
  {
    if (this->NestedElements[i] == element)
    {
      for (j = i; j < this->NumberOfNestedElements - 1; ++j)
      {
        this->NestedElements[j] = this->NestedElements[j + 1];
      }
      element->UnRegister(this);
      this->NumberOfNestedElements--;
    }
  }
}

//------------------------------------------------------------------------------
const char* vtkXMLDataElement::GetAttribute(const char* name)
{
  if (!name)
  {
    return nullptr;
  }

  int i;
  for (i = 0; i < this->NumberOfAttributes; ++i)
  {
    if (strcmp(this->AttributeNames[i], name) == 0)
    {
      return this->AttributeValues[i];
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
const char* vtkXMLDataElement::GetAttributeName(int idx)
{
  if (idx < 0 || idx >= this->NumberOfAttributes)
  {
    return nullptr;
  }

  return this->AttributeNames[idx];
}

//------------------------------------------------------------------------------
const char* vtkXMLDataElement::GetAttributeValue(int idx)
{
  if (idx < 0 || idx >= this->NumberOfAttributes)
  {
    return nullptr;
  }

  return this->AttributeValues[idx];
}

//------------------------------------------------------------------------------
// Limits the width of a stream of character data,
// by inserting new lines and indenting properly.
void vtkXMLDataElement::PrintCharacterData(ostream& os, vtkIndent indent)
{
  // anything to do?
  if (this->CharacterData == nullptr || strcmp(this->CharacterData, "") == 0)
  {
    return;
  }
  // No special format just dump what we have.
  if (this->CharacterDataWidth < 1)
  {
    os << indent;
    vtkXMLDataElement::PrintWithEscapedData(os, this->CharacterData);
    os << endl;
  }
  // Treat as space/line delimited fields limiting
  // the number of field per line.
  else
  {
    istringstream issCharacterData(this->CharacterData);

    string characterDataToken;
    issCharacterData >> characterDataToken;
    os << indent;
    vtkXMLDataElement::PrintWithEscapedData(os, characterDataToken.c_str());

    int it = 0;
    while (issCharacterData.good())
    {
      if ((it % this->CharacterDataWidth) == (this->CharacterDataWidth - 1))
      {
        os << endl << indent;
      }
      else
      {
        os << " ";
      }

      issCharacterData >> characterDataToken;
      vtkXMLDataElement::PrintWithEscapedData(os, characterDataToken.c_str());
      ++it;
    }

    os << endl;
  }
}

//------------------------------------------------------------------------------
// print out data while replacing XML special characters <, >, &, ", ' with
// &lt;, &gt;, &amp;, &quot;, &apos;, respectively.
void vtkXMLDataElement::PrintWithEscapedData(ostream& os, const char* data)
{
  for (size_t i = 0; data[i] != 0; i++)
  {
    switch (data[i])
    {
      case '&':
      {
        os << "&amp;";
        break;
      }
      case '<':
      {
        os << "&lt;";
        break;
      }
      case '>':
      {
        os << "&gt;";
        break;
      }
      case '"':
      {
        os << "&quot;";
        break;
      }
      case '\'':
      {
        os << "&apos;";
        break;
      }
      default:
      {
        os << data[i];
      }
    }
  }
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::PrintXML(const char* fname)
{
  vtksys::ofstream of(fname);
  of.imbue(std::locale::classic());
  this->PrintXML(of, vtkIndent());
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::PrintXML(ostream& os, vtkIndent indent)
{
  vtkIndent nextIndent = indent.GetNextIndent();

  os << indent << "<" << this->Name;
  int i;
  for (i = 0; i < this->NumberOfAttributes; ++i)
  {
    os << " " << this->AttributeNames[i] << "=\"";
    this->PrintWithEscapedData(os, this->AttributeValues[i]);
    os << "\"";
  }
  // Long format tag is needed if either or both
  // nested elements or inline data are present.
  if (this->NumberOfNestedElements > 0 ||
    (this->CharacterData != nullptr && this->CharacterData[0] != 0))
  {
    os << ">\n";
    // nested elements
    for (i = 0; i < this->NumberOfNestedElements; ++i)
    {
      this->NestedElements[i]->PrintXML(os, nextIndent);
    }
    // inline data
    this->PrintCharacterData(os, nextIndent);
    // close tag
    os << indent << "</" << this->Name << ">\n";
  }
  // We can get away with short format tag.
  else
  {
    os << "/>\n";
  }
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetParent(vtkXMLDataElement* parent)
{
  this->Parent = parent;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::GetParent()
{
  return this->Parent;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::GetRoot()
{
  if (!this->Parent)
  {
    return this;
  }
  return this->Parent->GetRoot();
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetNumberOfNestedElements()
{
  return this->NumberOfNestedElements;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::GetNestedElement(int index)
{
  if (index >= 0 && index < this->NumberOfNestedElements)
  {
    return this->NestedElements[index];
  }
  return nullptr;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::FindNestedElementWithName(const char* name)
{
  if (!name)
  {
    return nullptr;
  }

  int i;
  for (i = 0; i < this->NumberOfNestedElements; ++i)
  {
    const char* nname = this->NestedElements[i]->GetName();
    if (nname && (strcmp(nname, name) == 0))
    {
      return this->NestedElements[i];
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::FindNestedElementWithNameAndId(
  const char* name, const char* id)
{
  if (!name || !id)
  {
    return nullptr;
  }

  int i;
  for (i = 0; i < this->NumberOfNestedElements; ++i)
  {
    const char* nname = this->NestedElements[i]->GetName();
    const char* nid = this->NestedElements[i]->GetId();
    if (nname && nid && (strcmp(nname, name) == 0) && (strcmp(nid, id) == 0))
    {
      return this->NestedElements[i];
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::FindNestedElementWithNameAndAttribute(
  const char* name, const char* att_name, const char* att_value)
{
  if (!name || !att_name || !att_value)
  {
    return nullptr;
  }

  int i;
  for (i = 0; i < this->NumberOfNestedElements; ++i)
  {
    const char* nname = this->NestedElements[i]->GetName();
    if (nname && (strcmp(nname, name) == 0))
    {
      const char* val = this->NestedElements[i]->GetAttribute(att_name);
      if (val && !strcmp(val, att_value))
      {
        return this->NestedElements[i];
      }
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::LookupElement(const char* id)
{
  return this->LookupElementUpScope(id);
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::FindNestedElement(const char* id)
{
  if (!id)
  {
    return nullptr;
  }

  int i;
  for (i = 0; i < this->NumberOfNestedElements; ++i)
  {
    const char* nid = this->NestedElements[i]->GetId();
    if (nid && (strcmp(nid, id) == 0))
    {
      return this->NestedElements[i];
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::LookupElementInScope(const char* id)
{
  if (!id)
  {
    return nullptr;
  }

  // Pull off the first qualifier.
  const char* end = id;
  while (*end && (*end != '.'))
    ++end;
  int len = end - id;
  char* name = new char[len + 1];
  strncpy(name, id, len);
  name[len] = '\0';

  // Find the qualifier in this scope.
  vtkXMLDataElement* next = this->FindNestedElement(name);
  if (next && (*end == '.'))
  {
    // Lookup rest of qualifiers in nested scope.
    next = next->LookupElementInScope(end + 1);
  }

  delete[] name;
  return next;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::LookupElementUpScope(const char* id)
{
  if (!id)
  {
    return nullptr;
  }

  // Pull off the first qualifier.
  const char* end = id;
  while (*end && (*end != '.'))
    ++end;
  int len = end - id;
  char* name = new char[len + 1];
  strncpy(name, id, len);
  name[len] = '\0';

  // Find most closely nested occurrence of first qualifier.
  vtkXMLDataElement* curScope = this;
  vtkXMLDataElement* start = nullptr;
  while (curScope && !start)
  {
    start = curScope->FindNestedElement(name);
    curScope = curScope->GetParent();
  }
  if (start && (*end == '.'))
  {
    start = start->LookupElementInScope(end + 1);
  }

  delete[] name;
  return start;
}

//------------------------------------------------------------------------------
vtkXMLDataElement* vtkXMLDataElement::LookupElementWithName(const char* name)
{
  if (!name)
  {
    return nullptr;
  }

  int i;
  for (i = 0; i < this->NumberOfNestedElements; ++i)
  {
    const char* nname = this->NestedElements[i]->GetName();
    if (nname && !strcmp(nname, name))
    {
      return this->NestedElements[i];
    }
    vtkXMLDataElement* found = this->NestedElements[i]->LookupElementWithName(name);
    if (found)
    {
      return found;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetScalarAttribute(const char* name, int& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetScalarAttribute(const char* name, float& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetScalarAttribute(const char* name, double& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetScalarAttribute(const char* name, long& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetScalarAttribute(const char* name, unsigned long& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetScalarAttribute(const char* name, long long& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}
int vtkXMLDataElement::GetScalarAttribute(const char* name, unsigned long long& value)
{
  return this->GetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
template <class T>
int vtkXMLDataElementVectorAttributeParse(const char* str, int length, T* data)
{
  if (!str || !length)
  {
    return 0;
  }
  std::stringstream vstr;
  vstr.imbue(std::locale::classic());
  vstr << str;
  int i;
  for (i = 0; i < length; ++i)
  {
    T value;
    vstr >> value;
    if (data)
    {
      data[i] = value;
    }
    if (!vstr)
    {
      return i;
    }
  }
  return length;
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, int* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, float* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, double* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, long* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, unsigned long* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, long long* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}
int vtkXMLDataElement::GetVectorAttribute(const char* name, int length, unsigned long long* data)
{
  return vtkXMLDataElementVectorAttributeParse(this->GetAttribute(name), length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::GetWordTypeAttribute(const char* name, int& value)
{
  // These string values must match vtkXMLWriter::GetWordTypeName().
  const char* v = this->GetAttribute(name);
  if (!v)
  {
    vtkErrorMacro("Missing word type attribute \"" << name << "\".");
    return 0;
  }
  else if (strcmp(v, "Float32") == 0)
  {
#ifdef VTK_TYPE_FLOAT32
    value = VTK_TYPE_FLOAT32;
    return 1;
#else
    vtkErrorMacro("Float32 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "Float64") == 0)
  {
#ifdef VTK_TYPE_FLOAT64
    value = VTK_TYPE_FLOAT64;
    return 1;
#else
    vtkErrorMacro("Float64 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "Int8") == 0)
  {
#ifdef VTK_TYPE_INT8
    value = VTK_TYPE_INT8;
    return 1;
#else
    vtkErrorMacro("Int8 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "UInt8") == 0)
  {
#ifdef VTK_TYPE_UINT8
    value = VTK_TYPE_UINT8;
    return 1;
#else
    vtkErrorMacro("UInt8 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "Int16") == 0)
  {
#ifdef VTK_TYPE_INT16
    value = VTK_TYPE_INT16;
    return 1;
#else
    vtkErrorMacro("Int16 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "UInt16") == 0)
  {
#ifdef VTK_TYPE_UINT16
    value = VTK_TYPE_UINT16;
    return 1;
#else
    vtkErrorMacro("UInt16 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "Int32") == 0)
  {
#ifdef VTK_TYPE_INT32
    value = VTK_TYPE_INT32;
    return 1;
#else
    vtkErrorMacro("Int32 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "UInt32") == 0)
  {
#ifdef VTK_TYPE_UINT32
    value = VTK_TYPE_UINT32;
    return 1;
#else
    vtkErrorMacro("UInt32 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "Int64") == 0)
  {
#ifdef VTK_TYPE_INT64
    value = VTK_TYPE_INT64;
    return 1;
#else
    vtkErrorMacro("Int64 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "UInt64") == 0)
  {
#ifdef VTK_TYPE_UINT64
    value = VTK_TYPE_UINT64;
    return 1;
#else
    vtkErrorMacro("UInt64 support not compiled in VTK.");
    return 0;
#endif
  }
  else if (strcmp(v, "String") == 0)
  {
    value = VTK_STRING;
    return 1;
  }
  else if (strcmp(v, "Bit") == 0)
  {
    value = VTK_BIT;
    return 1;
  }
  else
  {
    vtkErrorMacro("Unknown data type \"" << v
                                         << "\".  Supported types are:\n"
                                            "Int8,  Int16,  Int32,  Int64,\n"
                                            "UInt8, UInt16, UInt32, UInt64,\n"
                                            "Float32, Float64, String, Bit\n");
    return 0;
  }
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetIntAttribute(const char* name, int value)
{
  this->SetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetFloatAttribute(const char* name, float value)
{
  this->SetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetDoubleAttribute(const char* name, double value)
{
  this->SetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetUnsignedLongAttribute(const char* name, unsigned long value)
{
  this->SetVectorAttribute(name, 1, &value);
}

//------------------------------------------------------------------------------
template <class T>
void vtkXMLDataElementVectorAttributeSet(
  vtkXMLDataElement* elem, const char* name, int length, const T* data)
{
  if (!elem || !name || !length)
  {
    return;
  }
  std::stringstream vstr;
  vstr.imbue(std::locale::classic());
  vstr << data[0];
  for (int i = 1; i < length; ++i)
  {
    vstr << ' ' << data[i];
  }

  elem->SetAttribute(name, vstr.str().c_str());
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetVectorAttribute(const char* name, int length, const int* data)
{
  vtkXMLDataElementVectorAttributeSet(this, name, length, data);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetVectorAttribute(const char* name, int length, const float* data)
{
  vtkXMLDataElementVectorAttributeSet(this, name, length, data);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetVectorAttribute(const char* name, int length, const double* data)
{
  vtkXMLDataElementVectorAttributeSet(this, name, length, data);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetVectorAttribute(const char* name, int length, const unsigned long* data)
{
  vtkXMLDataElementVectorAttributeSet(this, name, length, data);
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::SetVectorAttribute(const char* name, int length, long long const* data)
{
  vtkXMLDataElementVectorAttributeSet(this, name, length, data);
}
void vtkXMLDataElement::SetVectorAttribute(
  const char* name, int length, unsigned long long const* data)
{
  vtkXMLDataElementVectorAttributeSet(this, name, length, data);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::IsSpace(char c)
{
  return isspace(c);
}

//------------------------------------------------------------------------------
int vtkXMLDataElement::IsEqualTo(vtkXMLDataElement* elem)
{
  if (this == elem)
  {
    return 1;
  }

  if (!elem)
  {
    return 0;
  }

  if (this->GetNumberOfAttributes() != elem->GetNumberOfAttributes() ||
    this->GetNumberOfNestedElements() != elem->GetNumberOfNestedElements() ||
    (this->GetName() != elem->GetName() &&
      (!this->GetName() || !elem->GetName() || strcmp(this->GetName(), elem->GetName()) != 0)) ||
    (this->GetCharacterData() != elem->GetCharacterData() &&
      (!this->GetCharacterData() || !elem->GetCharacterData() ||
        strcmp(this->GetCharacterData(), elem->GetCharacterData()) != 0)))
  {
    return 0;
  }

  // Compare attributes

  int i;
  for (i = 0; i < this->GetNumberOfAttributes(); ++i)
  {
    const char* value = elem->GetAttribute(this->AttributeNames[i]);
    if (!value || strcmp(value, this->AttributeValues[i]) != 0)
    {
      return 0;
    }
  }

  // Compare nested elements

  for (i = 0; i < this->GetNumberOfNestedElements(); ++i)
  {
    if (!this->GetNestedElement(i)->IsEqualTo(elem->GetNestedElement(i)))
    {
      return 0;
    }
  }

  return 1;
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::DeepCopy(vtkXMLDataElement* elem)
{
  if (!elem)
  {
    return;
  }

  this->SetName(elem->GetName());
  this->SetId(elem->GetId());
  this->SetXMLByteIndex(elem->GetXMLByteIndex());
  this->SetAttributeEncoding(elem->GetAttributeEncoding()); ///
  const char* elem_cdata = elem->GetCharacterData();
  this->SetCharacterData(elem_cdata, elem_cdata ? static_cast<int>(strlen(elem_cdata)) : 0);
  this->SetCharacterDataWidth(elem->GetCharacterDataWidth());

  // Copy attributes

  this->RemoveAllAttributes();

  int i;
  for (i = 0; i < elem->GetNumberOfAttributes(); ++i)
  {
    const char* att_name = elem->GetAttributeName(i);
    this->SetAttribute(att_name, elem->GetAttribute(att_name));
  }

  // Copy nested elements

  this->RemoveAllNestedElements();

  for (i = 0; i < elem->GetNumberOfNestedElements(); ++i)
  {
    vtkXMLDataElement* nested_elem = vtkXMLDataElement::New();
    nested_elem->DeepCopy(elem->GetNestedElement(i));
    this->AddNestedElement(nested_elem);
    nested_elem->Delete();
  }
}

//------------------------------------------------------------------------------
void vtkXMLDataElement::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "XMLByteIndex: " << this->XMLByteIndex << "\n";
  os << indent << "Name: " << (this->Name ? this->Name : "(none)") << "\n";
  os << indent << "Id: " << (this->Id ? this->Id : "(none)") << "\n";
  os << indent << "NumberOfAttributes: " << this->NumberOfAttributes << "\n";
  os << indent << "AttributeEncoding: " << this->AttributeEncoding << "\n";
  os << indent << "CharacterData: " << (this->CharacterData ? this->CharacterData : "(null)")
     << endl;
  os << indent << "CharacterDataWidth: " << this->CharacterDataWidth << endl;
}
