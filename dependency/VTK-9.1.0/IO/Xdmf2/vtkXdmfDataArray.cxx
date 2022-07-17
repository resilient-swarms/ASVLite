/*******************************************************************/
/*                               XDMF                              */
/*                   eXtensible Data Model and Format              */
/*                                                                 */
/*  Id : Id  */
/*                                                                 */
/*  Author:                                                        */
/*     Jerry A. Clarke                                             */
/*     clarke@arl.army.mil                                         */
/*     US Army Research Laboratory                                 */
/*     Aberdeen Proving Ground, MD                                 */
/*                                                                 */
/*     Copyright @ 2002 US Army Research Laboratory                */
/*     All Rights Reserved                                         */
/*     See Copyright.txt or http://www.arl.hpc.mil/ice for details */
/*                                                                 */
/*     This software is distributed WITHOUT ANY WARRANTY; without  */
/*     even the implied warranty of MERCHANTABILITY or FITNESS     */
/*     FOR A PARTICULAR PURPOSE.  See the above copyright notice   */
/*     for more information.                                       */
/*                                                                 */
/*******************************************************************/
#include "vtkXdmfDataArray.h"

#include "vtkCommand.h"
#include "vtkObjectFactory.h"

#include "vtkCharArray.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"
#include "vtkIntArray.h"
#include "vtkLongArray.h"
#include "vtkShortArray.h"
#include "vtkUnsignedCharArray.h"
#include "vtkUnsignedIntArray.h"
#include "vtkUnsignedShortArray.h"

#include "vtk_xdmf2.h"
#include VTKXDMF2_HEADER(XdmfArray.h)

using namespace xdmf2;

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkXdmfDataArray);

//------------------------------------------------------------------------------
vtkXdmfDataArray::vtkXdmfDataArray()
{
  this->Array = nullptr;
  this->vtkArray = nullptr;
}

//------------------------------------------------------------------------------
vtkDataArray* vtkXdmfDataArray::FromXdmfArray(
  char* ArrayName, int CopyShape, int rank, int Components, int MakeCopy)
{
  xdmf2::XdmfArray* array = this->Array;
  XdmfInt64 components = 1;
  XdmfInt64 tuples = 0;
  if (ArrayName != nullptr)
  {
    array = TagNameToArray(ArrayName);
  }
  if (array == nullptr)
  {
    XdmfErrorMessage("Array is nullptr");
    return (nullptr);
  }
  if (this->vtkArray)
  {
    this->vtkArray->Delete();
    this->vtkArray = nullptr;
  }
  switch (array->GetNumberType())
  {
    case XDMF_INT8_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkCharArray::New();
      }
      break;
    case XDMF_UINT8_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkUnsignedCharArray::New();
      }
      break;
    case XDMF_INT16_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkShortArray::New();
      }
      break;
    case XDMF_UINT16_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkUnsignedShortArray::New();
      }
      break;
    case XDMF_UINT32_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkUnsignedIntArray::New();
      }
      break;
    case XDMF_INT32_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkIntArray::New();
      }
      break;
    case XDMF_INT64_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkLongArray::New();
      }
      break;
    case XDMF_FLOAT32_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkFloatArray::New();
      }
      break;
    case XDMF_FLOAT64_TYPE:
      if (this->vtkArray == nullptr)
      {
        this->vtkArray = vtkDoubleArray::New();
      }
      break;
    default:
      vtkErrorMacro("Cannot create VTK data array: " << array->GetNumberType());
      return nullptr;
  }
  if (CopyShape)
  {
    if (array->GetRank() > rank + 1)
    {
      this->vtkArray->Delete();
      this->vtkArray = nullptr;
      vtkErrorMacro("Rank of Xdmf array is more than 1 + rank of dataset");
      return nullptr;
    }
    if (array->GetRank() > rank)
    {
      components = array->GetDimension(rank);
    }
    tuples = array->GetNumberOfElements() / components;
    /// this breaks
    components = Components;
    tuples = array->GetNumberOfElements() / components;
    // cout << "Tuples: " << tuples << " components: " << components << endl;
    // cout << "Rank: " << rank << endl;
    this->vtkArray->SetNumberOfComponents(components);
    if (MakeCopy)
      this->vtkArray->SetNumberOfTuples(tuples);
  }
  else
  {
    this->vtkArray->SetNumberOfComponents(1);
    if (MakeCopy)
      this->vtkArray->SetNumberOfTuples(array->GetNumberOfElements());
  }
  // cout << "Number type: " << array->GetNumberType() << endl;
  if (MakeCopy)
  {
    switch (array->GetNumberType())
    {
      case XDMF_INT8_TYPE:
        array->GetValues(
          0, (XDMF_8_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_UINT8_TYPE:
        array->GetValues(
          0, (XDMF_8_U_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_INT16_TYPE:
        array->GetValues(
          0, (XDMF_16_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_UINT16_TYPE:
        array->GetValues(
          0, (XDMF_16_U_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_INT32_TYPE:
        array->GetValues(
          0, (XDMF_32_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_UINT32_TYPE:
        array->GetValues(
          0, (XDMF_32_U_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_INT64_TYPE:
        array->GetValues(
          0, (XDMF_64_INT*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_FLOAT32_TYPE:
        array->GetValues(
          0, (float*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      case XDMF_FLOAT64_TYPE:
        array->GetValues(
          0, (double*)this->vtkArray->GetVoidPointer(0), array->GetNumberOfElements());
        break;
      default:
        if (array->GetNumberOfElements() > 0)
        {
          // cout << "Manual idx" << endl;
          // cout << "Tuples: " << vtkArray->GetNumberOfTuples() << endl;
          // cout << "Components: " << vtkArray->GetNumberOfComponents() << endl;
          // cout << "Elements: " << array->GetNumberOfElements() << endl;
          vtkIdType jj, kk;
          vtkIdType idx = 0;
          for (jj = 0; jj < vtkArray->GetNumberOfTuples(); jj++)
          {
            for (kk = 0; kk < vtkArray->GetNumberOfComponents(); kk++)
            {
              double val = array->GetValueAsFloat64(idx);
              // cout << "Value: " << val << endl;
              vtkArray->SetComponent(jj, kk, val);
              idx++;
            }
          }
        }
        break;
    }
  }
  else
  {
    switch (array->GetNumberType())
    {
      case XDMF_INT8_TYPE:
      {
        vtkCharArray* chara = vtkArrayDownCast<vtkCharArray>(this->vtkArray);
        if (!chara)
        {
          XdmfErrorMessage("Cannot downcast data array");
          return (nullptr);
        }
        chara->SetArray((char*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_UINT8_TYPE:
      {
        vtkUnsignedCharArray* uchara = vtkArrayDownCast<vtkUnsignedCharArray>(this->vtkArray);
        if (!uchara)
        {
          XdmfErrorMessage("Cannot downcast ucharata array");
          return (nullptr);
        }
        uchara->SetArray((unsigned char*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_INT16_TYPE:
      {
        vtkShortArray* shorta = vtkArrayDownCast<vtkShortArray>(this->vtkArray);
        if (!shorta)
        {
          XdmfErrorMessage("Cannot downcast data array");
          return (nullptr);
        }
        shorta->SetArray((short*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_UINT16_TYPE:
      {
        vtkUnsignedShortArray* ushorta = vtkArrayDownCast<vtkUnsignedShortArray>(this->vtkArray);
        if (!ushorta)
        {
          XdmfErrorMessage("Cannot downcast ushortata array");
          return (nullptr);
        }
        ushorta->SetArray((unsigned short*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_INT32_TYPE:
      {
        vtkIntArray* inta = vtkArrayDownCast<vtkIntArray>(this->vtkArray);
        if (!inta)
        {
          XdmfErrorMessage("Cannot downcast intata array");
          return (nullptr);
        }
        inta->SetArray((int*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_UINT32_TYPE:
      {
        vtkUnsignedIntArray* uinta = vtkArrayDownCast<vtkUnsignedIntArray>(this->vtkArray);
        if (!uinta)
        {
          XdmfErrorMessage("Cannot downcast uintata array");
          return (nullptr);
        }
        uinta->SetArray((unsigned int*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_INT64_TYPE:
      {
        vtkLongArray* longa = vtkArrayDownCast<vtkLongArray>(this->vtkArray);
        if (!longa)
        {
          XdmfErrorMessage("Cannot downcast longa array");
          return (nullptr);
        }
        longa->SetArray((long*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_FLOAT32_TYPE:
      {
        vtkFloatArray* floata = vtkArrayDownCast<vtkFloatArray>(this->vtkArray);
        if (!floata)
        {
          XdmfErrorMessage("Cannot downcast floatata array");
          return (nullptr);
        }
        floata->SetArray((float*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      case XDMF_FLOAT64_TYPE:
      {
        vtkDoubleArray* doublea = vtkArrayDownCast<vtkDoubleArray>(this->vtkArray);
        if (!doublea)
        {
          XdmfErrorMessage("Cannot downcast doubleata array");
          return (nullptr);
        }
        doublea->SetArray((double*)array->GetDataPointer(), components * tuples, 0);
      }
      break;
      default:
        XdmfErrorMessage("Can't handle number type");
        return (nullptr);
    }
    array->Reset();
  }
  return (this->vtkArray);
}

//------------------------------------------------------------------------------
char* vtkXdmfDataArray::ToXdmfArray(vtkDataArray* DataArray, int CopyShape)
{
  xdmf2::XdmfArray* array;
  if (DataArray == nullptr)
  {
    DataArray = this->vtkArray;
  }
  if (DataArray == nullptr)
  {
    vtkDebugMacro(<< "Array is nullptr");
    return (nullptr);
  }
  if (this->Array == nullptr)
  {
    this->Array = new xdmf2::XdmfArray();
    switch (DataArray->GetDataType())
    {
      case VTK_CHAR:
      case VTK_UNSIGNED_CHAR:
        this->Array->SetNumberType(XDMF_INT8_TYPE);
        break;
      case VTK_SHORT:
      case VTK_UNSIGNED_SHORT:
      case VTK_INT:
      case VTK_UNSIGNED_INT:
      case VTK_LONG:
      case VTK_UNSIGNED_LONG:
        this->Array->SetNumberType(XDMF_INT32_TYPE);
        break;
      case VTK_FLOAT:
        this->Array->SetNumberType(XDMF_FLOAT32_TYPE);
        break;
      case VTK_DOUBLE:
        this->Array->SetNumberType(XDMF_FLOAT64_TYPE);
        break;
      default:
        XdmfErrorMessage("Can't handle Data Type");
        return (nullptr);
    }
  }
  array = this->Array;
  if (CopyShape)
  {
    XdmfInt64 Shape[3];

    Shape[0] = DataArray->GetNumberOfTuples();
    Shape[1] = DataArray->GetNumberOfComponents();
    if (Shape[1] == 1)
    {
      array->SetShape(1, Shape);
    }
    else
    {
      array->SetShape(2, Shape);
    }
  }
  switch (array->GetNumberType())
  {
    case XDMF_INT8_TYPE:
      array->SetValues(
        0, (unsigned char*)DataArray->GetVoidPointer(0), array->GetNumberOfElements());
      break;
    case XDMF_INT32_TYPE:
    case XDMF_INT64_TYPE:
      array->SetValues(0, (int*)DataArray->GetVoidPointer(0), array->GetNumberOfElements());
      break;
    case XDMF_FLOAT32_TYPE:
      array->SetValues(0, (float*)DataArray->GetVoidPointer(0), array->GetNumberOfElements());
      break;
    default:
      array->SetValues(0, (double*)DataArray->GetVoidPointer(0), array->GetNumberOfElements());
      break;
  }
  return (array->GetTagName());
}

//------------------------------------------------------------------------------
vtkDataArray* vtkXdmfDataArray::FromArray()
{
  return (this->FromXdmfArray());
}

//------------------------------------------------------------------------------
char* vtkXdmfDataArray::ToArray()
{
  return (this->ToXdmfArray());
}

//------------------------------------------------------------------------------
void vtkXdmfDataArray::SetArray(char* TagName)
{
  this->Array = TagNameToArray(TagName);
  if (this->Array)
  {
    this->FromXdmfArray();
  }
}

//------------------------------------------------------------------------------
char* vtkXdmfDataArray::GetArray()
{
  if (this->Array != nullptr)
  {
    return (this->Array->GetTagName());
  }
  return (nullptr);
}

//------------------------------------------------------------------------------
void vtkXdmfDataArray::SetVtkArray(vtkDataArray* array)
{
  this->vtkArray = array;
  this->ToXdmfArray(array);
}

//------------------------------------------------------------------------------
vtkDataArray* vtkXdmfDataArray::GetVtkArray()
{
  return (this->vtkArray);
}

//------------------------------------------------------------------------------
void vtkXdmfDataArray::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
