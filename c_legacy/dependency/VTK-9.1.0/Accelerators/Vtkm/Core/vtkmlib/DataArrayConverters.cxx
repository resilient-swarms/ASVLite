//=============================================================================
//
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//
//  Copyright 2012 Sandia Corporation.
//  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
//  the U.S. Government retains certain rights in this software.
//
//=============================================================================
#include "DataArrayConverters.hxx"

#include "vtkmDataArray.h"
#include "vtkmFilterPolicy.h"

#include "vtkmlib/PortalTraits.h"

#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/DataSet.h>

#include "vtkDataArray.h"
#include "vtkPoints.h"

namespace fromvtkm
{

namespace
{

struct ArrayConverter
{
public:
  mutable vtkDataArray* Data;

  ArrayConverter()
    : Data(nullptr)
  {
  }

  // CastAndCall always passes a const array handle. Just shallow copy to a
  // local array handle by taking by value.

  template <typename T, typename S>
  void operator()(vtkm::cont::ArrayHandle<T, S> handle) const
  {
    this->Data = make_vtkmDataArray(handle);
  }

  template <typename T>
  void operator()(vtkm::cont::ArrayHandle<T, vtkm::cont::StorageTagBasic> handle) const
  {
    // we can steal this array!
    using Traits = tovtkm::vtkPortalTraits<T>; // Handles Vec<Vec<T,N,N> properly
    using ValueType = typename Traits::ComponentType;
    using VTKArrayType = vtkAOSDataArrayTemplate<ValueType>;

    if (handle.GetNumberOfBuffers() == 0)
    {
      return;
    }

    VTKArrayType* array = VTKArrayType::New();
    array->SetNumberOfComponents(Traits::NUM_COMPONENTS);

    handle.SyncControlArray();
    const vtkm::Id size = handle.GetNumberOfValues() * Traits::NUM_COMPONENTS;
    auto bufferInfo = handle.GetBuffers()[0].GetHostBufferInfo();

    vtkm::cont::internal::TransferredBuffer transfer = bufferInfo.TransferOwnership();
    auto srcMemory = reinterpret_cast<ValueType*>(transfer.Memory);
    if (transfer.Memory == transfer.Container)
    { // transfer the memory ownership over to VTK instead of copy
      array->SetVoidArray(srcMemory, size, 0, vtkAbstractArray::VTK_DATA_ARRAY_USER_DEFINED);
      array->SetArrayFreeFunction(transfer.Delete);
    }
    else
    {
      // deep copy the memory to VTK as the memory coming from
      // a source that VTK can't represent
      ValueType* dataBuffer = new ValueType[size];
      std::copy(srcMemory, srcMemory + size, dataBuffer);

      array->SetVoidArray(dataBuffer, size, 0, vtkAbstractArray::VTK_DATA_ARRAY_DELETE);
      transfer.Delete(transfer.Container);
    }

    this->Data = array;
  }

  template <typename T>
  void operator()(vtkm::cont::ArrayHandle<T, vtkm::cont::StorageTagSOA> handle) const
  {
    // we can steal this array!
    using Traits = tovtkm::vtkPortalTraits<T>; // Handles Vec<Vec<T,N,N> properly
    using ValueType = typename Traits::ComponentType;
    using VTKArrayType = vtkSOADataArrayTemplate<ValueType>;

    if (handle.GetNumberOfBuffers() != Traits::NUM_COMPONENTS)
    {
      return;
    }

    VTKArrayType* array = VTKArrayType::New();
    array->SetNumberOfComponents(Traits::NUM_COMPONENTS);

    handle.SyncControlArray();
    auto buffers = handle.GetBuffers();
    const vtkm::Id size = handle.GetNumberOfValues();

    for (vtkm::IdComponent i = 0; i < Traits::NUM_COMPONENTS; ++i)
    {
      auto bufferInfo = buffers[i].GetHostBufferInfo();

      vtkm::cont::internal::TransferredBuffer transfer = bufferInfo.TransferOwnership();
      auto srcMemory = reinterpret_cast<ValueType*>(transfer.Memory);
      if (transfer.Memory == transfer.Container)
      { // transfer the memory ownership over to VTK instead of copy
        array->SetArray(i, srcMemory, size, true, 0, vtkAbstractArray::VTK_DATA_ARRAY_USER_DEFINED);
        array->SetArrayFreeFunction(i, transfer.Delete);
      }
      else
      {
        // deep copy the memory to VTK as the memory coming from
        // a source that VTK can't represent
        ValueType* dataBuffer = new ValueType[size];
        std::copy(srcMemory, srcMemory + size, dataBuffer);

        array->SetArray(i, dataBuffer, size, true, 0, vtkAbstractArray::VTK_DATA_ARRAY_DELETE);
        transfer.Delete(transfer.Container);
      }
    }

    this->Data = array;
  }
};
} // anonymous namespace

// Though the following conversion routines take const-ref parameters as input,
// the underlying storage will be stolen, whenever possible, instead of
// performing a full copy.
// Therefore, these routines should be treated as "moves" and the state of the
// input is undeterminisitic.

vtkDataArray* Convert(const vtkm::cont::Field& input)
{
  // We need to do the conversion from Field to a known vtkm::cont::ArrayHandle
  // after that we need to fill the vtkDataArray
  vtkmOutputFilterPolicy policy;
  vtkDataArray* data = nullptr;
  ArrayConverter aConverter;

  try
  {
    vtkm::cont::CastAndCall(vtkm::filter::ApplyPolicyFieldNotActive(input, policy), aConverter);
    data = aConverter.Data;
    if (data)
    {
      data->SetName(input.GetName().c_str());
    }
  }
  catch (vtkm::cont::Error&)
  {
  }
  return data;
}

vtkDataArray* Convert(const vtkm::cont::UnknownArrayHandle& input, const char* name)
{
  // We need to do the conversion from UnknownArrayHandle to a known vtkm::cont::ArrayHandle
  // after that we need to fill the vtkDataArray
  vtkDataArray* data = nullptr;
  ArrayConverter aConverter;

  try
  {
    vtkm::cont::CastAndCall(input, aConverter);
    data = aConverter.Data;
    if (data && name)
    {
      data->SetName(name);
    }
  }
  catch (vtkm::cont::Error&)
  {
  }
  return data;
}

vtkPoints* Convert(const vtkm::cont::CoordinateSystem& input)
{
  ArrayConverter aConverter;
  vtkPoints* points = nullptr;
  try
  {
    vtkm::cont::CastAndCall(input, aConverter);
    vtkDataArray* pdata = aConverter.Data;
    points = vtkPoints::New();
    points->SetData(pdata);
    pdata->FastDelete();
  }
  catch (vtkm::cont::Error& e)
  {
    vtkGenericWarningMacro("Converting vtkm::cont::CoordinateSystem to "
                           "vtkPoints failed: "
      << e.what());
  }
  return points;
}

}
