//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_cont_DeviceAdapterList_h
#define vtk_m_cont_DeviceAdapterList_h

#ifndef VTKM_DEFAULT_DEVICE_ADAPTER_LIST
#define VTKM_DEFAULT_DEVICE_ADAPTER_LIST ::vtkm::cont::DeviceAdapterListCommon
#endif

#include <vtkm/List.h>

#include <vtkm/cont/cuda/internal/DeviceAdapterTagCuda.h>
#include <vtkm/cont/kokkos/internal/DeviceAdapterTagKokkos.h>
#include <vtkm/cont/openmp/internal/DeviceAdapterTagOpenMP.h>
#include <vtkm/cont/serial/internal/DeviceAdapterTagSerial.h>
#include <vtkm/cont/tbb/internal/DeviceAdapterTagTBB.h>

namespace vtkm
{
namespace cont
{

using DeviceAdapterListCommon = vtkm::List<vtkm::cont::DeviceAdapterTagCuda,
                                           vtkm::cont::DeviceAdapterTagTBB,
                                           vtkm::cont::DeviceAdapterTagOpenMP,
                                           vtkm::cont::DeviceAdapterTagKokkos,
                                           vtkm::cont::DeviceAdapterTagSerial>;
}
} // namespace vtkm::cont

#endif //vtk_m_cont_DeviceAdapterList_h
