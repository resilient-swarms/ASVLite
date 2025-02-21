//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_worklet_internal_MaskBase_h
#define vtk_m_worklet_internal_MaskBase_h

#include <vtkm/internal/DecayHelpers.h>
#include <vtkm/internal/ExportMacros.h>

namespace vtkm
{
namespace worklet
{
namespace internal
{

/// Base class for all mask classes.
///
/// This allows VTK-m to determine when a parameter
/// is a mask type instead of a worklet parameter.
///
struct VTKM_ALWAYS_EXPORT MaskBase
{
};

template <typename T>
using is_mask = std::is_base_of<MaskBase, vtkm::internal::remove_cvref<T>>;
}
}
}
#endif
