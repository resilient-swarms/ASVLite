//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_cont_arg_TransportTagArrayIn_h
#define vtk_m_cont_arg_TransportTagArrayIn_h

#include <vtkm/Types.h>

#include <vtkm/cont/ArrayHandle.h>

#include <vtkm/cont/arg/Transport.h>

namespace vtkm
{
namespace cont
{
namespace arg
{

/// \brief \c Transport tag for input arrays.
///
/// \c TransportTagArrayIn is a tag used with the \c Transport class to
/// transport \c ArrayHandle objects for input data.
///
struct TransportTagArrayIn
{
};

template <typename ContObjectType, typename Device>
struct Transport<vtkm::cont::arg::TransportTagArrayIn, ContObjectType, Device>
{
  // MSVC will issue deprecation warnings here if this template is instantiated with
  // a deprecated class even if the template is used from a section of code where
  // deprecation warnings are suppressed. This is annoying behavior since this template
  // has no control over what class it is used with. To get around it, we have to
  // suppress all deprecation warnings here.
#ifdef VTKM_MSVC
  VTKM_DEPRECATED_SUPPRESS_BEGIN
#endif

  VTKM_IS_ARRAY_HANDLE(ContObjectType);

  using ExecObjectType = decltype(
    std::declval<ContObjectType>().PrepareForInput(Device(), std::declval<vtkm::cont::Token&>()));

  template <typename InputDomainType>
  VTKM_CONT ExecObjectType operator()(const ContObjectType& object,
                                      const InputDomainType& vtkmNotUsed(inputDomain),
                                      vtkm::Id inputRange,
                                      vtkm::Id vtkmNotUsed(outputRange),
                                      vtkm::cont::Token& token) const
  {
    if (object.GetNumberOfValues() != inputRange)
    {
      throw vtkm::cont::ErrorBadValue("Input array to worklet invocation the wrong size.");
    }

    return object.PrepareForInput(Device(), token);
  }

#ifdef VTKM_MSVC
  VTKM_DEPRECATED_SUPPRESS_END
#endif
};
}
}
} // namespace vtkm::cont::arg

#endif //vtk_m_cont_arg_TransportTagArrayIn_h
