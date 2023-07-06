//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_worklet_zfp_type_info_compressor_h
#define vtk_m_worklet_zfp_type_info_compressor_h

#include <vtkm/Math.h>

namespace vtkm
{
namespace worklet
{
namespace zfp
{
template <typename T>
inline VTKM_EXEC int get_ebias();
template <>
inline VTKM_EXEC int get_ebias<double>()
{
  return 1023;
}
template <>
inline VTKM_EXEC int get_ebias<float>()
{
  return 127;
}
template <>
inline VTKM_EXEC int get_ebias<long long int>()
{
  return 0;
}
template <>
inline VTKM_EXEC int get_ebias<int>()
{
  return 0;
}

template <typename T>
inline VTKM_EXEC int get_ebits();
template <>
inline VTKM_EXEC int get_ebits<double>()
{
  return 11;
}
template <>
inline VTKM_EXEC int get_ebits<float>()
{
  return 8;
}
template <>
inline VTKM_EXEC int get_ebits<int>()
{
  return 0;
}
template <>
inline VTKM_EXEC int get_ebits<long long int>()
{
  return 0;
}

template <typename T>
inline VTKM_EXEC int get_precision();
template <>
inline VTKM_EXEC int get_precision<double>()
{
  return 64;
}
template <>
inline VTKM_EXEC int get_precision<long long int>()
{
  return 64;
}
template <>
inline VTKM_EXEC int get_precision<float>()
{
  return 32;
}
template <>
inline VTKM_EXEC int get_precision<int>()
{
  return 32;
}

template <typename T>
inline VTKM_EXEC int get_min_exp();
template <>
inline VTKM_EXEC int get_min_exp<double>()
{
  return -1074;
}
template <>
inline VTKM_EXEC int get_min_exp<float>()
{
  return -1074;
}
template <>
inline VTKM_EXEC int get_min_exp<long long int>()
{
  return 0;
}
template <>
inline VTKM_EXEC int get_min_exp<int>()
{
  return 0;
}

template <typename T>
inline VTKM_EXEC int scalar_sizeof();

template <>
inline VTKM_EXEC int scalar_sizeof<double>()
{
  return 8;
}
template <>
inline VTKM_EXEC int scalar_sizeof<long long int>()
{
  return 8;
}
template <>
inline VTKM_EXEC int scalar_sizeof<float>()
{
  return 4;
}
template <>
inline VTKM_EXEC int scalar_sizeof<int>()
{
  return 4;
}

template <typename T>
inline VTKM_EXEC bool is_float();

template <>
inline VTKM_EXEC bool is_float<double>()
{
  return true;
}
template <>
inline VTKM_EXEC bool is_float<long long int>()
{
  return true;
}
template <>
inline VTKM_EXEC bool is_float<float>()
{
  return false;
}
template <>
inline VTKM_EXEC bool is_float<int>()
{
  return false;
}

template <typename T>
struct zfp_traits;

template <>
struct zfp_traits<double>
{
  typedef unsigned long long int UInt;
  typedef long long int Int;
};

template <>
struct zfp_traits<long long int>
{
  typedef unsigned long long int UInt;
  typedef long long int Int;
};

template <>
struct zfp_traits<float>
{
  typedef unsigned int UInt;
  typedef int Int;
};

template <>
struct zfp_traits<int>
{
  typedef unsigned int UInt;
  typedef int Int;
};

template <typename T>
inline VTKM_EXEC bool is_int()
{
  return false;
}

template <>
inline VTKM_EXEC bool is_int<int>()
{
  return true;
}

template <>
inline VTKM_EXEC bool is_int<long long int>()
{
  return true;
}

template <int T>
struct block_traits;

template <>
struct block_traits<1>
{
  typedef unsigned char PlaneType;
};

template <>
struct block_traits<2>
{
  typedef unsigned short PlaneType;
};

} // namespace zfp
} // namespace worklet
} // namespace vtkm
#endif //  vtk_m_worklet_zfp_type_info_h
