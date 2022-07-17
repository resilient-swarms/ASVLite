//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================
#ifndef vtk_m_io_PixelTypes_h
#define vtk_m_io_PixelTypes_h

#include <vtkm/Types.h>
#include <vtkm/VecTraits.h>

VTKM_THIRDPARTY_PRE_INCLUDE
#include <vtkm/thirdparty/lodepng/vtkmlodepng/lodepng.h>
VTKM_THIRDPARTY_POST_INCLUDE

namespace vtkm
{
namespace io
{

// ----------------------------------------------------------------------
// Define custom SFINAE structures to calculate the VTKM types associated
// with provided BitDepths
template <const vtkm::Id size, typename = void>
struct ComponentTypeFromSize
{
};

template <const vtkm::Id size>
struct ComponentTypeFromSize<size, typename std::enable_if<(size == 8)>::type>
{
  using type = vtkm::UInt8;
};
template <const vtkm::Id size>
struct ComponentTypeFromSize<size, typename std::enable_if<(size == 16)>::type>
{
  using type = vtkm::UInt16;
};
// ----------------------------------------------------------------------

/// \brief Base type for more complex pixels (RGB, Greyscale, etc) that describes various values
/// such as bit-depth, channel width, bytes per pixel, and how various data should be polled
///
/// \c BasePixel takes BitDepth and Channels as template parameters. BitDepth describes the number
/// of bits in the pixel, while Channels describes the multiple of bits that are available.
/// BasePixel extends vtkm::Vec.  The ComponentType is pulled from the ComponentTypeFromSize
/// SFINAE struct defined above.  This helps with optimizing the pixel size for a given
/// bit-depth.  The Size is pulled from the Channels param.
///
/// \c BasePixel requires BitDepths that are > 8 and powers of 2 at the moment.  BitDepths of
/// 4, 2, or 1 bit are not correctly handled at the moment.
///
/// \c BasePixel describes how to populate itself from an unsigned char pointer (assuming that
/// the data stored within the pointer matches the bit-depth and channels described by the
/// BasePixel type), and how to fill in data for an unsigned char pointer.  This is mostly
/// useful in serialization and deserialization to various image formats.
///
template <const vtkm::Id BitDepth, const vtkm::IdComponent Channels>
class BasePixel : public vtkm::Vec<typename ComponentTypeFromSize<BitDepth>::type, Channels>
{
  static_assert(BitDepth >= 8, "BitDepth not >= 8");
  static_assert(!(BitDepth & (BitDepth - 1)), "BitDepth not a power of 2");

public:
  using Superclass = vtkm::Vec<typename ComponentTypeFromSize<BitDepth>::type, Channels>;
  using ComponentType = typename Superclass::ComponentType;
  using BaseType = BasePixel<BitDepth, Channels>;

  static constexpr vtkm::IdComponent BIT_DEPTH = BitDepth;
  static constexpr vtkm::IdComponent NUM_BYTES = BitDepth / 8;
  static constexpr vtkm::IdComponent MAX_COLOR_VALUE = (1 << BitDepth) - 1;
  static constexpr vtkm::IdComponent NUM_CHANNELS = Superclass::NUM_COMPONENTS;
  static constexpr vtkm::IdComponent BYTES_PER_PIXEL = NUM_CHANNELS * NUM_BYTES;

  using Superclass::Superclass;
  BasePixel() = default;

  /// Fills in this->Components by calling ConstructPixelFromImage. Requires
  /// the base vec values to be zeroed out initially.
  ///
  BasePixel(const unsigned char* imageData, const vtkm::Id index)
    : Superclass(0)
  {
    ConstructPixelFromImage(imageData, index);
  }

  virtual ~BasePixel() = default;

  /// Calculates this difference between two pixels as a single value.
  ///
  virtual ComponentType Diff(const BaseType& pixel) const = 0;

  /// Generates a Vec4f_32 from the current data available in the pixel
  ///
  virtual vtkm::Vec4f_32 ToVec4f() const = 0;

  /// Implement the << operator for this class type.  Will call the overloaded
  /// print method for the subclassed type.
  ///
  friend std::ostream& operator<<(std::ostream& os, const BaseType& basePixel)
  {
    basePixel.print(os);
    return os;
  }

  /// Takes an output imageData pointer and in index to a location in that dataset
  /// and fills in the pixel data at the location. Utilizes BIT_DEPTH and
  /// NUM_CHANNELS to fill in multiple bytes worth of data if necessary.
  ///
  void FillImageAtIndexWithPixel(unsigned char* imageData, const vtkm::Id index);

protected:
  /// Takes an input imageData pointer and an index to a location in that dataset
  /// and fills in this->Components correctly using the provided BIT_DEPTH and
  /// NUM_CHANNELS. Does NOT 0 out the current Components.
  ///
  void ConstructPixelFromImage(const unsigned char* imageData, const vtkm::Id index);

  virtual void print(std::ostream& os) const = 0;
};


template <const vtkm::Id BitDepth>
class RGBPixel : public BasePixel<BitDepth, 3>
{
public:
  // RGB values are stored in a vtkm::Vec<ComponentType, 3>
  using Superclass = BasePixel<BitDepth, 3>;
  using ComponentType = typename Superclass::ComponentType;
  static constexpr vtkm::png::LodePNGColorType PNG_COLOR_TYPE =
    vtkm::png::LodePNGColorType::LCT_RGB;

  using Superclass::Superclass;
  RGBPixel() = default;
  RGBPixel(vtkm::Vec4f_32 tuple)
    : Superclass(static_cast<ComponentType>(tuple[0] * this->MAX_COLOR_VALUE),
                 static_cast<ComponentType>(tuple[1] * this->MAX_COLOR_VALUE),
                 static_cast<ComponentType>(tuple[2] * this->MAX_COLOR_VALUE))
  {
  }

  virtual ~RGBPixel() = default;

  ComponentType Diff(const Superclass& pixel) const override;
  vtkm::Vec4f_32 ToVec4f() const override;

protected:
  void print(std::ostream& os) const override
  {
    os << "(" << (int)this->Components[0] << "," << (int)this->Components[1] << ","
       << (int)this->Components[2] << ")";
  }
};

// Default types for 8 and 16 bit RGB pixels
using RGBPixel_8 = RGBPixel<8>;
using RGBPixel_16 = RGBPixel<16>;

template <const vtkm::Id BitDepth>
class GreyPixel : public BasePixel<BitDepth, 1>
{
public:
  // Grey values are stored in a vtkm::Vec<ComponentType, 1>
  // Note: A vec of size 1 is used instead of just a `ComponentType`
  // in order to simplify the pixel helper functions
  using Superclass = BasePixel<BitDepth, 1>;
  using ComponentType = typename Superclass::ComponentType;
  static constexpr vtkm::png::LodePNGColorType PNG_COLOR_TYPE =
    vtkm::png::LodePNGColorType::LCT_GREY;

  using Superclass::Superclass;
  GreyPixel() = default;
  GreyPixel(vtkm::Vec4f_32 tuple)
    : Superclass(
        static_cast<ComponentType>((tuple[0] + tuple[1] + tuple[2]) * this->MAX_COLOR_VALUE / 3))
  {
  }

  virtual ~GreyPixel() = default;

  ComponentType Diff(const Superclass& pixel) const override;
  vtkm::Vec4f_32 ToVec4f() const override;

protected:
  void print(std::ostream& os) const override { os << "(" << (int)this->Components[0] << ")"; }
};

// Default types for 8 and 16 bit Grey pixels
using GreyPixel_16 = GreyPixel<16>;
using GreyPixel_8 = GreyPixel<8>;

} // namespace io
} // namespace vtkm

#include <vtkm/io/PixelTypes.hxx>

#endif //vtk_m_io_PixelTypes_h
