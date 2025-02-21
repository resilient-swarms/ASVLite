//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/io/ErrorIO.h>
#include <vtkm/io/ImageWriterHDF5.h>
#include <vtkm/io/PixelTypes.h>

#include <hdf5.h>
#include <hdf5_hl.h>

namespace
{
// This trait is written in an unusual way since HDF5 calls H5Open() in macros
// like H5T_NATIVE_UCHAR. This makes it NOT a compile time constant. We need to
// make the trait a runtime evaluated function object.
template <typename PixelType>
struct hdf5_type_trait
{
};
template <>
struct hdf5_type_trait<vtkm::io::RGBPixel_8>
{
  auto operator()() { return H5T_NATIVE_UCHAR; }
};

template <>
struct hdf5_type_trait<vtkm::io::RGBPixel_16>
{
  auto operator()() { return H5T_NATIVE_UINT16; }
};
} //namespace

namespace vtkm
{
namespace io
{
ImageWriterHDF5::~ImageWriterHDF5() noexcept = default;

void ImageWriterHDF5::WriteDataSet(const vtkm::cont::DataSet& dataSet,
                                   const std::string& colorField)
{
  this->fieldName = colorField;
  Superclass::WriteDataSet(dataSet, colorField);
}

template <typename PixelType>
herr_t ImageWriterHDF5::WriteToFile(vtkm::Id width, vtkm::Id height, const ColorArrayType& pixels)
{
  constexpr auto BYTES_PER_PIXEL = PixelType::BYTES_PER_PIXEL;

  auto pixelPortal = pixels.ReadPortal();
  std::vector<unsigned char> imageData(pixels.GetNumberOfValues() * BYTES_PER_PIXEL);

  // copy from pixelPortal to imageData, FIXME: do we need this copy?
  vtkm::Id pixelIndex = 0;
  for (vtkm::Id yindex = 0; yindex < height; ++yindex)
  {
    for (vtkm::Id xindex = 0; xindex < width; ++xindex)
    {
      vtkm::Id vtkmIndex = yindex * width + xindex;
      PixelType(pixelPortal.Get(vtkmIndex)).FillImageAtIndexWithPixel(imageData.data(), pixelIndex);
      pixelIndex++;
    }
  }

  // Shamelessly copied from H5IMmake_image_24bit() implementation.
  auto dset_name = this->fieldName.c_str();

  // The image is stored as height*width*3 array of UCHAR/UINT16, i.e. INTERLACE_PIXEL
  hsize_t dims[] = { hsize_t(height), hsize_t(width), 3 };

  // TODO: change it to exception based error handling.
  // Create a HDF5 DataSet
  if (H5LTmake_dataset(
        this->fileid, dset_name, 3, dims, hdf5_type_trait<PixelType>{}(), imageData.data()) < 0)
  {
    return -1;
  }

  /* Attach the CLASS attribute */
  if (H5LTset_attribute_string(fileid, dset_name, "CLASS", IMAGE_CLASS) < 0)
  {
    return -1;
  }

  /* Attach the VERSION attribute */
  if (H5LTset_attribute_string(fileid, dset_name, "IMAGE_VERSION", IMAGE_VERSION) < 0)
  {
    return -1;
  }

  /* Attach the IMAGE_SUBCLASS attribute */
  if (H5LTset_attribute_string(fileid, dset_name, "IMAGE_SUBCLASS", "IMAGE_TRUECOLOR") < 0)
  {
    return -1;
  }

  /* Attach the INTERLACE_MODE attribute. This attributes is only for true color images */
  if (H5LTset_attribute_string(fileid, dset_name, "INTERLACE_MODE", "INTERLACE_PIXEL") < 0)
  {
    return -1;
  }
  return 0;
}

void ImageWriterHDF5::Write(vtkm::Id width, vtkm::Id height, const ColorArrayType& pixels)
{
  this->fileid = H5Fcreate(this->FileName.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  if (this->fileid < 0)
  {
    throw vtkm::io::ErrorIO{ "Can not create HDF5 image file" };
  }

  switch (this->Depth)
  {
    case PixelDepth::PIXEL_8:
      this->WriteToFile<vtkm::io::RGBPixel_8>(width, height, pixels);
      break;
    case PixelDepth::PIXEL_16:
      this->WriteToFile<vtkm::io::RGBPixel_16>(width, height, pixels);
      break;
  }

  H5Fclose(this->fileid);
}
}
}
