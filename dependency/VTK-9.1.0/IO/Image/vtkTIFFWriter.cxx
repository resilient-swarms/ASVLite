/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTIFFWriter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTIFFWriter.h"

#include "vtkDataArray.h"
#include "vtkErrorCode.h"
#include "vtkImageData.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkSetGet.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtk_tiff.h"

#include <sstream>
#include <vector>

vtkStandardNewMacro(vtkTIFFWriter);

//------------------------------------------------------------------------------
vtkTIFFWriter::vtkTIFFWriter()
  : TIFFPtr(nullptr)
  , Compression(PackBits)
  , Width(0)
  , Height(0)
  , Pages(0)
  , XResolution(-1.0)
  , YResolution(-1.0)
{
  // by default process active point scalars
  this->SetInputArrayToProcess(
    0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
}

//------------------------------------------------------------------------------
void vtkTIFFWriter::Write()
{
  // make sure the latest input is available.
  this->GetInputAlgorithm()->Update();
  this->SetErrorCode(vtkErrorCode::NoError);
  // Error checking
  if (this->GetInput() == nullptr)
  {
    vtkErrorMacro(<< "Write: Please specify an input!");
    return;
  }
  if (!this->FileName && !this->FilePattern)
  {
    vtkErrorMacro(<< "Write: Please specify either a FileName or a file prefix and pattern");
    this->SetErrorCode(vtkErrorCode::NoFileNameError);
    return;
  }

  // Make sure the file name is allocated - inherited from parent class,
  // would be great to rewrite in more modern C++, but sticking with superclass
  // for now to maintain behavior without rewriting/translating code.
  size_t internalFileNameSize = (this->FileName ? strlen(this->FileName) : 1) +
    (this->FilePrefix ? strlen(this->FilePrefix) : 1) +
    (this->FilePattern ? strlen(this->FilePattern) : 1) + 256;
  this->InternalFileName = new char[internalFileNameSize];
  this->InternalFileName[0] = 0;
  int bytesPrinted = 0;
  // determine the name
  if (this->FileName)
  {
    bytesPrinted = snprintf(this->InternalFileName, internalFileNameSize, "%s", this->FileName);
  }
  else
  {
    if (this->FilePrefix)
    {
      bytesPrinted = snprintf(this->InternalFileName, internalFileNameSize, this->FilePattern,
        this->FilePrefix, this->FileNumber);
    }
    else
    {
      bytesPrinted =
        snprintf(this->InternalFileName, internalFileNameSize, this->FilePattern, this->FileNumber);
    }
  }
  if (static_cast<size_t>(bytesPrinted) >= internalFileNameSize)
  {
    // Add null terminating character just to be safe.
    this->InternalFileName[internalFileNameSize - 1] = 0;
    vtkWarningMacro("Filename has been truncated.");
  }

  // Fill in image information.
  this->GetInputExecutive(0, 0)->UpdateInformation();
  int* wExtent;
  wExtent = vtkStreamingDemandDrivenPipeline::GetWholeExtent(this->GetInputInformation(0, 0));
  this->FilesDeleted = 0;
  this->UpdateProgress(0.0);

  this->WriteFileHeader(nullptr, this->GetInput(), wExtent);
  this->WriteFile(nullptr, this->GetInput(), wExtent, nullptr);
  if (this->ErrorCode == vtkErrorCode::OutOfDiskSpaceError)
  {
    this->DeleteFiles();
  }
  else
  {
    this->WriteFileTrailer(nullptr, nullptr);
  }

  delete[] this->InternalFileName;
  this->InternalFileName = nullptr;
}

//------------------------------------------------------------------------------
void vtkTIFFWriter::WriteFileHeader(ostream*, vtkImageData* data, int wExt[6])
{
  vtkDataArray* scalarArray = this->GetInputArrayToProcess(0, this->GetInput());

  int dims[3];
  data->GetDimensions(dims);
  int scomponents = scalarArray->GetNumberOfComponents();
  int stype = scalarArray->GetDataType();
  uint32_t rowsperstrip = (uint32_t)-1;

  int bps;
  switch (stype)
  {
    case VTK_CHAR:
    case VTK_SIGNED_CHAR:
    case VTK_UNSIGNED_CHAR:
      bps = 8;
      break;
    case VTK_SHORT:
    case VTK_UNSIGNED_SHORT:
      bps = 16;
      break;
    case VTK_FLOAT:
      bps = 32;
      break;
    default:
      vtkErrorMacro(<< "Unsupported data type: " << vtkImageScalarTypeNameMacro(stype));
      this->SetErrorCode(vtkErrorCode::FileFormatError);
      return;
  }

  int predictor;

  // Find the width/height of the images
  this->Width = wExt[1] - wExt[0] + 1;
  this->Height = wExt[3] - wExt[2] + 1;
  // Check if we need to write an image stack (pages > 2).
  this->Pages = wExt[5] - wExt[4] + 1;

  // Check the resolution too, assume we store it in metric (as in reader).
  this->XResolution = 10.0 / data->GetSpacing()[0];
  this->YResolution = 10.0 / data->GetSpacing()[1];

  std::stringstream writeMode;
  writeMode << "w";
  vtkTypeInt64 len =
    static_cast<vtkTypeInt64>(this->Width) * this->Height * this->Pages * scomponents * (bps / 8);
  if (len > VTK_INT_MAX)
  {
    // Large image detected, use BigTIFF mode
    writeMode << "8";
  }
  TIFF* tif = TIFFOpen(this->InternalFileName, writeMode.str().c_str());

  if (!tif)
  {
    this->TIFFPtr = nullptr;
    return;
  }
  this->TIFFPtr = tif;

  // Let the volume do its metadata, keep existing for 2D images.
  if (this->Pages > 1)
  {
    return;
  }

  uint32_t w = this->Width;
  uint32_t h = this->Height;
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, w);
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, h);
  TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, scomponents);
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bps); // Fix for stype
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  if (stype == VTK_FLOAT)
  {
    TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  }

  if (scomponents > 3)
  {
    // if number of scalar components is greater than 3, that means we assume
    // there is alpha.
    uint16_t extra_samples = scomponents - 3;
    std::vector<uint16_t> sample_info(scomponents - 3);
    sample_info[0] = EXTRASAMPLE_ASSOCALPHA;
    int cc;
    for (cc = 1; cc < scomponents - 3; cc++)
    {
      sample_info[cc] = EXTRASAMPLE_UNSPECIFIED;
    }
    TIFFSetField(tif, TIFFTAG_EXTRASAMPLES, extra_samples, sample_info.data());
  }

  int compression;
  switch (this->Compression)
  {
    case vtkTIFFWriter::PackBits:
      compression = COMPRESSION_PACKBITS;
      break;
    case vtkTIFFWriter::JPEG:
      compression = COMPRESSION_JPEG;
      break;
    case vtkTIFFWriter::Deflate:
      compression = COMPRESSION_DEFLATE;
      break;
    case vtkTIFFWriter::LZW:
      compression = COMPRESSION_LZW;
      break;
    default:
      compression = COMPRESSION_NONE;
  }
  // compression = COMPRESSION_JPEG;
  TIFFSetField(tif, TIFFTAG_COMPRESSION, compression); // Fix for compression
  uint16_t photometric = (scomponents == 1 ? PHOTOMETRIC_MINISBLACK : PHOTOMETRIC_RGB);
  if (compression == COMPRESSION_JPEG)
  {
    TIFFSetField(tif, TIFFTAG_JPEGQUALITY, 75); // Parameter
    TIFFSetField(tif, TIFFTAG_JPEGCOLORMODE, JPEGCOLORMODE_RGB);
    photometric = PHOTOMETRIC_YCBCR;
  }
  else if (compression == COMPRESSION_LZW)
  {
    predictor = 2;
    TIFFSetField(tif, TIFFTAG_PREDICTOR, predictor);
    vtkErrorMacro("LZW compression is patented outside US so it is disabled");
  }
  else if (compression == COMPRESSION_DEFLATE)
  {
    predictor = 2;
    TIFFSetField(tif, TIFFTAG_PREDICTOR, predictor);
  }

  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, photometric); // Fix for scomponents
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, rowsperstrip));
  if (this->XResolution > 0.0 && this->YResolution > 0.0)
  {
    TIFFSetField(tif, TIFFTAG_XRESOLUTION, this->XResolution);
    TIFFSetField(tif, TIFFTAG_YRESOLUTION, this->YResolution);
    TIFFSetField(tif, TIFFTAG_RESOLUTIONUNIT, RESUNIT_CENTIMETER);
  }
}

//------------------------------------------------------------------------------
void vtkTIFFWriter::WriteFile(ostream*, vtkImageData* data, int extent[6], int*)
{
  vtkDataArray* scalarArray = this->GetInputArrayToProcess(0, this->GetInput());

  // Make sure we actually have data.
  if (!scalarArray)
  {
    vtkErrorMacro(<< "Could not get data from input.");
    return;
  }

  TIFF* tif = reinterpret_cast<TIFF*>(this->TIFFPtr);
  if (!tif)
  {
    vtkErrorMacro("Problem writing file.");
    this->SetErrorCode(vtkErrorCode::FileFormatError);
    return;
  }

  int dataType = scalarArray->GetDataType();
  // take into consideration the scalar type
  if (dataType != VTK_UNSIGNED_CHAR && dataType != VTK_UNSIGNED_SHORT && dataType != VTK_FLOAT)
  {
    vtkErrorMacro("TIFFWriter only accepts unsigned char/short or float scalars!");
    return;
  }

  if (this->Pages > 1)
  {
    // Call the correct templated function for the input
    void* inPtr = scalarArray->GetVoidPointer(0);

    switch (dataType)
    {
      vtkTemplateMacro(this->WriteVolume((VTK_TT*)(inPtr)));
      default:
        vtkErrorMacro("UpdateFromFile: Unknown data type");
    }
  }
  else
  {
    // Now write the image for the current page/directory element.
    int row = 0;
    for (int idx2 = extent[4]; idx2 <= extent[5]; ++idx2)
    {
      for (int idx1 = extent[3]; idx1 >= extent[2]; idx1--)
      {
        int coords[3] = { extent[0], idx1, idx2 };
        void* ptr = data->GetArrayPointer(scalarArray, coords);
        if (TIFFWriteScanline(tif, static_cast<unsigned char*>(ptr), row, 0) < 0)
        {
          this->SetErrorCode(vtkErrorCode::OutOfDiskSpaceError);
          break;
        }
        ++row;
      }
    }
  }
}

//------------------------------------------------------------------------------
template <typename T>
void vtkTIFFWriter::WriteVolume(T* buffer)
{
  TIFF* tif = reinterpret_cast<TIFF*>(this->TIFFPtr);
  if (!tif)
  {
    vtkErrorMacro("Problem writing volume.");
    this->SetErrorCode(vtkErrorCode::FileFormatError);
    return;
  }
  int width = this->Width;
  int height = this->Height;
  int pages = this->Pages;

  uint32_t w = width;
  uint32_t h = height;
  int bitsPerSample = sizeof(T) * 8;

  for (int page = 0; page < pages; ++page)
  {
    this->UpdateProgress(static_cast<double>(page + 1) / pages);

    // TIFF directory set up/tags.
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, w);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, h);
    TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bitsPerSample);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

    int compression;
    switch (this->Compression)
    {
      case vtkTIFFWriter::PackBits:
        compression = COMPRESSION_PACKBITS;
        break;
      case vtkTIFFWriter::JPEG:
        compression = COMPRESSION_JPEG;
        break;
      case vtkTIFFWriter::Deflate:
        compression = COMPRESSION_DEFLATE;
        break;
      case vtkTIFFWriter::LZW:
        compression = COMPRESSION_LZW;
        break;
      default:
        compression = COMPRESSION_NONE;
    }
    TIFFSetField(tif, TIFFTAG_COMPRESSION, compression); // Fix for compression
    if (compression == COMPRESSION_LZW)
    {
      TIFFSetField(tif, TIFFTAG_PREDICTOR, 2);
      vtkErrorMacro("LZW compression is patented outside US so it is disabled");
    }
    else if (compression == COMPRESSION_DEFLATE)
    {
      TIFFSetField(tif, TIFFTAG_PREDICTOR, 2);
    }

    if (bitsPerSample == 32)
    {
      TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    }

    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    uint32_t rowsperstrip = (uint32_t)-1;
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, rowsperstrip));
    if (this->XResolution > 0.0 && this->YResolution > 0.0)
    {
      TIFFSetField(tif, TIFFTAG_XRESOLUTION, this->XResolution);
      TIFFSetField(tif, TIFFTAG_YRESOLUTION, this->YResolution);
      TIFFSetField(tif, TIFFTAG_RESOLUTIONUNIT, RESUNIT_CENTIMETER);
    }

    // Extra pieces for multidirectory files.
    TIFFSetField(tif, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);
    TIFFSetField(tif, TIFFTAG_PAGENUMBER, page, pages);

    T* volume = buffer;
    volume += width * height * page;
    for (int i = 0; i < height; ++i)
    {
      T* tmp = volume + i * width;
      if (TIFFWriteScanline(tif, reinterpret_cast<char*>(tmp), i, 0) < 0)
      {
        this->SetErrorCode(vtkErrorCode::OutOfDiskSpaceError);
        return;
      }
    }
    if (!TIFFWriteDirectory(tif))
    {
      this->SetErrorCode(vtkErrorCode::OutOfDiskSpaceError);
      return;
    }
  }
}

//------------------------------------------------------------------------------
void vtkTIFFWriter::WriteFileTrailer(ostream*, vtkImageData*)
{
  TIFF* tif = reinterpret_cast<TIFF*>(this->TIFFPtr);
  if (tif)
  {
    TIFFClose(tif);
  }
  else
  {
    vtkErrorMacro("Problem writing trailer.");
    this->SetErrorCode(vtkErrorCode::FileFormatError);
  }

  this->TIFFPtr = nullptr;
}

//------------------------------------------------------------------------------
void vtkTIFFWriter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Compression: ";
  if (this->Compression == vtkTIFFWriter::PackBits)
  {
    os << "Pack Bits\n";
  }
  else if (this->Compression == vtkTIFFWriter::JPEG)
  {
    os << "JPEG\n";
  }
  else if (this->Compression == vtkTIFFWriter::Deflate)
  {
    os << "Deflate\n";
  }
  else if (this->Compression == vtkTIFFWriter::LZW)
  {
    os << "LZW\n";
  }
  else // if ( this->Compression == vtkTIFFWriter::NoCompression )
  {
    os << "No Compression\n";
  }
}
