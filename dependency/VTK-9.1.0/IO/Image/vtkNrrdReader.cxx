// -*- c++ -*-
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkNrrdReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*----------------------------------------------------------------------------
 Copyright (c) Sandia Corporation
 See Copyright.txt or http://www.paraview.org/HTML/Copyright.html for details.
----------------------------------------------------------------------------*/

#include "vtkNrrdReader.h"

#include "vtkCharArray.h"
#include "vtkErrorCode.h"
#include "vtkImageData.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStringArray.h"

// Header for zlib
#include "vtk_zlib.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include <algorithm>
#include <istream>
#include <sstream>
#include <string>
#include <vector>
#include <vtksys/FStream.hxx>
#include <vtksys/SystemTools.hxx>

#include <cctype>
#include <cmath>
#include <cstring>

#include "vtkSmartPointer.h"

#define VTK_CREATE(type, name) vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

//=============================================================================
static std::string trim(std::string s)
{
  size_t start = 0;
  while ((start < s.length()) && (isspace(s[start])))
  {
    start++;
  }
  size_t end = s.length();
  while ((end > start) && (isspace(s[end - 1])))
  {
    end--;
  }
  return s.substr(start, end - start);
}

//------------------------------------------------------------------------------
static std::vector<std::string> split(std::string s)
{
  std::vector<std::string> result;
  size_t startValue = 0;
  while (true)
  {
    while ((startValue < s.length()) && isspace(s[startValue]))
    {
      startValue++;
    }
    if (startValue >= s.length())
      return result;
    size_t endValue = startValue;
    while ((endValue < s.length()) && !isspace(s[endValue]))
    {
      endValue++;
    }
    result.push_back(s.substr(startValue, endValue - startValue));
    startValue = endValue;
  }
}

//------------------------------------------------------------------------------
static void GetVector(std::string& s, std::vector<int>& dest)
{
  std::vector<std::string> strlist = split(s);
  for (size_t i = 0; i < dest.size(); i++)
  {
    if (i < strlist.size())
    {
      dest[i] = atoi(strlist[i].c_str());
    }
    else
    {
      dest[i] = 0;
    }
  }
}

//------------------------------------------------------------------------------
static void GetVector(std::string& s, std::vector<double>& dest)
{
  std::vector<std::string> strlist = split(s);
  for (size_t i = 0; i < dest.size(); i++)
  {
    if (i < strlist.size())
    {
      dest[i] = atof(strlist[i].c_str());
    }
    else
    {
      dest[i] = 0.0;
    }
  }
}

//------------------------------------------------------------------------------
static std::vector<double> ParseVector(std::string s)
{
  std::vector<double> result;

  s = trim(s);
  if ((s[0] != '(') || (s[s.length() - 1] != ')'))
    return result;
  s = s.substr(1, s.length() - 2);
  while (true)
  {
    size_t i = s.find(',');
    std::string value = s.substr(0, i);
    result.push_back(atof(value.c_str()));
    if (i == std::string::npos)
      break;
    s = s.substr(i + 1);
  }

  return result;
}

//------------------------------------------------------------------------------
static int NrrdType2VTKType(std::string nrrdType)
{
  nrrdType = trim(nrrdType);
  if ((nrrdType == "signed char") || (nrrdType == "int8") || (nrrdType == "int8_t"))
  {
    return VTK_CHAR;
  }
  else if ((nrrdType == "uchar") || (nrrdType == "unsigned char") || (nrrdType == "uint8") ||
    (nrrdType == "uint8_t"))
  {
    return VTK_UNSIGNED_CHAR;
  }
  else if ((nrrdType == "short") || (nrrdType == "short int") || (nrrdType == "signed short") ||
    (nrrdType == "signed short int") || (nrrdType == "int16") || (nrrdType == "int16_t"))
  {
    return VTK_SHORT;
  }
  else if ((nrrdType == "ushort") || (nrrdType == "unsigned short") ||
    (nrrdType == "unsigned short int") || (nrrdType == "uint16") || (nrrdType == "uint16_t"))
  {
    return VTK_UNSIGNED_SHORT;
  }
  else if ((nrrdType == "int") || (nrrdType == "signed int") || (nrrdType == "int32") ||
    (nrrdType == "int32_t"))
  {
    return VTK_INT;
  }
  else if ((nrrdType == "uint") || (nrrdType == "unsigned int") || (nrrdType == "uint32") ||
    (nrrdType == "uint32_t"))
  {
    return VTK_UNSIGNED_INT;
  }
  else if ((nrrdType == "longlong") || (nrrdType == "long long") || (nrrdType == "long long int") ||
    (nrrdType == "signed long long") || (nrrdType == "signed long long int") ||
    (nrrdType == "int64") || (nrrdType == "int64_t"))
  {
    return VTK_TYPE_INT64;
  }
  else if ((nrrdType == "ulonglong") || (nrrdType == "unsigned long long") ||
    (nrrdType == "unsigned long long int") || (nrrdType == "uint64") || (nrrdType == "uint64_t"))
  {
    return VTK_TYPE_UINT64;
  }
  else if (nrrdType == "float")
  {
    return VTK_FLOAT;
  }
  else if (nrrdType == "double")
  {
    return VTK_DOUBLE;
  }
  else if (nrrdType == "block")
  {
    vtkGenericWarningMacro(<< "Reading blocks not supported.");
    return VTK_VOID;
  }
  else
  {
    vtkGenericWarningMacro(<< "Unknown type: '" << nrrdType << "'");
    return VTK_VOID;
  }
}

//------------------------------------------------------------------------------

vtkObjectFactoryNewMacro(vtkNrrdReader);

//------------------------------------------------------------------------------
vtkNrrdReader::vtkNrrdReader()
{
  this->DataFiles = vtkStringArray::New();
  this->Encoding = ENCODING_RAW;
}

vtkNrrdReader::~vtkNrrdReader()
{
  this->DataFiles->Delete();
  this->DataFiles = nullptr;
}

void vtkNrrdReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Encoding: ";
  switch (this->Encoding)
  {
    case ENCODING_RAW:
      os << "raw" << endl;
      break;
    case ENCODING_ASCII:
      os << "ascii" << endl;
      break;
    case ENCODING_GZIP:
      os << "gzip" << endl;
      break;
    default:
      os << "UNKNOWN!" << endl;
      break;
  }
}

//------------------------------------------------------------------------------
int vtkNrrdReader::CanReadFile(const char* filename)
{
  vtksys::ifstream file(filename, ios::in | ios::binary);
  std::string firstLine;
  getline(file, firstLine);
  if (firstLine.substr(0, 4) == "NRRD")
  {
    return 2;
  }
  else
  {
    return 0;
  }
}

//------------------------------------------------------------------------------
int vtkNrrdReader::RequestInformation(
  vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  if (!this->ReadHeader())
    return 0;

  return this->Superclass::RequestInformation(request, inputVector, outputVector);
}

//------------------------------------------------------------------------------
int vtkNrrdReader::ReadHeaderInternal(vtkCharArray* headerBuffer)
{
  if (!this->FileName)
  {
    vtkErrorMacro(<< "No filename set.");
    return 0;
  }

  vtksys::ifstream file(this->FileName, ios::in | ios::binary);
  // Read in 4 MB.  Assuming that the header will be smaller than that.
  headerBuffer->SetNumberOfTuples(0x400000);
  file.read(headerBuffer->GetPointer(0), 0x400000 - 1);
  vtkIdType buffersize = file.gcount();
  headerBuffer->SetValue(buffersize, '\0');
  headerBuffer->SetNumberOfTuples(buffersize + 1);

  // Find a blank line (which signals the end of the header).  Be careful
  // because line endings can be "\n", "\r\n", or both.  It is possible that
  // the entire file is the header (happens with "detached headers").
  char* bufferStart = headerBuffer->GetPointer(0);
  char* s = bufferStart;
  while ((s = strchr(s + 1, '\n')) != nullptr)
  {
    // If you find a double line ending, terminate the string and shorten the
    // buffer.
    if (s[1] == '\n')
    {
      s[2] = '\0';
      headerBuffer->SetNumberOfTuples(static_cast<vtkIdType>(s + 3 - bufferStart));
      break;
    }
    if ((s[1] == '\r') && (s[2] == '\n'))
    {
      s[3] = '\0';
      headerBuffer->SetNumberOfTuples(static_cast<vtkIdType>(s + 4 - bufferStart));
      break;
    }
  }

  return 1;
}

//------------------------------------------------------------------------------
int vtkNrrdReader::ReadHeader()
{
  VTK_CREATE(vtkCharArray, headerBuffer);

  if (!this->ReadHeaderInternal(headerBuffer))
  {
    return 0;
  }

  return this->ReadHeader(headerBuffer);
}

//------------------------------------------------------------------------------
int vtkNrrdReader::ReadHeader(vtkCharArray* headerBuffer)
{
  this->HeaderSize = headerBuffer->GetNumberOfTuples() - 1;
  this->ManualHeaderSize = 1;

  std::string headerStringBuffer = headerBuffer->GetPointer(0);
  std::stringstream header(headerStringBuffer);

  std::string line;
  getline(header, line);
  if (line.substr(0, 4) != "NRRD")
  {
    vtkErrorMacro(<< this->FileName << " is not a nrrd file.");
    return 0;
  }

  this->DataFiles->Initialize();
  int numDimensions = 0;
  int subDimension = -1;
  std::vector<int> dimSizes;
  std::vector<double> dimSpacing;
  this->FileLowerLeft = 1;
  this->Encoding = ENCODING_RAW;
  while (true)
  {
    getline(header, line);
    if (line.length() < 1)
      break;

    if (line[0] == '#')
    {
      // Comment.  Ignore.
      continue;
    }

    size_t delm = line.find(": ");
    if (delm != std::string::npos)
    {
      // A field/description pair.
      std::string field = line.substr(0, delm);
      std::string description = trim(line.substr(delm + 2));
      if (field == "dimension")
      {
        numDimensions = atoi(description.c_str());
      }
      else if (field == "sizes")
      {
        dimSizes.resize(numDimensions);
        GetVector(description, dimSizes);
      }
      else if (field == "spacings")
      {
        dimSpacing.resize(numDimensions);
        GetVector(description, dimSpacing);
      }
      else if (field == "type")
      {
        this->DataScalarType = NrrdType2VTKType(description);
        if (this->DataScalarType == VTK_VOID)
          return 0;
      }
      else if (field == "encoding")
      {
        std::transform(description.begin(), description.end(), description.begin(), ::tolower);
        if (description == "raw")
        {
          this->Encoding = ENCODING_RAW;
        }
        else if (description == "ascii" || description == "txt" || description == "text")
        {
          this->Encoding = ENCODING_ASCII;
        }
        else if (description == "gzip" || description == "gz")
        {
          this->Encoding = ENCODING_GZIP;
        }
        else
        {
          vtkErrorMacro(<< "Unsupported encoding: " << description);
          return 0;
        }
      }
      else if ((field == "data file") || (field == "datafile"))
      {
        std::vector<std::string> filepatterninfo = split(description);
        if (filepatterninfo[0] == "LIST")
        {
          // After LIST there is an optional subdimension (see next case below).
          subDimension =
            ((filepatterninfo.size() > 1) ? atoi(filepatterninfo[1].c_str()) : numDimensions);

          // In this mode files are listed one per line to the end of the file.
          while (true)
          {
            getline(header, line);
            trim(line);
            if (line.length() < 1)
              break;
            this->DataFiles->InsertNextValue(line);
          }
          break;
        }
        else if (filepatterninfo.size() >= 4)
        {
          // description should be "<format> <min> <max> <step> [<subdim>]"
          // where <format> is a string to be processed by snprintf and <min>,
          // <max>, and <step> form the numbers.  <subdim> defines on which
          // dimension the files are split up.
          std::string format = filepatterninfo[0];
          int min = atoi(filepatterninfo[1].c_str());
          int max = atoi(filepatterninfo[2].c_str());
          int step = atoi(filepatterninfo[3].c_str());
          subDimension =
            ((filepatterninfo.size() > 4) ? atoi(filepatterninfo[4].c_str()) : numDimensions);
          size_t filenamelen = format.size() + 20;
          char* filename = new char[filenamelen];
          for (int i = min; i <= max; i += step)
          {
            snprintf(filename, filenamelen, format.c_str(), i);
            this->DataFiles->InsertNextValue(filename);
          }
          delete[] filename;
        }
        else
        {
          // description is simply a filename
          this->DataFiles->InsertNextValue(description);
        }
      }
      else if (field == "space")
      {
        // All spaces are either 3D or 3D with time.
        if (description.find("time") != std::string::npos)
        {
          vtkErrorMacro(<< "Time in NRRD array not supported (yet).");
          return 0;
        }
        if ((description == "left-anterior-superior") || (description == "LAS") ||
          (description == "3D-left-handed"))
        {
          this->FileLowerLeft = 0;
        }
        numDimensions = 3;
      }
      else if (field == "labels")
      {
        std::string dataname = description.substr(description.find('\"') + 1);
        dataname = dataname.substr(0, dataname.find('\"'));
        delete[] this->ScalarArrayName;
        this->ScalarArrayName = new char[dataname.size() + 1];
        strcpy(this->ScalarArrayName, dataname.c_str());
      }
      else if (field == "space dimension")
      {
        numDimensions = atoi(description.c_str());
      }
      else if (field == "space origin")
      {
        std::vector<double> origin = ParseVector(description);
        for (size_t i = 0; (i < 3) && (i < origin.size()); i++)
        {
          this->DataOrigin[i] = origin[i];
        }
      }
      else if (field == "space directions")
      {
        std::vector<std::string> directions = split(description);
        dimSpacing.resize(0);
        for (size_t j = 0; j < directions.size(); j++)
        {
          if (directions[j] == "none")
          {
            dimSpacing.push_back(0);
            continue;
          }
          std::vector<double> dir = ParseVector(directions[j]);
          // We don't support orientation, but we do support spacing.
          double mag = 0.0;
          for (size_t k = 0; k < dir.size(); k++)
          {
            mag += dir[k] * dir[k];
          }
          dimSpacing.push_back(sqrt(mag));
        }
      }
      else if (field == "endian")
      {
        if (description == "little")
        {
#ifdef VTK_WORDS_BIGENDIAN
          this->SwapBytes = 1;
#else
          this->SwapBytes = 0;
#endif
        }
        else if (description == "big")
        {
#ifdef VTK_WORDS_BIGENDIAN
          this->SwapBytes = 0;
#else
          this->SwapBytes = 1;
#endif
        }
        else
        {
          vtkErrorMacro(<< "Unknown endian: '" << description << "'");
          return 0;
        }
      }
      else if ((field == "line skip") || (field == "lineskip"))
      {
        if (atoi(description.c_str()) != 0)
        {
          vtkErrorMacro(<< "line skip not supported");
          return 0;
        }
      }
      else if ((field == "byte skip") || (field == "byteskip"))
      {
        if (atoi(description.c_str()) != 0)
        {
          vtkErrorMacro(<< "byte skip not supported");
          return 0;
        }
      }
      else if ((field == "space units") || (field == "sample units") || (field == "sampleunits") ||
        (field == "measurement frame") || (field == "block size") || (field == "blocksize") ||
        (field == "content") || (field == "thicknesses") || (field == "axis mins") ||
        (field == "axismins") || (field == "axis maxs") || (field == "axismaxs") ||
        (field == "centers") || (field == "centerings") || (field == "units") ||
        (field == "kinds") || (field == "min") || (field == "max") || (field == "old min") ||
        (field == "oldmin") || (field == "old max") || (field == "oldmax") || (field == "number"))
      {
        // Ignored.
      }
      else
      {
        vtkWarningMacro(<< "Unknown field: '" << field << "'");
      }
      continue;
    }

    delm = line.find(":=");
    if (delm != std::string::npos)
    {
      // A key/value pair.
      continue;
    }
  }

  // NRRD does not distinguish between vector entries and dimensions.  For
  // example, RGB tuples are represented by adding a dimension of size 3.
  // VTK really needs to know the difference.  Here we are going to guess.
  // If the fastest changing dimension is 9 or less we consider that a tuple.
  // We will also consider any 4th dimension as a tuple.
  if ((dimSizes.size() > 3) || ((!dimSizes.empty()) && (dimSizes[0] <= 9)) ||
    ((!dimSpacing.empty()) && (dimSpacing[0] == 0)))
  {
    this->NumberOfScalarComponents = dimSizes[0];
    dimSizes.erase(dimSizes.begin());
    if (!dimSpacing.empty())
      dimSpacing.erase(dimSpacing.begin());
    subDimension--;
  }
  else
  {
    this->NumberOfScalarComponents = 1;
  }

  // Record the dimensions.
  this->FileDimensionality = static_cast<int>(dimSizes.size());
  for (unsigned int i = 0; i < 3; i++)
  {
    this->DataExtent[i * 2 + 0] = 0;
    this->DataExtent[i * 2 + 1] = (i < dimSizes.size()) ? dimSizes[i] - 1 : 0;
    this->DataSpacing[i] = (i < dimSpacing.size()) ? dimSpacing[i] : 1;
  }

  if (this->DataFiles->GetNumberOfValues() > 0)
  {
    if (this->DataFiles->GetNumberOfValues() > 1)
    {
      this->FileDimensionality--;
      if (this->FileDimensionality != 2)
      {
        vtkErrorMacro(<< "Data split into multiple files is only supported"
                      << " when each file is 2D (+ an optional vector"
                      << " dimension).");
      }
      if (subDimension != 3)
      {
        vtkErrorMacro(<< "Data split into multiple files is only supported"
                      << " when each file is 2D (+ an optional vector"
                      << " dimension).  This means the subdim must be on"
                      << " that third (or fourth in the case of a vector)"
                      << " dimension.");
      }
    }
    std::string parentDir = vtksys::SystemTools::GetParentDirectory(this->FileName);
    for (vtkIdType i = 0; i < this->DataFiles->GetNumberOfValues(); i++)
    {
      std::string relativePath = this->DataFiles->GetValue(i);
      std::string fullPath = vtksys::SystemTools::CollapseFullPath(relativePath, parentDir.c_str());
      this->DataFiles->SetValue(i, fullPath);
    }
    // Header file pointing to data files.  Data files should have no header.
    this->HeaderSize = 0;
    this->ManualHeaderSize = 0;
  }

  return 1;
}

//=============================================================================
int vtkNrrdReader::RequestData(
  vtkInformation* request, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // Get rid of superclasses FileNames.  We don't support that functionality,
  // but we exploit that of the superclass.
  if (this->FileNames != nullptr)
  {
    this->FileNames->Delete();
    this->FileNames = nullptr;
  }

  char* saveFileName = this->FileName;

  if (this->DataFiles->GetNumberOfValues() == 1)
  {
    this->FileName = const_cast<char*>(this->DataFiles->GetValue(0).c_str());
  }
  else if (this->DataFiles->GetNumberOfValues() > 1)
  {
    this->FileNames = this->DataFiles;
  }

  int result;

  if (this->Encoding == ENCODING_RAW)
  {
    // Superclass knows how to read raw data.  Use that.

    result = this->Superclass::RequestData(request, inputVector, outputVector);
  }
  else if (this->Encoding == ENCODING_ASCII)
  {
    vtkImageData* outputData = vtkImageData::GetData(outputVector);
    this->AllocateOutputData(outputData, outputVector->GetInformationObject(0));
    if (outputData == nullptr)
    {
      vtkErrorMacro(<< "Data not created correctly?");
      return 0;
    }

    result = this->ReadDataAscii(outputData);
  }
  else if (this->Encoding == ENCODING_GZIP)
  {
    vtkImageData* outputData = vtkImageData::GetData(outputVector);
    this->AllocateOutputData(outputData, outputVector->GetInformationObject(0));
    if (outputData == nullptr)
    {
      vtkErrorMacro(<< "Data not created correctly?");
      return 0;
    }

    result = this->ReadDataGZip(outputData);
  }
  else
  {
    vtkErrorMacro(<< "Bad encoding set");
    result = 0;
  }

  this->FileName = saveFileName;
  this->FileNames = nullptr;

  return result;
}

//=============================================================================
template <typename T>
int vtkNrrdReaderReadDataAsciiTemplate(vtkNrrdReader* self, vtkImageData* output, T* outBuffer)
{
  // Get the requested extent
  int outExtent[6];
  output->GetExtent(outExtent);
  int numComponents = output->GetNumberOfScalarComponents();

  int fileDataExtent[6];
  self->GetDataExtent(fileDataExtent);
  vtkIdType fileDataIncrements[3];
  fileDataIncrements[0] = numComponents;
  fileDataIncrements[1] = fileDataIncrements[0] * fileDataExtent[1];
  fileDataIncrements[2] = fileDataIncrements[1] * fileDataExtent[3];

  vtkStringArray* filenames = self->GetFileNames();
  vtkStdString filename = self->GetFileName();

  vtksys::ifstream file;
  if (self->GetFileDimensionality() == 3)
  {
    if (filenames != nullptr)
    {
      filename = filenames->GetValue(0);
    }
    file.open(filename.c_str(), std::ios::in);
    if (file.fail())
    {
      vtkErrorWithObjectMacro(self, "Could not open file " << filename);
      return 0;
    }
    // Skip to start of first slab.
    for (vtkIdType skip = 0; skip < fileDataIncrements[2] * outExtent[4]; skip++)
    {
      T dummy;
      file >> dummy;
    }
  }

  vtkIdType bufferIndex = 0;
  int fileDataIndex[3];
  for (fileDataIndex[2] = outExtent[4]; fileDataIndex[2] <= outExtent[5]; fileDataIndex[2]++)
  {
    if (self->GetFileDimensionality() == 2)
    {
      if (file.is_open())
      {
        file.close();
      }
      if (filenames != nullptr)
      {
        filename = filenames->GetValue(fileDataIndex[2]);
      }
      file.open(filename.c_str(), std::ios::in);
      if (file.fail())
      {
        vtkErrorWithObjectMacro(self, "Could not open file " << filename);
        return 0;
      }
    }
    // Skip unused rows.
    for (vtkIdType skip = 0; skip < fileDataIncrements[1] * outExtent[2]; skip++)
    {
      T dummy;
      file >> dummy;
    }
    for (fileDataIndex[1] = outExtent[2]; fileDataIndex[1] <= outExtent[3]; fileDataIndex[1]++)
    {
      // Skip unused columns.
      for (vtkIdType skip = 0; skip < fileDataIncrements[0] * outExtent[0]; skip++)
      {
        T dummy;
        file >> dummy;
      }
      for (fileDataIndex[0] = outExtent[0]; fileDataIndex[0] <= outExtent[1]; fileDataIndex[0]++)
      {
        file >> outBuffer[bufferIndex];
        bufferIndex++;
      }
      // Skip unused columns.
      for (vtkIdType skip = 0; skip < fileDataIncrements[0] * (fileDataExtent[1] - outExtent[1]);
           skip++)
      {
        T dummy;
        file >> dummy;
      }
    }
    // Skip unused rows.
    for (vtkIdType skip = 0; skip < fileDataIncrements[1] * (fileDataExtent[2] - outExtent[2]);
         skip++)
    {
      T dummy;
      file >> dummy;
    }
  }

  file.close();
  return 1;
}

//=============================================================================
int vtkNrrdReader::ReadDataAscii(vtkImageData* output)
{
  void* outBuffer = output->GetScalarPointer();
  switch (output->GetScalarType())
  {
    vtkTemplateMacro(vtkNrrdReaderReadDataAsciiTemplate(this, output, (VTK_TT*)(outBuffer)));
    default:
      vtkErrorMacro("Unknown data type");
      return 0;
  }

  return 1;
}

//=============================================================================
template <typename T>
int vtkNrrdReader::vtkNrrdReaderReadDataGZipTemplate(vtkImageData* output, T* outBuffer)
{
  vtkIdType inIncr[3];
  output->GetIncrements(inIncr);

  vtkStringArray* filenames = this->GetFileNames();
  vtkStdString filename = this->GetFileName();

  int outExtent[6];
  output->GetExtent(outExtent);

  int fileDataExtent[6];
  this->GetDataExtent(fileDataExtent);

  // cannot do partial reads efficiently from gzipped data stream
  for (int i = 0; i < 6; i++)
  {
    if (fileDataExtent[i] != outExtent[i])
    {
      vtkErrorMacro(<< "File and requested extents must agree: " << filename);
      this->SetErrorCode(vtkErrorCode::UnknownError);
      return 0;
    }
  }

  if ((this->GetFileDimensionality() == 2) || (this->GetFileDimensionality() == 3))
  {
    if (filenames != nullptr)
    {
      filename = filenames->GetValue(0);
    }

    int flags = O_RDONLY;
#ifdef _WIN32
    flags |= O_BINARY;
#endif
    int fd = open(filename.c_str(), flags);
    if (fd < 0)
    {
      vtkErrorMacro(<< "Couldn't open nrrd file: " << filename);
      this->SetErrorCode(vtkErrorCode::CannotOpenFileError);
      return 0;
    }
    else
    {
      off_t offset = this->GetHeaderSize();
      lseek(fd, offset, SEEK_SET);
      gzFile gf = gzdopen(fd, "r");
      if (gf == nullptr)
      {
        vtkErrorMacro(<< "Couldn't open gzip stream from nrrd file: " << filename);
        this->SetErrorCode(vtkErrorCode::CannotOpenFileError);
        close(fd);
        return 0;
      }

      unsigned numBytes = static_cast<unsigned>(inIncr[2]) * sizeof(T);
      if (this->GetFileDimensionality() == 3)
      {
        auto width = fileDataExtent[1] - fileDataExtent[0] + 1;
        auto height = fileDataExtent[3] - fileDataExtent[2] + 1;
        auto depth = fileDataExtent[5] - fileDataExtent[4] + 1;
        numBytes = static_cast<unsigned>(width) * height * depth * sizeof(T);
      }

      int rsize = gzread(gf, outBuffer, numBytes);
      if ((rsize < 0) || (static_cast<unsigned>(rsize) != numBytes))
      {
        vtkErrorMacro(<< "Couldn't read gzip data from nrrd file: " << filename << " " << numBytes
                      << "/" << rsize << ", GetHeaderSize(): " << this->GetHeaderSize());
        this->SetErrorCode(vtkErrorCode::PrematureEndOfFileError);
        gzclose(gf);
        return 0;
      }
      gzclose(gf);
    }
  }
  else
  {
    vtkErrorMacro(<< "Unsupported dimensionality in nrrd file: " << filename);
    this->SetErrorCode(vtkErrorCode::UnrecognizedFileTypeError);
    return 0;
  }

  return 1;
}

//=============================================================================
int vtkNrrdReader::ReadDataGZip(vtkImageData* output)
{
  void* outBuffer = output->GetScalarPointer();
  switch (output->GetScalarType())
  {
    vtkTemplateMacro(vtkNrrdReaderReadDataGZipTemplate(output, (VTK_TT*)(outBuffer)));
    default:
      vtkErrorMacro("Unknown data type");
      return 0;
  }

  return 1;
}
