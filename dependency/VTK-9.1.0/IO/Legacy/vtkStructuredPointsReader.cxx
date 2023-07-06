/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkStructuredPointsReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkStructuredPointsReader.h"

#include "vtkDataArray.h"
#include "vtkErrorCode.h"
#include "vtkFieldData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkStructuredPoints.h"

vtkStandardNewMacro(vtkStructuredPointsReader);

vtkStructuredPointsReader::vtkStructuredPointsReader() = default;
vtkStructuredPointsReader::~vtkStructuredPointsReader() = default;

//------------------------------------------------------------------------------
void vtkStructuredPointsReader::SetOutput(vtkStructuredPoints* output)
{
  this->GetExecutive()->SetOutputData(0, output);
}

//------------------------------------------------------------------------------
vtkStructuredPoints* vtkStructuredPointsReader::GetOutput()
{
  return this->GetOutput(0);
}

//------------------------------------------------------------------------------
vtkStructuredPoints* vtkStructuredPointsReader::GetOutput(int idx)
{
  return vtkStructuredPoints::SafeDownCast(this->GetOutputDataObject(idx));
}

//------------------------------------------------------------------------------
int vtkStructuredPointsReader::ReadMetaDataSimple(
  const std::string& fname, vtkInformation* metadata)
{
  this->SetErrorCode(vtkErrorCode::NoError);

  char line[256];
  int dimsRead = 0, arRead = 0, originRead = 0;

  if (!this->OpenVTKFile(fname.c_str()) || !this->ReadHeader(fname.c_str()))
  {
    return 1;
  }

  // Read structured points specific stuff
  if (!this->ReadString(line))
  {
    vtkErrorMacro(<< "Data file ends prematurely!");
    this->CloseVTKFile();
    this->SetErrorCode(vtkErrorCode::PrematureEndOfFileError);
    return 1;
  }

  if (!strncmp(this->LowerCase(line), "dataset", 7))
  {
    // Make sure we're reading right type of geometry
    if (!this->ReadString(line))
    {
      vtkErrorMacro(<< "Data file ends prematurely!");
      this->CloseVTKFile();
      this->SetErrorCode(vtkErrorCode::PrematureEndOfFileError);
      return 1;
    }

    if (strncmp(this->LowerCase(line), "structured_points", 17) != 0)
    {
      vtkErrorMacro(<< "Cannot read dataset type: " << line);
      this->CloseVTKFile();
      this->SetErrorCode(vtkErrorCode::UnrecognizedFileTypeError);
      return 1;
    }

    // Read keyword and number of points
    while (true)
    {
      if (!this->ReadString(line))
      {
        break;
      }

      if (!strncmp(this->LowerCase(line), "dimensions", 10) && !dimsRead)
      {
        int dim[3];
        if (!(this->Read(dim) && this->Read(dim + 1) && this->Read(dim + 2)))
        {
          vtkErrorMacro(<< "Error reading dimensions!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }
        metadata->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), 0, dim[0] - 1, 0,
          dim[1] - 1, 0, dim[2] - 1);
        dimsRead = 1;
      }

      else if (!strncmp(line, "extent", 6) && !dimsRead)
      {
        int extent[6];
        if (!(this->Read(extent) && this->Read(extent + 1) && this->Read(extent + 2) &&
              this->Read(extent + 3) && this->Read(extent + 4) && this->Read(extent + 5)))
        {
          vtkErrorMacro(<< "Error reading extent!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        metadata->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), extent[0], extent[1],
          extent[2], extent[3], extent[4], extent[5]);

        dimsRead = 1;
      }

      else if (!strncmp(line, "aspect_ratio", 12) || !strncmp(line, "spacing", 7))
      {
        double ar[3];
        if (!(this->Read(ar) && this->Read(ar + 1) && this->Read(ar + 2)))
        {
          vtkErrorMacro(<< "Error reading spacing!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }
        metadata->Set(vtkDataObject::SPACING(), ar, 3);
        arRead = 1;
      }

      else if (!strncmp(line, "origin", 6))
      {
        double origin[3];
        if (!(this->Read(origin) && this->Read(origin + 1) && this->Read(origin + 2)))
        {
          vtkErrorMacro(<< "Error reading origin!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }
        metadata->Set(vtkDataObject::ORIGIN(), origin, 3);
        originRead = 1;
      }

      else if (!strncmp(line, "point_data", 10))
      {
        vtkIdType npts;
        if (!this->Read(&npts))
        {
          vtkErrorMacro(<< "Cannot read point data!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        while (this->ReadString(line))
        {
          // read scalar data
          //
          if (!strncmp(this->LowerCase(line), "scalars", 7))
          {
            int scalarType = VTK_DOUBLE;
            this->ReadString(line);
            this->ReadString(line);

            if (!strncmp(line, "bit", 3))
            {
              scalarType = VTK_BIT;
            }
            else if (!strncmp(line, "char", 4))
            {
              scalarType = VTK_CHAR;
            }
            else if (!strncmp(line, "unsigned_char", 13))
            {
              scalarType = VTK_UNSIGNED_CHAR;
            }
            else if (!strncmp(line, "short", 5))
            {
              scalarType = VTK_SHORT;
            }
            else if (!strncmp(line, "unsigned_short", 14))
            {
              scalarType = VTK_UNSIGNED_SHORT;
            }
            else if (!strncmp(line, "int", 3))
            {
              scalarType = VTK_INT;
            }
            else if (!strncmp(line, "unsigned_int", 12))
            {
              scalarType = VTK_UNSIGNED_INT;
            }
            else if (!strncmp(line, "long", 4))
            {
              scalarType = VTK_LONG;
            }
            else if (!strncmp(line, "unsigned_long", 13))
            {
              scalarType = VTK_UNSIGNED_LONG;
            }
            else if (!strncmp(line, "float", 5))
            {
              scalarType = VTK_FLOAT;
            }
            else if (!strncmp(line, "double", 6))
            {
              scalarType = VTK_DOUBLE;
            }

            // the next string could be an integer number of components or a
            // lookup table
            this->ReadString(line);
            int numComp;
            if (strcmp(this->LowerCase(line), "lookup_table") != 0)
            {
              numComp = atoi(line);
              if (numComp < 1 || !this->ReadString(line))
              {
                vtkErrorMacro(<< "Cannot read scalar header!"
                              << " for file: " << fname);
                return 1;
              }
            }
            else
            {
              numComp = 1;
            }

            vtkDataObject::SetPointDataActiveScalarInfo(metadata, scalarType, numComp);
            break;
          }
          else if (!strncmp(this->LowerCase(line), "color_scalars", 13))
          {
            this->ReadString(line);
            this->ReadString(line);
            int numComp = atoi(line);
            if (numComp < 1)
            {
              vtkErrorMacro("Cannot read color_scalar header!"
                << " for file: " << fname);
              return 1;
            }

            // Color scalar type is predefined by FileType.
            int scalarType;
            if (this->FileType == VTK_BINARY)
            {
              scalarType = VTK_UNSIGNED_CHAR;
            }
            else
            {
              scalarType = VTK_FLOAT;
            }

            vtkDataObject::SetPointDataActiveScalarInfo(metadata, scalarType, numComp);
            break;
          }
        }
        break; // out of this loop
      }
    }

    if (!dimsRead || !arRead || !originRead)
    {
      vtkWarningMacro(<< "Not all meta data was read from the file.");
    }
  }

  this->CloseVTKFile();

  return 1;
}

int vtkStructuredPointsReader::ReadMeshSimple(const std::string& fname, vtkDataObject* doOutput)
{
  this->SetErrorCode(vtkErrorCode::NoError);
  vtkIdType numPts = 0, numCells = 0;
  char line[256];
  vtkIdType npts, ncells;
  int dimsRead = 0, arRead = 0, originRead = 0;
  vtkStructuredPoints* output = vtkStructuredPoints::SafeDownCast(doOutput);

  // ImageSource superclass does not do this.
  output->ReleaseData();

  vtkDebugMacro(<< "Reading vtk structured points file...");

  if (!this->OpenVTKFile(fname.c_str()) || !this->ReadHeader(fname.c_str()))
  {
    return 1;
  }

  // Read structured points specific stuff
  //
  if (!this->ReadString(line))
  {
    vtkErrorMacro(<< "Data file ends prematurely!");
    this->CloseVTKFile();
    this->SetErrorCode(vtkErrorCode::PrematureEndOfFileError);
    return 1;
  }

  if (!strncmp(this->LowerCase(line), "dataset", 7))
  {
    // Make sure we're reading right type of geometry
    //
    if (!this->ReadString(line))
    {
      vtkErrorMacro(<< "Data file ends prematurely!");
      this->CloseVTKFile();
      this->SetErrorCode(vtkErrorCode::PrematureEndOfFileError);
      return 1;
    }

    if (strncmp(this->LowerCase(line), "structured_points", 17) != 0)
    {
      vtkErrorMacro(<< "Cannot read dataset type: " << line);
      this->CloseVTKFile();
      this->SetErrorCode(vtkErrorCode::UnrecognizedFileTypeError);
      return 1;
    }

    // Read keyword and number of points
    //
    numPts = output->GetNumberOfPoints(); // get default
    while (true)
    {
      if (!this->ReadString(line))
      {
        break;
      }

      if (!strncmp(this->LowerCase(line), "field", 5))
      {
        vtkFieldData* fd = this->ReadFieldData();
        output->SetFieldData(fd);
        fd->Delete(); // ?
      }
      else if (!strncmp(line, "extent", 6) && !dimsRead)
      {
        int extent[6];
        if (!(this->Read(extent) && this->Read(extent + 1) && this->Read(extent + 2) &&
              this->Read(extent + 3) && this->Read(extent + 4) && this->Read(extent + 5)))
        {
          vtkErrorMacro(<< "Error reading extent!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        output->SetExtent(extent);
        numPts = output->GetNumberOfPoints();
        numCells = output->GetNumberOfCells();
        dimsRead = 1;
      }
      else if (!strncmp(line, "dimensions", 10))
      {
        int dim[3];
        if (!(this->Read(dim) && this->Read(dim + 1) && this->Read(dim + 2)))
        {
          vtkErrorMacro(<< "Error reading dimensions!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        numPts = static_cast<vtkIdType>(dim[0]) * static_cast<vtkIdType>(dim[1]) *
          static_cast<vtkIdType>(dim[2]);
        output->SetDimensions(dim);
        numCells = output->GetNumberOfCells();
        dimsRead = 1;
      }

      else if (!strncmp(line, "aspect_ratio", 12) || !strncmp(line, "spacing", 7))
      {
        double ar[3];
        if (!(this->Read(ar) && this->Read(ar + 1) && this->Read(ar + 2)))
        {
          vtkErrorMacro(<< "Error reading spacing!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        output->SetSpacing(ar);
        arRead = 1;
      }

      else if (!strncmp(line, "origin", 6))
      {
        double origin[3];
        if (!(this->Read(origin) && this->Read(origin + 1) && this->Read(origin + 2)))
        {
          vtkErrorMacro(<< "Error reading origin!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        output->SetOrigin(origin);
        originRead = 1;
      }

      else if (!strncmp(line, "cell_data", 9))
      {
        if (!this->Read(&ncells))
        {
          vtkErrorMacro(<< "Cannot read cell data!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        if (ncells != numCells)
        {
          vtkErrorMacro(<< "Number of cells don't match data values!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        this->ReadCellData(output, ncells);
        break; // out of this loop
      }

      else if (!strncmp(line, "point_data", 10))
      {
        if (!this->Read(&npts))
        {
          vtkErrorMacro(<< "Cannot read point data!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        if (npts != numPts)
        {
          vtkErrorMacro(<< "Number of points don't match data values!");
          this->CloseVTKFile();
          this->SetErrorCode(vtkErrorCode::FileFormatError);
          return 1;
        }

        this->ReadPointData(output, npts);
        break; // out of this loop
      }

      else
      {
        vtkErrorMacro(<< "Unrecognized keyword: " << line);
        this->CloseVTKFile();
        this->SetErrorCode(vtkErrorCode::FileFormatError);
        return 1;
      }
    }

    if (!dimsRead)
      vtkWarningMacro(<< "No dimensions read.");
    if (!arRead)
      vtkWarningMacro(<< "No spacing read.");
    if (!originRead)
      vtkWarningMacro(<< "No origin read.");
  }

  else if (!strncmp(line, "cell_data", 9))
  {
    vtkWarningMacro(<< "No geometry defined in data file!");
    if (!this->Read(&ncells))
    {
      vtkErrorMacro(<< "Cannot read cell data!");
      this->CloseVTKFile();
      this->SetErrorCode(vtkErrorCode::FileFormatError);
      return 1;
    }
    this->ReadCellData(output, numCells);
  }

  else if (!strncmp(line, "point_data", 10))
  {
    vtkWarningMacro(<< "No geometry defined in data file!");
    if (!this->Read(&npts))
    {
      vtkErrorMacro(<< "Cannot read point data!");
      this->CloseVTKFile();
      this->SetErrorCode(vtkErrorCode::FileFormatError);
      return 1;
    }
    this->ReadPointData(output, numPts);
  }

  else
  {
    vtkErrorMacro(<< "Unrecognized keyword: " << line);
  }
  this->CloseVTKFile();

  return 1;
}

int vtkStructuredPointsReader::FillOutputPortInformation(int, vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkStructuredPoints");
  return 1;
}

void vtkStructuredPointsReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
