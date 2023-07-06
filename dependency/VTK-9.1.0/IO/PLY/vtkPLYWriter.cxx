/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPLYWriter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPLYWriter.h"

#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkFloatArray.h"
#include "vtkInformation.h"
#include "vtkObjectFactory.h"
#include "vtkPLY.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkScalarsToColors.h"
#include "vtkStringArray.h"
#include "vtkUnsignedCharArray.h"

#include <cstddef>

vtkStandardNewMacro(vtkPLYWriter);

vtkCxxSetObjectMacro(vtkPLYWriter, LookupTable, vtkScalarsToColors);

vtkPLYWriter::vtkPLYWriter()
{
  this->FileName = nullptr;
  this->FileType = VTK_BINARY;
  this->DataByteOrder = VTK_LITTLE_ENDIAN;
  this->ArrayName = nullptr;
  this->Component = 0;
  this->ColorMode = VTK_COLOR_MODE_DEFAULT;
  this->LookupTable = nullptr;
  this->Color[0] = this->Color[1] = this->Color[2] = 255;
  this->EnableAlpha = false;
  this->Alpha = 255;
  this->TextureCoordinatesName = VTK_TEXTURECOORDS_UV;

  this->HeaderComments = vtkSmartPointer<vtkStringArray>::New();
  this->HeaderComments->InsertNextValue("VTK generated PLY File");
  this->WriteToOutputString = false;
}

vtkPLYWriter::~vtkPLYWriter()
{
  this->SetLookupTable(nullptr);
  delete[] this->ArrayName;
  delete[] this->FileName;
}

typedef struct
{
  float x[3]; // the usual 3-space position of a vertex
  unsigned char red;
  unsigned char green;
  unsigned char blue;
  unsigned char alpha;
  float tex[2];
} plyVertex;

typedef struct
{
  unsigned char nverts; // number of vertex indices in list
  int* verts;           // vertex index list
  unsigned char red;
  unsigned char green;
  unsigned char blue;
  unsigned char alpha;
} plyFace;

void vtkPLYWriter::WriteData()
{
  vtkIdType i, j, idx;
  vtkPoints* inPts;
  vtkCellArray* polys;
  vtkPolyData* input = this->GetInput();

  vtkSmartPointer<vtkUnsignedCharArray> cellColors, pointColors;
  PlyFile* ply;
  static const char* elemNames[] = { "vertex", "face" };
  static PlyProperty vertProps[] = {
    // property information for a vertex
    { "x", PLY_FLOAT, PLY_FLOAT, static_cast<int>(offsetof(plyVertex, x)), 0, 0, 0, 0 },
    { "y", PLY_FLOAT, PLY_FLOAT, static_cast<int>(offsetof(plyVertex, x) + sizeof(float)), 0, 0, 0,
      0 },
    { "z", PLY_FLOAT, PLY_FLOAT,
      static_cast<int>(offsetof(plyVertex, x) + sizeof(float) + sizeof(float)), 0, 0, 0, 0 },
    { "red", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, red)), 0, 0, 0, 0 },
    { "green", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, green)), 0, 0, 0, 0 },
    { "blue", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, blue)), 0, 0, 0, 0 },
    { "alpha", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, alpha)), 0, 0, 0, 0 },
    { (TextureCoordinatesName == 1) ? "texture_u" : "u", PLY_FLOAT, PLY_FLOAT,
      static_cast<int>(offsetof(plyVertex, tex)), 0, 0, 0, 0 },
    { (TextureCoordinatesName == 1) ? "texture_v" : "v", PLY_FLOAT, PLY_FLOAT,
      static_cast<int>(offsetof(plyVertex, tex) + sizeof(float)), 0, 0, 0, 0 },
  };
  static PlyProperty faceProps[] = {
    // property information for a face
    { "vertex_indices", PLY_INT, PLY_INT, static_cast<int>(offsetof(plyFace, verts)), 1, PLY_UCHAR,
      PLY_UCHAR, static_cast<int>(offsetof(plyFace, nverts)) },
    { "red", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, red)), 0, 0, 0, 0 },
    { "green", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, green)), 0, 0, 0, 0 },
    { "blue", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, blue)), 0, 0, 0, 0 },
    { "alpha", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, alpha)), 0, 0, 0, 0 },
  };

  // Get input and check data
  polys = input->GetPolys();
  inPts = input->GetPoints();
  if (inPts == nullptr || polys == nullptr)
  {
    vtkErrorMacro(<< "No data to write!");
    return;
  }

  // Open the file in appropriate way
  if (this->WriteToOutputString)
  {
    ply = vtkPLY::ply_open_for_writing_to_string(this->OutputString, 2, elemNames,
      this->FileType == VTK_BINARY
        ? (this->DataByteOrder == VTK_LITTLE_ENDIAN ? PLY_BINARY_LE : PLY_BINARY_BE)
        : PLY_ASCII);
  }
  else
  {
    ply = vtkPLY::ply_open_for_writing(this->FileName, 2, elemNames,
      this->FileType == VTK_BINARY
        ? (this->DataByteOrder == VTK_LITTLE_ENDIAN ? PLY_BINARY_LE : PLY_BINARY_BE)
        : PLY_ASCII);
  }

  if (ply == nullptr)
  {
    vtkErrorMacro(<< "Error opening PLY file");
    return;
  }

  // compute colors, if any
  vtkIdType numPts = inPts->GetNumberOfPoints();
  vtkIdType numPolys = polys->GetNumberOfCells();
  pointColors = this->GetColors(numPts, input->GetPointData());
  cellColors = this->GetColors(numPolys, input->GetCellData());

  bool pointAlpha = pointColors && pointColors->GetNumberOfComponents() == 4;
  bool cellAlpha = cellColors && cellColors->GetNumberOfComponents() == 4;

  // get texture coordinates, if any
  const float* textureCoords = this->GetTextureCoordinates(numPts, input->GetPointData());

  // describe what properties go into the vertex and face elements
  vtkPLY::ply_element_count(ply, "vertex", numPts);
  vtkPLY::ply_describe_property(ply, "vertex", &vertProps[0]);
  vtkPLY::ply_describe_property(ply, "vertex", &vertProps[1]);
  vtkPLY::ply_describe_property(ply, "vertex", &vertProps[2]);
  if (pointColors)
  {
    vtkPLY::ply_describe_property(ply, "vertex", &vertProps[3]);
    vtkPLY::ply_describe_property(ply, "vertex", &vertProps[4]);
    vtkPLY::ply_describe_property(ply, "vertex", &vertProps[5]);
    if (pointAlpha)
    {
      vtkPLY::ply_describe_property(ply, "vertex", &vertProps[6]);
    }
  }
  if (textureCoords)
  {
    vtkPLY::ply_describe_property(ply, "vertex", &vertProps[7]);
    vtkPLY::ply_describe_property(ply, "vertex", &vertProps[8]);
  }

  vtkPLY::ply_element_count(ply, "face", numPolys);
  vtkPLY::ply_describe_property(ply, "face", &faceProps[0]);
  if (cellColors)
  {
    vtkPLY::ply_describe_property(ply, "face", &faceProps[1]);
    vtkPLY::ply_describe_property(ply, "face", &faceProps[2]);
    vtkPLY::ply_describe_property(ply, "face", &faceProps[3]);
    if (cellAlpha)
    {
      vtkPLY::ply_describe_property(ply, "face", &faceProps[4]);
    }
  }

  // write comments and an object information field
  for (idx = 0; idx < this->HeaderComments->GetNumberOfValues(); ++idx)
  {
    vtkPLY::ply_put_comment(ply, this->HeaderComments->GetValue(idx));
  }
  vtkPLY::ply_put_obj_info(ply, "vtkPolyData points and polygons: vtk4.0");

  // complete the header
  vtkPLY::ply_header_complete(ply);

  // set up and write the vertex elements
  plyVertex vert;
  vtkPLY::ply_put_element_setup(ply, "vertex");
  double dpoint[3];
  for (i = 0; i < numPts; i++)
  {
    inPts->GetPoint(i, dpoint);
    vert.x[0] = static_cast<float>(dpoint[0]);
    vert.x[1] = static_cast<float>(dpoint[1]);
    vert.x[2] = static_cast<float>(dpoint[2]);
    if (pointColors)
    {
      idx = pointAlpha ? 4 * i : 3 * i;
      vert.red = pointColors->GetValue(idx);
      vert.green = pointColors->GetValue(idx + 1);
      vert.blue = pointColors->GetValue(idx + 2);
      if (pointAlpha)
      {
        vert.alpha = pointColors->GetValue(idx + 3);
      }
    }
    if (textureCoords)
    {
      idx = 2 * i;
      vert.tex[0] = *(textureCoords + idx);
      vert.tex[1] = *(textureCoords + idx + 1);
    }
    vtkPLY::ply_put_element(ply, (void*)&vert);
  }

  // set up and write the face elements
  plyFace face;
  int verts[256];
  face.verts = verts;
  vtkPLY::ply_put_element_setup(ply, "face");
  vtkIdType npts = 0;
  const vtkIdType* pts = nullptr;
  for (polys->InitTraversal(), i = 0; i < numPolys; i++)
  {
    polys->GetNextCell(npts, pts);
    if (npts > 256)
    {
      vtkErrorMacro(<< "Ply file only supports polygons with <256 points");
    }
    else
    {
      for (j = 0; j < npts; j++)
      {
        face.nverts = npts;
        verts[j] = (int)pts[j];
      }
      if (cellColors)
      {
        idx = cellAlpha ? 4 * i : 3 * i;
        face.red = cellColors->GetValue(idx);
        face.green = cellColors->GetValue(idx + 1);
        face.blue = cellColors->GetValue(idx + 2);
        if (cellAlpha)
        {
          face.alpha = cellColors->GetValue(idx + 3);
        }
      }
      vtkPLY::ply_put_element(ply, (void*)&face);
    }
  } // for all polygons

  // close the PLY file
  vtkPLY::ply_close(ply);
}

vtkSmartPointer<vtkUnsignedCharArray> vtkPLYWriter::GetColors(
  vtkIdType num, vtkDataSetAttributes* dsa)
{
  unsigned char* c;
  vtkIdType i;
  int numComp;

  if (this->ColorMode == VTK_COLOR_MODE_OFF ||
    (this->ColorMode == VTK_COLOR_MODE_UNIFORM_CELL_COLOR &&
      vtkPointData::SafeDownCast(dsa) != nullptr) ||
    (this->ColorMode == VTK_COLOR_MODE_UNIFORM_POINT_COLOR &&
      vtkCellData::SafeDownCast(dsa) != nullptr))
  {
    return nullptr;
  }
  else if (this->ColorMode == VTK_COLOR_MODE_UNIFORM_COLOR ||
    this->ColorMode == VTK_COLOR_MODE_UNIFORM_POINT_COLOR ||
    this->ColorMode == VTK_COLOR_MODE_UNIFORM_CELL_COLOR)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(this->EnableAlpha ? 4 : 3);
    colors->SetNumberOfTuples(num);
    c = colors->WritePointer(0, this->EnableAlpha ? 4 * num : 3 * num);
    if (this->EnableAlpha)
    {
      for (i = 0; i < num; i++)
      {
        *c++ = this->Color[0];
        *c++ = this->Color[1];
        *c++ = this->Color[2];
        *c++ = this->Alpha;
      }
    }
    else
    {
      for (i = 0; i < num; i++)
      {
        *c++ = this->Color[0];
        *c++ = this->Color[1];
        *c++ = this->Color[2];
      }
    }
    return colors;
  }
  else // we will color based on data
  {
    vtkDataArray* da;
    vtkUnsignedCharArray* rgbArray;

    if (!this->ArrayName || (da = dsa->GetArray(this->ArrayName)) == nullptr ||
      this->Component >= (numComp = da->GetNumberOfComponents()))
    {
      return nullptr;
    }
    else if ((rgbArray = vtkArrayDownCast<vtkUnsignedCharArray>(da)) != nullptr && numComp == 3)
    { // have unsigned char array of three components, copy it
      return rgbArray;
    }
    else if ((rgbArray = vtkArrayDownCast<vtkUnsignedCharArray>(da)) != nullptr && numComp == 4)
    {
      if (this->EnableAlpha)
      {
        return rgbArray;
      }

      // have unsigned char array of four components (RGBA), copy it without the `A`.
      vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
      colors->SetNumberOfComponents(3);
      colors->SetNumberOfTuples(num);
      c = colors->WritePointer(0, 3 * num);
      const unsigned char* rgba = rgbArray->GetPointer(0);
      for (i = 0; i < num; i++)
      {
        *c++ = *rgba++;
        *c++ = *rgba++;
        *c++ = *rgba++;
        rgba++;
      }
      return colors;
    }
    else if (this->LookupTable != nullptr)
    { // use the data array mapped through lookup table
      vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
      colors->SetNumberOfComponents(this->EnableAlpha ? 4 : 3);
      colors->SetNumberOfTuples(num);
      c = colors->WritePointer(0, this->EnableAlpha ? 4 * num : 3 * num);
      if (this->EnableAlpha)
      {
        for (i = 0; i < num; i++)
        {
          double* tuple = da->GetTuple(i);
          const unsigned char* rgba = this->LookupTable->MapValue(tuple[this->Component]);
          *c++ = rgba[0];
          *c++ = rgba[1];
          *c++ = rgba[2];
          *c++ = rgba[3];
        }
      }
      else
      {
        for (i = 0; i < num; i++)
        {
          double* tuple = da->GetTuple(i);
          const unsigned char* rgb = this->LookupTable->MapValue(tuple[this->Component]);
          *c++ = rgb[0];
          *c++ = rgb[1];
          *c++ = rgb[2];
        }
      }
      return colors;
    }
    else // no lookup table
    {
      return nullptr;
    }
  }
}

const float* vtkPLYWriter::GetTextureCoordinates(vtkIdType num, vtkDataSetAttributes* dsa)
{
  vtkDataArray* tCoords = dsa->GetTCoords();
  if (!tCoords || (tCoords->GetNumberOfTuples() != num) || (tCoords->GetNumberOfComponents() != 2))
    return nullptr;

  vtkFloatArray* textureArray;
  if ((textureArray = vtkArrayDownCast<vtkFloatArray>(tCoords)) == nullptr)
    vtkErrorMacro(<< "PLY writer only supports float texture coordinates");

  return textureArray->GetPointer(0);
}

void vtkPLYWriter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Data Byte Order: ";
  if (this->DataByteOrder == VTK_LITTLE_ENDIAN)
  {
    os << "Little Endian\n";
  }
  else
  {
    os << "Big Endian\n";
  }

  os << indent << "Color Mode: ";
  if (this->ColorMode == VTK_COLOR_MODE_DEFAULT)
  {
    os << "Default\n";
  }
  else if (this->ColorMode == VTK_COLOR_MODE_UNIFORM_CELL_COLOR)
  {
    os << "Uniform Cell Color\n";
  }
  else if (this->ColorMode == VTK_COLOR_MODE_UNIFORM_POINT_COLOR)
  {
    os << "Uniform Point Color\n";
  }
  else if (this->ColorMode == VTK_COLOR_MODE_UNIFORM_COLOR)
  {
    os << "Uniform Color\n";
  }
  else // VTK_COLOR_MODE_OFF
  {
    os << "Off\n";
  }

  os << indent << "Array Name: " << (this->ArrayName ? this->ArrayName : "(none)") << "\n";

  os << indent << "Component: " << this->Component << "\n";

  os << indent << "Lookup Table: " << this->LookupTable << "\n";

  os << indent << "Color: (" << this->Color[0] << "," << this->Color[1] << "," << this->Color[2]
     << ")\n";

  os << indent << "EnableAlpha: " << this->EnableAlpha << "\n";
  os << indent << "Alpha: " << static_cast<int>(this->Alpha) << "\n";
}

void vtkPLYWriter::AddComment(const std::string& comment)
{
  this->HeaderComments->InsertNextValue(comment.c_str());
}

vtkPolyData* vtkPLYWriter::GetInput()
{
  return vtkPolyData::SafeDownCast(this->Superclass::GetInput());
}

vtkPolyData* vtkPLYWriter::GetInput(int port)
{
  return vtkPolyData::SafeDownCast(this->Superclass::GetInput(port));
}

int vtkPLYWriter::FillInputPortInformation(int, vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  return 1;
}
