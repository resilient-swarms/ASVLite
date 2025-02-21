/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTreeReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTreeReader.h"

#include "vtkByteSwap.h"
#include "vtkCellArray.h"
#include "vtkFieldData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMutableDirectedGraph.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkTree.h"

vtkStandardNewMacro(vtkTreeReader);

#ifdef read
#undef read
#endif

//------------------------------------------------------------------------------
vtkTreeReader::vtkTreeReader() = default;

//------------------------------------------------------------------------------
vtkTreeReader::~vtkTreeReader() = default;

//------------------------------------------------------------------------------
vtkTree* vtkTreeReader::GetOutput()
{
  return this->GetOutput(0);
}

//------------------------------------------------------------------------------
vtkTree* vtkTreeReader::GetOutput(int idx)
{
  return vtkTree::SafeDownCast(this->GetOutputDataObject(idx));
}

//------------------------------------------------------------------------------
void vtkTreeReader::SetOutput(vtkTree* output)
{
  this->GetExecutive()->SetOutputData(0, output);
}

//------------------------------------------------------------------------------
int vtkTreeReader::ReadMeshSimple(const std::string& fname, vtkDataObject* doOutput)
{
  vtkDebugMacro(<< "Reading vtk tree ...");

  if (!this->OpenVTKFile(fname.c_str()) || !this->ReadHeader())
  {
    return 1;
  }

  // Read table-specific stuff
  char line[256];
  if (!this->ReadString(line))
  {
    vtkErrorMacro(<< "Data file ends prematurely!");
    this->CloseVTKFile();
    return 1;
  }

  if (strncmp(this->LowerCase(line), "dataset", 7) != 0)
  {
    vtkErrorMacro(<< "Unrecognized keyword: " << line);
    this->CloseVTKFile();
    return 1;
  }

  if (!this->ReadString(line))
  {
    vtkErrorMacro(<< "Data file ends prematurely!");
    this->CloseVTKFile();
    return 1;
  }

  if (strncmp(this->LowerCase(line), "tree", 4) != 0)
  {
    vtkErrorMacro(<< "Cannot read dataset type: " << line);
    this->CloseVTKFile();
    return 1;
  }

  vtkTree* const output = vtkTree::SafeDownCast(doOutput);

  vtkSmartPointer<vtkMutableDirectedGraph> builder =
    vtkSmartPointer<vtkMutableDirectedGraph>::New();

  while (true)
  {
    if (!this->ReadString(line))
    {
      break;
    }

    if (!strncmp(this->LowerCase(line), "field", 5))
    {
      vtkFieldData* const field_data = this->ReadFieldData();
      builder->SetFieldData(field_data);
      field_data->Delete();
      continue;
    }

    if (!strncmp(this->LowerCase(line), "points", 6))
    {
      vtkIdType point_count = 0;
      if (!this->Read(&point_count))
      {
        vtkErrorMacro(<< "Cannot read number of points!");
        this->CloseVTKFile();
        return 1;
      }

      this->ReadPointCoordinates(builder, point_count);
      continue;
    }

    if (!strncmp(this->LowerCase(line), "edges", 5))
    {
      vtkIdType edge_count = 0;
      if (!this->Read(&edge_count))
      {
        vtkErrorMacro(<< "Cannot read number of edges!");
        this->CloseVTKFile();
        return 1;
      }

      // Create all of the tree vertices (number of edges + 1)
      for (vtkIdType edge = 0; edge <= edge_count; ++edge)
      {
        builder->AddVertex();
      }

      // Reparent the existing vertices so their order and topology match the original
      vtkIdType child = 0;
      vtkIdType parent = 0;
      for (vtkIdType edge = 0; edge != edge_count; ++edge)
      {
        if (!(this->Read(&child) && this->Read(&parent)))
        {
          vtkErrorMacro(<< "Cannot read edge!");
          this->CloseVTKFile();
          return 1;
        }

        builder->AddEdge(parent, child);
      }

      if (!output->CheckedShallowCopy(builder))
      {
        vtkErrorMacro(<< "Edges do not create a valid tree.");
        this->CloseVTKFile();
        return 1;
      }

      continue;
    }

    if (!strncmp(this->LowerCase(line), "vertex_data", 10))
    {
      vtkIdType vertex_count = 0;
      if (!this->Read(&vertex_count))
      {
        vtkErrorMacro(<< "Cannot read number of vertices!");
        this->CloseVTKFile();
        return 1;
      }

      this->ReadVertexData(output, vertex_count);
      continue;
    }

    if (!strncmp(this->LowerCase(line), "edge_data", 9))
    {
      vtkIdType edge_count = 0;
      if (!this->Read(&edge_count))
      {
        vtkErrorMacro(<< "Cannot read number of edges!");
        this->CloseVTKFile();
        return 1;
      }

      this->ReadEdgeData(output, edge_count);
      continue;
    }

    vtkErrorMacro(<< "Unrecognized keyword: " << line);
  }

  vtkDebugMacro(<< "Read " << output->GetNumberOfVertices() << " vertices and "
                << output->GetNumberOfEdges() << " edges.\n");

  this->CloseVTKFile();

  return 1;
}

//------------------------------------------------------------------------------
int vtkTreeReader::FillOutputPortInformation(int, vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTree");
  return 1;
}

//------------------------------------------------------------------------------
void vtkTreeReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
