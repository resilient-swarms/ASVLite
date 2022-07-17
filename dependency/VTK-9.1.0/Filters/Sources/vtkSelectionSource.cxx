/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkSelectionSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkSelectionSource.h"

#include "vtkCommand.h"
#include "vtkDoubleArray.h"
#include "vtkIdTypeArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkSelection.h"
#include "vtkSelectionNode.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkStringArray.h"
#include "vtkUnsignedIntArray.h"

#include <algorithm>
#include <set>
#include <vector>

vtkStandardNewMacro(vtkSelectionSource);

class vtkSelectionSource::vtkInternals
{
public:
  typedef std::set<vtkIdType> IDSetType;
  typedef std::vector<IDSetType> IDsType;
  IDsType IDs;

  typedef std::set<vtkStdString> StringIDSetType;
  typedef std::vector<StringIDSetType> StringIDsType;
  StringIDsType StringIDs;

  std::vector<double> Thresholds;
  std::vector<double> Locations;

  IDSetType Blocks;
  double Frustum[32];

  std::vector<std::string> BlockSelectors;
  std::vector<std::string> Selectors; //< Qualifiers

  vtkInternals() { std::fill_n(this->Frustum, 32, 0); }
};

//------------------------------------------------------------------------------
vtkSelectionSource::vtkSelectionSource()
  : ContentType(vtkSelectionNode::INDICES)
  , FieldType(vtkSelectionNode::CELL)
  , ContainingCells(true)
  , PreserveTopology(false)
  , Inverse(false)
  , CompositeIndex(-1)
  , HierarchicalLevel(-1)
  , HierarchicalIndex(-1)
  , ArrayName(nullptr)
  , ArrayComponent(0)
  , QueryString(nullptr)
  , NumberOfLayers(0)
  , AssemblyName(nullptr)
  , Internal(new vtkSelectionSource::vtkInternals())
{
  this->SetNumberOfInputPorts(0);
}

//------------------------------------------------------------------------------
vtkSelectionSource::~vtkSelectionSource()
{
  delete this->Internal;
  delete[] this->ArrayName;
  delete[] this->QueryString;
  delete[] this->AssemblyName;
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllIDs()
{
  this->Internal->IDs.clear();
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllStringIDs()
{
  this->Internal->StringIDs.clear();
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllLocations()
{
  this->Internal->Locations.clear();
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllThresholds()
{
  this->Internal->Thresholds.clear();
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddID(vtkIdType proc, vtkIdType id)
{
  // proc == -1 means all processes. All other are stored at index proc+1
  proc++;

  if (proc >= (vtkIdType)this->Internal->IDs.size())
  {
    this->Internal->IDs.resize(proc + 1);
  }
  auto& idSet = this->Internal->IDs[proc];
  idSet.insert(id);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddStringID(vtkIdType proc, const char* id)
{
  // proc == -1 means all processes. All other are stored at index proc+1
  proc++;

  if (proc >= (vtkIdType)this->Internal->StringIDs.size())
  {
    this->Internal->StringIDs.resize(proc + 1);
  }
  auto& idSet = this->Internal->StringIDs[proc];
  idSet.insert(id);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddLocation(double x, double y, double z)
{
  this->Internal->Locations.push_back(x);
  this->Internal->Locations.push_back(y);
  this->Internal->Locations.push_back(z);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddThreshold(double min, double max)
{
  this->Internal->Thresholds.push_back(min);
  this->Internal->Thresholds.push_back(max);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::SetFrustum(double* vertices)
{
  for (int cc = 0; cc < 32; cc++)
  {
    if (vertices[cc] != this->Internal->Frustum[cc])
    {
      memcpy(this->Internal->Frustum, vertices, 32 * sizeof(double));
      this->Modified();
      break;
    }
  }
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddBlock(vtkIdType block)
{
  this->Internal->Blocks.insert(block);
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllBlocks()
{
  this->Internal->Blocks.clear();
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddSelector(const char* selector)
{
  if (selector)
  {
    this->Internal->Selectors.emplace_back(selector);
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllSelectors()
{
  if (!this->Internal->Selectors.empty())
  {
    this->Internal->Selectors.clear();
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkSelectionSource::AddBlockSelector(const char* selector)
{
  if (selector)
  {
    this->Internal->BlockSelectors.emplace_back(selector);
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkSelectionSource::RemoveAllBlockSelectors()
{
  if (!this->Internal->BlockSelectors.empty())
  {
    this->Internal->BlockSelectors.clear();
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkSelectionSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "ContentType: " << vtkSelectionNode::GetContentTypeAsString(this->ContentType)
     << endl;
  os << indent << "FieldType: " << vtkSelectionNode::GetFieldTypeAsString(this->FieldType) << endl;
  os << indent << "ContainingCells: " << (this->ContainingCells ? "CELLS" : "POINTS") << endl;
  os << indent << "Inverse: " << this->Inverse << endl;
  os << indent << "ArrayName: " << (this->ArrayName ? this->ArrayName : "(nullptr)") << endl;
  os << indent << "ArrayComponent: " << this->ArrayComponent << endl;
  os << indent << "CompositeIndex: " << this->CompositeIndex << endl;
  os << indent << "HierarchicalLevel: " << this->HierarchicalLevel << endl;
  os << indent << "HierarchicalIndex: " << this->HierarchicalIndex << endl;
  os << indent << "QueryString: " << (this->QueryString ? this->QueryString : "(nullptr)") << endl;
  os << indent << "NumberOfLayers: " << this->NumberOfLayers << endl;
  os << indent << "AssemblyName: " << (this->AssemblyName ? this->AssemblyName : "(nullptr)")
     << endl;
}

//------------------------------------------------------------------------------
int vtkSelectionSource::RequestInformation(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(CAN_HANDLE_PIECE_REQUEST(), 1);
  return 1;
}

//------------------------------------------------------------------------------
int vtkSelectionSource::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkSelection* outputSel = vtkSelection::GetData(outputVector);
  vtkNew<vtkSelectionNode> output;
  outputSel->AddNode(output);
  vtkInformation* oProperties = output->GetProperties();

  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  int piece = 0;
  if (outInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_PIECE_NUMBER()))
  {
    piece = outInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_PIECE_NUMBER());
  }

  if (this->CompositeIndex >= 0)
  {
    oProperties->Set(vtkSelectionNode::COMPOSITE_INDEX(), this->CompositeIndex);
  }

  if (this->HierarchicalLevel >= 0 && this->HierarchicalIndex >= 0)
  {
    oProperties->Set(vtkSelectionNode::HIERARCHICAL_LEVEL(), this->HierarchicalLevel);
    oProperties->Set(vtkSelectionNode::HIERARCHICAL_INDEX(), this->HierarchicalIndex);
  }

  if (this->AssemblyName != nullptr && !this->Internal->Selectors.empty())
  {
    oProperties->Set(vtkSelectionNode::ASSEMBLY_NAME(), this->AssemblyName);
    for (auto& selector : this->Internal->Selectors)
    {
      oProperties->Append(vtkSelectionNode::SELECTORS(), selector.c_str());
    }
  }

  // First look for string ids.
  if (((this->ContentType == vtkSelectionNode::GLOBALIDS) ||
        (this->ContentType == vtkSelectionNode::PEDIGREEIDS) ||
        (this->ContentType == vtkSelectionNode::INDICES) ||
        (this->ContentType == vtkSelectionNode::VALUES)) &&
    !this->Internal->StringIDs.empty())
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);

    vtkNew<vtkStringArray> selectionList;
    output->SetSelectionList(selectionList);

    // Number of selected items common to all pieces
    vtkIdType numCommonElems = 0;
    if (!this->Internal->StringIDs.empty())
    {
      numCommonElems = static_cast<vtkIdType>(this->Internal->StringIDs[0].size());
    }
    if (piece + 1 >= (int)this->Internal->StringIDs.size() && numCommonElems == 0)
    {
      vtkDebugMacro("No selection for piece: " << piece);
    }
    else
    {
      // idx == 0 is the list for all pieces
      // idx == piece+1 is the list for the current piece
      size_t pids[2];
      pids[0] = 0;
      pids[1] = static_cast<size_t>(piece + 1);
      for (int i = 0; i < 2; i++)
      {
        size_t idx = pids[i];
        if (idx >= this->Internal->StringIDs.size())
        {
          continue;
        }

        auto& selSet = this->Internal->StringIDs[idx];

        if (!selSet.empty())
        {
          // Create the selection list
          selectionList->SetNumberOfTuples(static_cast<vtkIdType>(selSet.size()));
          // iterate over ids and insert to the selection list
          auto iter = selSet.begin();
          for (vtkIdType idx2 = 0; iter != selSet.end(); ++iter, ++idx2)
          {
            selectionList->SetValue(idx2, *iter);
          }
        }
      }
    }
  }

  // If no string ids, use integer ids.
  if (((this->ContentType == vtkSelectionNode::GLOBALIDS) ||
        (this->ContentType == vtkSelectionNode::PEDIGREEIDS) ||
        (this->ContentType == vtkSelectionNode::INDICES) ||
        (this->ContentType == vtkSelectionNode::VALUES)) &&
    this->Internal->StringIDs.empty())
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);

    vtkNew<vtkIdTypeArray> selectionList;
    output->SetSelectionList(selectionList);

    // Number of selected items common to all pieces
    vtkIdType numCommonElems = 0;
    if (!this->Internal->IDs.empty())
    {
      numCommonElems = static_cast<vtkIdType>(this->Internal->IDs[0].size());
    }
    if (piece + 1 >= (int)this->Internal->IDs.size() && numCommonElems == 0)
    {
      vtkDebugMacro("No selection for piece: " << piece);
    }
    else
    {
      // idx == 0 is the list for all pieces
      // idx == piece+1 is the list for the current piece
      size_t pids[2] = { static_cast<size_t>(0), static_cast<size_t>(piece + 1) };
      for (int i = 0; i < 2; i++)
      {
        size_t idx = pids[i];
        if (idx >= this->Internal->IDs.size())
        {
          continue;
        }

        auto& selSet = this->Internal->IDs[idx];

        if (!selSet.empty())
        {
          // Create the selection list
          selectionList->SetNumberOfTuples(static_cast<vtkIdType>(selSet.size()));
          // iterate over ids and insert to the selection list
          auto iter = selSet.begin();
          for (vtkIdType idx2 = 0; iter != selSet.end(); ++iter, ++idx2)
          {
            selectionList->SetValue(idx2, *iter);
          }
        }
      }
    }
  }

  if (this->ContentType == vtkSelectionNode::LOCATIONS)
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);
    // Create the selection list
    vtkNew<vtkDoubleArray> selectionList;
    selectionList->SetNumberOfComponents(3);
    selectionList->SetNumberOfValues(static_cast<vtkIdType>(this->Internal->Locations.size()));

    std::vector<double>::iterator iter = this->Internal->Locations.begin();
    for (vtkIdType cc = 0; iter != this->Internal->Locations.end(); ++iter, ++cc)
    {
      selectionList->SetValue(cc, *iter);
    }

    output->SetSelectionList(selectionList);
  }

  if (this->ContentType == vtkSelectionNode::THRESHOLDS)
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);
    oProperties->Set(vtkSelectionNode::COMPONENT_NUMBER(), this->ArrayComponent);
    // Create the selection list
    vtkNew<vtkDoubleArray> selectionList;
    selectionList->SetNumberOfComponents(2);
    selectionList->SetNumberOfValues(static_cast<vtkIdType>(this->Internal->Thresholds.size()));

    std::vector<double>::iterator iter = this->Internal->Thresholds.begin();
    for (vtkIdType cc = 0; iter != this->Internal->Thresholds.end(); ++iter, ++cc)
    {
      selectionList->SetTypedComponent(cc, 0, *iter);
      ++iter;
      selectionList->SetTypedComponent(cc, 1, *iter);
    }

    output->SetSelectionList(selectionList);
  }

  if (this->ContentType == vtkSelectionNode::FRUSTUM)
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);
    // Create the selection list
    vtkNew<vtkDoubleArray> selectionList;
    selectionList->SetNumberOfComponents(4);
    selectionList->SetNumberOfTuples(8);
    for (vtkIdType cc = 0; cc < 32; cc++)
    {
      selectionList->SetValue(cc, this->Internal->Frustum[cc]);
    }

    output->SetSelectionList(selectionList);
  }

  if (this->ContentType == vtkSelectionNode::BLOCKS)
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);
    vtkNew<vtkUnsignedIntArray> selectionList;
    selectionList->SetNumberOfComponents(1);
    selectionList->SetNumberOfTuples(static_cast<vtkIdType>(this->Internal->Blocks.size()));
    vtkIdType cc = 0;
    for (auto iter = this->Internal->Blocks.begin(); iter != this->Internal->Blocks.end();
         ++iter, ++cc)
    {
      selectionList->SetValue(cc, *iter);
    }
    output->SetSelectionList(selectionList);
  }

  if (this->ContentType == vtkSelectionNode::BLOCK_SELECTORS)
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);
    vtkNew<vtkStringArray> selectionList;
    selectionList->SetNumberOfTuples(static_cast<vtkIdType>(this->Internal->BlockSelectors.size()));
    vtkIdType cc = 0;
    for (const auto& selector : this->Internal->BlockSelectors)
    {
      selectionList->SetValue(cc++, selector);
    }
    output->SetSelectionList(selectionList);
  }

  if (this->ContentType == vtkSelectionNode::QUERY)
  {
    oProperties->Set(vtkSelectionNode::CONTENT_TYPE(), this->ContentType);
    oProperties->Set(vtkSelectionNode::FIELD_TYPE(), this->FieldType);
    output->SetQueryString(this->QueryString);
  }

  if (this->ContentType == vtkSelectionNode::USER)
  {
    vtkErrorMacro("User-supplied, application-specific selections are not supported.");
    return 0;
  }

  oProperties->Set(vtkSelectionNode::CONTAINING_CELLS(), this->ContainingCells);

  oProperties->Set(vtkSelectionNode::INVERSE(), this->Inverse);

  if (output->GetSelectionList())
  {
    output->GetSelectionList()->SetName(this->ArrayName);
  }
  oProperties->Set(vtkSelectionNode::CONNECTED_LAYERS(), this->NumberOfLayers);
  return 1;
}
