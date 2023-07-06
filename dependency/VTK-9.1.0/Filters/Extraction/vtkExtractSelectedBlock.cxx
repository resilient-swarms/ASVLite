/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkExtractSelectedBlock.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkExtractSelectedBlock.h"

#include "vtkArrayDispatch.h"
#include "vtkDataArrayRange.h"
#include "vtkDataObjectTreeIterator.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkObjectFactory.h"
#include "vtkSelection.h"
#include "vtkSelectionNode.h"
#include "vtkUnsignedIntArray.h"

#include <unordered_set>
vtkStandardNewMacro(vtkExtractSelectedBlock);
//------------------------------------------------------------------------------
vtkExtractSelectedBlock::vtkExtractSelectedBlock() = default;

//------------------------------------------------------------------------------
vtkExtractSelectedBlock::~vtkExtractSelectedBlock() = default;

//------------------------------------------------------------------------------
int vtkExtractSelectedBlock::FillInputPortInformation(int port, vtkInformation* info)
{
  this->Superclass::FillInputPortInformation(port, info);

  // now add our info
  if (port == 0)
  {
    // Can work with composite datasets.
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkDataObject");
  }

  return 1;
}

//------------------------------------------------------------------------------
// Needed because parent class sets output type to input type
// and we sometimes want to change it to make an UnstructuredGrid regardless of
// input type
int vtkExtractSelectedBlock::RequestDataObject(
  vtkInformation* req, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  if (!inInfo)
  {
    return 0;
  }

  vtkCompositeDataSet* input = vtkCompositeDataSet::GetData(inInfo);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  if (input)
  {
    vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::GetData(outInfo);
    if (!output)
    {
      output = vtkMultiBlockDataSet::New();
      outInfo->Set(vtkDataObject::DATA_OBJECT(), output);
      output->Delete();
    }
    return 1;
  }

  return this->Superclass::RequestDataObject(req, inputVector, outputVector);
}

namespace
{
/**
 * Copies subtree and removes ids for subtree from `ids`.
 */
void vtkCopySubTree(std::unordered_set<unsigned int>& ids, vtkCompositeDataIterator* loc,
  vtkCompositeDataSet* output, vtkCompositeDataSet* input)
{
  vtkDataObject* inputNode = input->GetDataSet(loc);
  if (vtkCompositeDataSet* cinput = vtkCompositeDataSet::SafeDownCast(inputNode))
  {
    vtkCompositeDataSet* coutput = vtkCompositeDataSet::SafeDownCast(output->GetDataSet(loc));
    assert(coutput != nullptr);

    // shallow copy..this pass the non-leaf nodes over.
    coutput->ShallowCopy(cinput);

    // now, we need to remove all composite ids for the subtree from the set to
    // extract to avoid attempting to copy them multiple times (although it
    // should not be harmful at all).

    vtkCompositeDataIterator* iter = cinput->NewIterator();
    if (vtkDataObjectTreeIterator* treeIter = vtkDataObjectTreeIterator::SafeDownCast(iter))
    {
      treeIter->VisitOnlyLeavesOff();
    }
    for (iter->InitTraversal(); !iter->IsDoneWithTraversal(); iter->GoToNextItem())
    {
      ids.erase(loc->GetCurrentFlatIndex() + iter->GetCurrentFlatIndex());
    }
    iter->Delete();
  }
  else
  {
    output->SetDataSet(loc, inputNode);
  }
  ids.erase(loc->GetCurrentFlatIndex());
}
}

namespace
{
struct SelectionToIds
{
  template <typename ArrayT>
  void operator()(ArrayT* array, std::unordered_set<unsigned int>& blocks) const
  {
    for (auto value : vtk::DataArrayValueRange(array))
    {
      blocks.insert(static_cast<unsigned int>(value));
    }
  }
};
} // namespace

//------------------------------------------------------------------------------
int vtkExtractSelectedBlock::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // get the info objects
  vtkInformation* selInfo = inputVector[1]->GetInformationObject(0);
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  vtkCompositeDataSet* cd = vtkCompositeDataSet::GetData(inInfo);
  if (!cd)
  {
    vtkDataObject* outputDO = vtkDataObject::GetData(outInfo);
    outputDO->ShallowCopy(vtkDataObject::GetData(inInfo));
    return 1;
  }

  if (!selInfo)
  {
    // When not given a selection, quietly select nothing.
    return 1;
  }

  vtkSelection* input = vtkSelection::GetData(selInfo);
  vtkSelectionNode* node = input->GetNode(0);
  if (input->GetNumberOfNodes() != 1 || node->GetContentType() != vtkSelectionNode::BLOCKS)
  {
    vtkErrorMacro("This filter expects a single-node selection of type BLOCKS.");
    return 0;
  }

  bool inverse = (node->GetProperties()->Has(vtkSelectionNode::INVERSE()) &&
    node->GetProperties()->Get(vtkSelectionNode::INVERSE()) == 1);

  vtkDataArray* selectionList = vtkArrayDownCast<vtkDataArray>(node->GetSelectionList());
  std::unordered_set<unsigned int> blocks;
  if (selectionList)
  {
    using Dispatcher = vtkArrayDispatch::DispatchByValueType<vtkArrayDispatch::Integrals>;
    if (!Dispatcher::Execute(selectionList, SelectionToIds{}, blocks))
    { // fallback for unsupported array types
      // and non-integral value types:
      SelectionToIds{}(selectionList, blocks);
    }
  }

  vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::GetData(outInfo);

  // short-circuit if root index is present.
  const bool has_root = (blocks.find(0) != blocks.end());
  if (has_root && !inverse)
  {
    // pass everything.
    output->ShallowCopy(cd);
    return 1;
  }

  if (has_root && inverse)
  {
    // pass nothing.
    output->CopyStructure(cd);
    return 1;
  }

  // pass selected ids (or invert)
  output->CopyStructure(cd);

  vtkCompositeDataIterator* citer = cd->NewIterator();
  if (vtkDataObjectTreeIterator* diter = vtkDataObjectTreeIterator::SafeDownCast(citer))
  {
    diter->VisitOnlyLeavesOff();
  }

  for (citer->InitTraversal(); !citer->IsDoneWithTraversal(); citer->GoToNextItem())
  {
    auto fiter = blocks.find(citer->GetCurrentFlatIndex());
    if ((inverse && fiter == blocks.end()) || (!inverse && fiter != blocks.end()))
    {
      vtkCopySubTree(blocks, citer, output, cd);
    }
  }
  citer->Delete();
  return 1;
}

//------------------------------------------------------------------------------
void vtkExtractSelectedBlock::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
