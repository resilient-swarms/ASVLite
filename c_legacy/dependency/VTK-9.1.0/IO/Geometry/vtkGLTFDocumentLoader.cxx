/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkGLTFDocumentLoader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkGLTFDocumentLoader.h"

#include "vtkArrayDispatch.h"
#include "vtkAssume.h"
#include "vtkBase64Utilities.h"
#include "vtkCommand.h"
#include "vtkFloatArray.h"
#include "vtkGLTFDocumentLoaderInternals.h"
#include "vtkGLTFUtils.h"
#include "vtkGenericDataArray.h"
#include "vtkIdTypeArray.h"
#include "vtkImageData.h"
#include "vtkImageReader2.h"
#include "vtkImageReader2Factory.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkIntArray.h"
#include "vtkJPEGReader.h"
#include "vtkMath.h"
#include "vtkMatrix3x3.h"
#include "vtkPNGReader.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkQuaternion.h"
#include "vtkTransform.h"
#include "vtkUnsignedShortArray.h"
#include "vtksys/FStream.hxx"
#include "vtksys/SystemTools.hxx"

#include <algorithm>
#include <limits>
#include <numeric>
#include <sstream>

// gltf uses hard coded numbers to represent data types
// they match the definitions from gl.h but are redefined below to avoid including vtkOpenGL.h
#define GL_BYTE 0x1400
#define GL_UNSIGNED_BYTE 0x1401
#define GL_SHORT 0x1402
#define GL_UNSIGNED_SHORT 0x1403
#define GL_INT 0x1404
#define GL_UNSIGNED_INT 0x1405
#define GL_FLOAT 0x1406

#define GL_CLAMP_TO_EDGE 0x812F
#define GL_REPEAT 0x2901

#define GL_NEAREST 0x2600
#define GL_LINEAR 0x2601

namespace
{
//------------------------------------------------------------------------------
// Replacement for std::to_string as it is not supported by certain compilers
template <typename T>
std::string value_to_string(const T& val)
{
  std::ostringstream ss;
  ss << val;
  return ss.str();
}

//------------------------------------------------------------------------------
vtkIdType GetNumberOfCellsForPrimitive(int mode, int cellSize, int numberOfIndices)
{
  if (cellSize <= 0)
  {
    vtkWarningWithObjectMacro(nullptr, "Invalid cell size. Ignoring connectivity.");
    return 0;
  }
  switch (mode)
  {
    case vtkGLTFDocumentLoaderInternals::GL_TRIANGLES:
    case vtkGLTFDocumentLoaderInternals::GL_LINES:
    case vtkGLTFDocumentLoaderInternals::GL_POINTS:
      return numberOfIndices / cellSize;
    case vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_FAN:
      return numberOfIndices - 2;
    case vtkGLTFDocumentLoaderInternals::GL_LINE_LOOP:
      return numberOfIndices;
    case vtkGLTFDocumentLoaderInternals::GL_LINE_STRIP:
    case vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_STRIP:
      return 1; // Number of strips
    default:
      vtkWarningWithObjectMacro(nullptr, "Invalid primitive draw mode. Ignoring connectivity.");
      return 0;
  }
}

//------------------------------------------------------------------------------
void GenerateIndicesForPrimitive(vtkGLTFDocumentLoader::Primitive& primitive)
{
  primitive.Indices = vtkSmartPointer<vtkCellArray>::New();

  vtkIdType nVert = primitive.Geometry->GetPoints()->GetNumberOfPoints();

  // Handles cases where we need a single cell
  if (primitive.Mode == vtkGLTFDocumentLoaderInternals::GL_LINE_STRIP ||
    primitive.Mode == vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_STRIP ||
    primitive.Mode == vtkGLTFDocumentLoaderInternals::GL_LINE_LOOP)
  {
    primitive.Indices->AllocateEstimate(1, 1);
    std::vector<vtkIdType> cell(nVert);
    // Append all indices
    std::iota(cell.begin(), cell.end(), 0);
    if (primitive.Mode == vtkGLTFDocumentLoaderInternals::GL_LINE_LOOP)
    {
      cell.push_back(0);
    }
    primitive.Indices->InsertNextCell(static_cast<vtkIdType>(cell.size()), cell.data());
  }
  else
  {
    vtkIdType nCells = GetNumberOfCellsForPrimitive(primitive.Mode, primitive.CellSize, nVert);
    primitive.Indices->AllocateEstimate(nCells, 1);
    std::vector<vtkIdType> cell(primitive.CellSize, 0);
    for (int cellId = 0; cellId < nCells; cellId++)
    {
      // Triangle fan (for each vertex N, create primitive {0, n-1, n})
      if (primitive.Mode == vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_FAN)
      {
        cell[0] = 0;
        cell[1] = cellId + 1;
        cell[2] = cellId + 2;
      }
      else
      {
        std::iota(cell.begin(), cell.end(), primitive.CellSize * cellId);
      }
      primitive.Indices->InsertNextCell(primitive.CellSize, cell.data());
    }
  }
}
}

//------------------------------------------------------------------------------
const std::vector<std::string> vtkGLTFDocumentLoader::SupportedExtensions = { "KHR_lights_punctual",
  "KHR_materials_unlit" };

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkGLTFDocumentLoader);

/** Metadata loading **/
//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::LoadModelMetaDataFromFile(std::string fileName)
{
  vtkGLTFDocumentLoaderInternals impl;

  impl.Self = this;
  // Create new Model and delete previous one
  this->InternalModel = std::make_shared<Model>();
  if (this->InternalModel == nullptr)
  {
    vtkErrorMacro("Could not allocate InternalModel");
    return false;
  }

  fileName = vtksys::SystemTools::CollapseFullPath(fileName);
  this->InternalModel->FileName = fileName;

  if (!impl.LoadModelMetaDataFromFile(fileName, this->UsedExtensions))
  {
    return false;
  }
  return true;
}

/** Data loading **/
//------------------------------------------------------------------------------
template <typename Type>
struct vtkGLTFDocumentLoader::BufferDataExtractionWorker
{
  int ByteOffset;
  int ByteStride;
  int Count;
  const std::vector<char>* Inbuf;
  int NumberOfComponents;
  bool Normalized = false;
  bool NormalizeTuples = false;
  bool LoadTangents = false;

  /**
   * Extracts data from a binary buffer into a typed vtk array.
   * If NormalizeTuples is set to true, tuples will be normalized between 0 and 1
   * If normalized is set to true, normalized integers will be converted to float
   */
  template <typename ArrayType>
  void operator()(ArrayType* output)
  {
    if (output == nullptr)
    {
      return;
    }

    VTK_ASSUME(output->GetNumberOfComponents() == this->NumberOfComponents);

    if (this->LoadTangents)
    {
      output->SetNumberOfComponents(3);
    }

    size_t size = sizeof(Type);

    // If a special stride value is not specified, the step size is equal to the size of an
    // element.
    size_t step = this->ByteStride == 0 ? this->NumberOfComponents * size : this->ByteStride;

    output->Allocate(this->NumberOfComponents * this->Count);

    // keeps track of the last tuple's index. Only used if this->NormalizeTuples is set to true
    int tupleCount = 0;
    // iterate across elements
    for (auto it = this->Inbuf->begin() + this->ByteOffset;
         it != this->Inbuf->begin() + this->ByteOffset + this->Count * step; it += step)
    {
      // iterate across element components
      for (auto elemIt = it; elemIt != it + this->NumberOfComponents * size; elemIt += size)
      {
        if (this->LoadTangents && (elemIt - it) == 3 * static_cast<int>(size))
        {
          break;
        }
        Type val;
        std::copy(elemIt, elemIt + size, reinterpret_cast<char*>(&val));
        if (this->Normalized)
        {
          // Convert from normalized integer ([min val;max val]) to normalized real ([0.0;1.0] for
          // unsigned types, or [-1.0;1.0] for signed types), using the specification's equations
          float max = static_cast<float>(std::numeric_limits<Type>::max());
          float realVal = 0.0f;
          if (std::numeric_limits<Type>::is_signed)
          {
            realVal = vtkMath::Max(val / max, -1.0f);
          }
          else
          {
            realVal = val / max;
          }
          output->InsertNextValue(realVal);
        }
        else
        {
          output->InsertNextValue(val);
        }
      }
      // normalize the previous tuple

      if (this->NormalizeTuples)
      {
        std::vector<double> tuple(output->GetNumberOfComponents(), 0);
        output->GetTuple(tupleCount, tuple.data());
        // compute sum
        double tupleSum = std::accumulate(tuple.begin(), tuple.end(), 0.0);
        // check sum value
        if (tupleSum != 1 && tupleSum != 0)
        {
          // normalize
          for (int i = 0; i < output->GetNumberOfComponents(); i++)
          {
            tuple[i] /= tupleSum;
            output->SetComponent(tupleCount, i, tuple[i]);
          }
        }
        tupleCount++;
      }
    }
  }
};

//------------------------------------------------------------------------------
struct vtkGLTFDocumentLoader::AccessorLoadingWorker
{
  const std::vector<Accessor>* Accessors;
  const std::vector<BufferView>* BufferViews;
  const std::vector<std::vector<char>>* Buffers;
  int AccessorId;
  AccessorType ExpectedType;
  bool NormalizeTuples = false;
  bool Result = false;
  bool LoadTangents = false;

  /**
   * Maps ComponentType value to actual component type, then calls
   * ExecuteBufferDataExtractionWorker, forwarding template types and parameters.
   */
  template <typename ArrayType, typename vtkArrayDispatchType>
  void DispatchWorkerExecutionByComponentType(
    ArrayType* output, const Accessor& accessor, const BufferView& bufferView)
  {
    switch (accessor.ComponentTypeValue)
    {
      case ComponentType::BYTE:
        this->ExecuteBufferDataExtractionWorker<char, ArrayType, vtkArrayDispatchType>(
          output, accessor, bufferView);
        break;
      case ComponentType::UNSIGNED_BYTE:
        this->ExecuteBufferDataExtractionWorker<unsigned char, ArrayType, vtkArrayDispatchType>(
          output, accessor, bufferView);
        break;
      case ComponentType::SHORT:
        this->ExecuteBufferDataExtractionWorker<short, ArrayType, vtkArrayDispatchType>(
          output, accessor, bufferView);
        break;
      case ComponentType::UNSIGNED_SHORT:
        this->ExecuteBufferDataExtractionWorker<uint16_t, ArrayType, vtkArrayDispatchType>(
          output, accessor, bufferView);
        break;
      case ComponentType::UNSIGNED_INT:
        this->ExecuteBufferDataExtractionWorker<uint32_t, ArrayType, vtkArrayDispatchType>(
          output, accessor, bufferView);
        break;
      case ComponentType::FLOAT:
        this->ExecuteBufferDataExtractionWorker<float, ArrayType, vtkArrayDispatchType>(
          output, accessor, bufferView);
        break;
      default:
        return;
    }
  }

  /**
   * Determines vtkArrayDispatch type, then calls DispatchWorkerExecutionByComponentType,
   * forwarding template types and parameters.
   */
  template <typename ArrayType>
  void DispatchWorkerExecution(
    ArrayType* output, const Accessor& accessor, const BufferView& bufferView)
  {
    if (accessor.Normalized || accessor.ComponentTypeValue == ComponentType::FLOAT)
    {
      this->DispatchWorkerExecutionByComponentType<ArrayType, vtkArrayDispatch::Reals>(
        output, accessor, bufferView);
    }
    else
    {
      this->DispatchWorkerExecutionByComponentType<ArrayType, vtkArrayDispatch::Integrals>(
        output, accessor, bufferView);
    }
  }

  /**
   * Creates a new BufferDataExtractionWorker, initializes it and starts its execution
   */
  template <typename ComponentType, typename ArrayType, typename vtkArrayDispatchType>
  void ExecuteBufferDataExtractionWorker(
    ArrayType* output, const Accessor& accessor, const BufferView& bufferView)
  {
    // Create worker
    BufferDataExtractionWorker<ComponentType> worker;
    // Set worker parameters
    worker.ByteOffset = bufferView.ByteOffset + accessor.ByteOffset;
    worker.ByteStride = bufferView.ByteStride;
    worker.Count = accessor.Count;
    worker.Inbuf = &this->Buffers->operator[](bufferView.Buffer);
    worker.Normalized = accessor.Normalized;
    worker.NormalizeTuples = this->NormalizeTuples;
    worker.NumberOfComponents = accessor.NumberOfComponents;
    worker.LoadTangents = this->LoadTangents;

    // Start worker execution
    vtkArrayDispatch::DispatchByValueType<vtkArrayDispatchType>::Execute(output, worker);
  }

  void Setup(int accessorId, vtkGLTFDocumentLoader::AccessorType expectedType)
  {
    this->AccessorId = accessorId;
    this->ExpectedType = expectedType;
  }

  template <typename ArrayType>
  void operator()(ArrayType* output)
  {
    this->Result = false;
    // Checks
    if (this->Accessors == nullptr || this->Buffers == nullptr || this->BufferViews == nullptr)
    {
      return;
    }
    int nbAccessors = static_cast<int>(this->Accessors->size());
    if (output == nullptr || this->AccessorId < 0 || this->AccessorId >= nbAccessors)
    {
      return;
    }
    const Accessor& accessor = this->Accessors->operator[](this->AccessorId);
    if (accessor.Type != this->ExpectedType)
    {
      return;
    }

    // Load base accessor data
    if (accessor.BufferView >= 0)
    {
      const BufferView& bufferView = this->BufferViews->operator[](accessor.BufferView);

      output->SetNumberOfComponents(GetNumberOfComponentsForType(this->ExpectedType));

      DispatchWorkerExecution<ArrayType>(output, accessor, bufferView);
    }
    else if (!accessor.IsSparse)
    {
      return;
    }

    // Load sparse accessor data
    if (accessor.IsSparse)
    {
      // If accessor.bufferview is undefined, the accessor is initialized as an array of zeroes
      if (accessor.BufferView < 0)
      {
        output->SetNumberOfComponents(accessor.NumberOfComponents);
        output->Allocate(accessor.Count * accessor.NumberOfComponents);
        output->Fill(0);
      }

      const Accessor::Sparse& sparse = accessor.SparseObject;
      const BufferView& indicesBufferView = BufferViews->operator[](sparse.IndicesBufferView);
      const BufferView& valuesBufferView = BufferViews->operator[](sparse.ValuesBufferView);

      // Load indices
      vtkNew<vtkIntArray> sparseIndices;
      sparseIndices->SetNumberOfComponents(1);

      Accessor mockIndicesAccessor = accessor;
      mockIndicesAccessor.Count = sparse.Count;
      mockIndicesAccessor.ByteOffset = sparse.IndicesByteOffset;
      mockIndicesAccessor.NumberOfComponents = 1;
      mockIndicesAccessor.ComponentTypeValue = sparse.IndicesComponentType;

      DispatchWorkerExecution<vtkIntArray>(sparseIndices, mockIndicesAccessor, indicesBufferView);

      // Load values
      vtkNew<ArrayType> sparseValues;
      sparseValues->SetNumberOfComponents(accessor.NumberOfComponents);

      Accessor mockValuesAccessor = accessor;
      mockValuesAccessor.Count = sparse.Count;
      mockValuesAccessor.ByteOffset = sparse.ValuesByteOffset;

      DispatchWorkerExecution<ArrayType>(
        sparseValues.GetPointer(), mockValuesAccessor, valuesBufferView);

      // Replace values into original (non sparse) array
      for (int id = 0; id < sparseIndices->GetNumberOfValues(); id++)
      {
        int index = sparseIndices->GetValue(id);
        // Get tuple from sparse values array
        std::vector<typename ArrayType::ValueType> tuple(sparseValues->GetNumberOfComponents());
        sparseValues->GetTypedTuple(id, tuple.data());
        // Set corresponding tuple in output
        output->SetTypedTuple(index, tuple.data());
      }
    }
    this->Result = true;
  }
};

namespace
{
//------------------------------------------------------------------------------
/**
 * Extracts a primitive's connectivity indices, and stores the corresponding cells into a
 * vtkCellArray.
 */
template <typename Type>
void ExtractAndCastCellBufferData(const std::vector<char>& inbuf,
  vtkSmartPointer<vtkCellArray> output, int byteOffset, int byteStride, int count,
  int numberOfComponents, int mode = vtkGLTFDocumentLoaderInternals::GL_TRIANGLES)
{
  if (output == nullptr)
  {
    return;
  }

  // Compute the step between each value
  size_t size = sizeof(Type);
  size_t step = byteStride == 0 ? size : byteStride;

  // Compute cell size
  vtkIdType cellSize = numberOfComponents;
  if (mode == vtkGLTFDocumentLoaderInternals::GL_LINE_STRIP ||
    mode == vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_STRIP)
  {
    cellSize = count;
  }
  else if (mode == vtkGLTFDocumentLoaderInternals::GL_LINE_LOOP)
  {
    cellSize = count + 1;
  }

  // Preallocate cells
  vtkIdType nCells = GetNumberOfCellsForPrimitive(mode, numberOfComponents, count);
  output->AllocateEstimate(nCells, 1);

  std::vector<vtkIdType> currentCell(cellSize);

  // Loop iterators
  auto accessorBegin = inbuf.begin() + byteOffset;
  auto accessorEnd = accessorBegin + count * step;

  // Will be used to read each buffer value
  Type val;

  if (mode == vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_FAN)
  {
    // The first two iterations set currentCell[0] and currentCell[1], then for each iteration, we
    // read the current index into currentCell[2], insert the new cell into the output array, then
    // set currentCell[1] to currentCell[2]
    size_t i = 0;
    for (auto it = accessorBegin; it != accessorEnd; it += step)
    {
      // Read the current value
      std::copy(it, it + size, reinterpret_cast<char*>(&val));
      currentCell[i] = static_cast<vtkIdType>(val);

      // First two iterations: set currentCell[0] then currentCell[1]
      if (it <= accessorBegin + step)
      {
        i++;
      }
      // Following iterations: insert the new cell into the output array, then save the current
      // index value into currentCell[1]
      else
      {
        output->InsertNextCell(static_cast<vtkIdType>(currentCell.size()), currentCell.data());
        // Save the current third triangle index to be the second index of the next triangle cell
        currentCell[1] = currentCell[2];
      }
    }
  }
  else
  {
    auto cellPosition = currentCell.begin();

    // Iterate across the buffer's elements
    for (auto it = accessorBegin; it != accessorEnd; it += step)
    {
      // Read the current index value from the buffer
      std::copy(it, it + size, reinterpret_cast<char*>(&val));
      // Append the current index value to the cell
      *cellPosition = static_cast<vtkIdType>(val);
      // Advance the iterator
      cellPosition++;

      // When we have read all of the current cell's components, insert it into the cell array
      if (cellPosition == currentCell.end())
      {
        output->InsertNextCell(static_cast<vtkIdType>(currentCell.size()), currentCell.data());
        // Start creating the new cell
        cellPosition = currentCell.begin();
      }
    }

    // In case of a line loop, we need to append the first index value at the end of the cell, then
    // insert the cell into the cell array
    if (mode == vtkGLTFDocumentLoaderInternals::GL_LINE_LOOP)
    {
      currentCell.back() = currentCell[0];
      output->InsertNextCell(cellSize, currentCell.data());
    }
  }
}
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::ExtractPrimitiveAccessorData(Primitive& primitive)
{
  // Load connectivity
  if (primitive.IndicesId >= 0)
  {
    // Load indices
    auto accessor = this->InternalModel->Accessors[primitive.IndicesId];
    auto bufferView = this->InternalModel->BufferViews[accessor.BufferView];

    if (accessor.Type != AccessorType::SCALAR)
    {
      vtkErrorMacro(
        "Invalid accessor.type value for primitive connectivity loading. Expected 'SCALAR'");
      return false;
    }
    auto& buffer = this->InternalModel->Buffers[bufferView.Buffer];

    primitive.Indices = vtkSmartPointer<vtkCellArray>::New();
    unsigned int byteOffset = accessor.ByteOffset + bufferView.ByteOffset;

    switch (accessor.ComponentTypeValue)
    {
      case ComponentType::UNSIGNED_BYTE:
        ExtractAndCastCellBufferData<uint8_t>(buffer, primitive.Indices, byteOffset,
          bufferView.ByteStride, accessor.Count, primitive.CellSize, primitive.Mode);
        break;
      case ComponentType::UNSIGNED_SHORT:
        ExtractAndCastCellBufferData<uint16_t>(buffer, primitive.Indices, byteOffset,
          bufferView.ByteStride, accessor.Count, primitive.CellSize, primitive.Mode);
        break;
      case ComponentType::UNSIGNED_INT:
        ExtractAndCastCellBufferData<uint32_t>(buffer, primitive.Indices, byteOffset,
          bufferView.ByteStride, accessor.Count, primitive.CellSize, primitive.Mode);
        break;
      default:
        vtkErrorMacro("Invalid accessor.componentType for primitive connectivity. Expected either "
                      "GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT or GL_UNSIGNED_INT.");
        return false;
    }
  }
  else
  {
    primitive.Indices = nullptr;
  }

  if (!this->ExtractPrimitiveAttributes(primitive))
  {
    vtkErrorMacro("Error loading mesh.primitive.attributes");
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::ExtractPrimitiveAttributes(Primitive& primitive)
{
  AccessorLoadingWorker worker;
  worker.Accessors = &(this->InternalModel->Accessors);
  worker.BufferViews = &(this->InternalModel->BufferViews);
  worker.Buffers = &(this->InternalModel->Buffers);
  using AttributeArrayTypes =
    vtkTypeList::Create<vtkFloatArray, vtkIntArray, vtkUnsignedShortArray>;

  // Load all attributes
  for (auto& attributePair : primitive.AttributeIndices)
  {
    Accessor accessor = this->InternalModel->Accessors[attributePair.second];
    // Create array
    if (attributePair.first == "JOINTS_0")
    {
      primitive.AttributeValues[attributePair.first] =
        vtkSmartPointer<vtkUnsignedShortArray>::New();
    }
    else
    {
      primitive.AttributeValues[attributePair.first] = vtkSmartPointer<vtkFloatArray>::New();
    }

    worker.NormalizeTuples = attributePair.first == "WEIGHTS_0";
    worker.LoadTangents = attributePair.first == "TANGENT";

    // Read data
    worker.Setup(attributePair.second, accessor.Type);
    vtkArrayDispatch::DispatchByArray<AttributeArrayTypes>::Execute(
      primitive.AttributeValues[attributePair.first], worker);

    if (!worker.Result)
    {
      vtkErrorMacro("Error loading mesh.primitive attribute '" << attributePair.first << "'");
      return false;
    }
  }

  worker.NormalizeTuples = false;
  worker.LoadTangents = false;

  // Load morph targets
  for (auto& target : primitive.Targets)
  {
    for (auto& attributePair : target.AttributeIndices)
    {
      if (attributePair.first != "POSITION" && attributePair.first != "NORMAL" &&
        attributePair.first != "TANGENT")
      {
        vtkWarningMacro(
          "Invalid attribute name for morph target: " << attributePair.first << " ignoring.");
        continue;
      }
      Accessor accessor = this->InternalModel->Accessors[attributePair.second];
      target.AttributeValues[attributePair.first] = vtkSmartPointer<vtkFloatArray>::New();
      worker.Setup(attributePair.second, accessor.Type);
      vtkArrayDispatch::DispatchByArray<AttributeArrayTypes>::Execute(
        target.AttributeValues[attributePair.first], worker);
      if (!worker.Result)
      {
        vtkErrorMacro(
          "Error loading mesh.primitive.target attribute '" << attributePair.first << "'");
        return false;
      }
    }
  }
  return true;
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::LoadAnimationData()
{
  AccessorLoadingWorker worker;
  worker.Accessors = &(this->InternalModel->Accessors);
  worker.BufferViews = &(this->InternalModel->BufferViews);
  worker.Buffers = &(this->InternalModel->Buffers);

  using AttributeArrayTypes = vtkTypeList::Create<vtkFloatArray>;

  for (Animation& animation : this->InternalModel->Animations)
  {
    float maxDuration = 0;
    for (Animation::Sampler& sampler : animation.Samplers)
    {
      // Create arrays
      sampler.InputData = vtkSmartPointer<vtkFloatArray>::New();
      sampler.OutputData = vtkSmartPointer<vtkFloatArray>::New();

      // Load inputs (time stamps)
      worker.Setup(sampler.Input, AccessorType::SCALAR);
      vtkArrayDispatch::DispatchByArray<AttributeArrayTypes>::Execute(sampler.InputData, worker);
      if (!worker.Result)
      {
        vtkErrorMacro(
          "Error loading animation.sampler.input buffer data for animation " << animation.Name);
        return false;
      }
      // Get max duration
      float duration = sampler.InputData->GetValueRange()[1];
      maxDuration = vtkMath::Max(maxDuration, duration);

      // Load outputs (frame data)
      worker.Setup(sampler.Output, this->InternalModel->Accessors[sampler.Output].Type);
      vtkArrayDispatch::DispatchByArray<AttributeArrayTypes>::Execute(sampler.OutputData, worker);
      if (!worker.Result)
      {
        vtkErrorMacro(
          "Error loading animation.sampler.output buffer data for animation " << animation.Name);
        return false;
      }

      // Get actual tuple size when loading morphing weights
      unsigned int numberOfComponents = sampler.OutputData->GetNumberOfComponents();
      // If we're loading T/R/S, tuple size is already set (to 3 or 4) in outputdata.
      if (numberOfComponents ==
        vtkGLTFDocumentLoader::GetNumberOfComponentsForType(AccessorType::SCALAR))
      {
        unsigned int nInput = sampler.InputData->GetNumberOfValues();
        unsigned int nOutput = sampler.OutputData->GetNumberOfValues();

        if (sampler.Interpolation == Animation::Sampler::InterpolationMode::CUBICSPLINE)
        {
          nOutput /= 3;
        }

        if (nInput == 0 || nOutput % nInput != 0)
        {
          // Output size has to be a multiple of the Input size, or we're missing data
          vtkErrorMacro("Invalid animation.sampler data. The number of outputs should be a "
                        "multiple of the number of inputs");
          return false;
        }
        numberOfComponents = nOutput / nInput;
      }
      sampler.OutputData->SetNumberOfComponents(numberOfComponents);
    }
    animation.Duration = maxDuration;
  }
  return true;
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::LoadImageData()
{
  vtkNew<vtkImageReader2Factory> factory;

  for (Image& image : this->InternalModel->Images)
  {
    vtkSmartPointer<vtkImageReader2> reader = nullptr;
    image.ImageData = vtkSmartPointer<vtkImageData>::New();
    std::vector<char> buffer;

    // If mime-type is defined, get appropriate reader here (only two possible values)
    if (image.MimeType == "image/jpeg")
    {
      reader = vtkSmartPointer<vtkJPEGReader>::New();
    }
    else if (image.MimeType == "image/png")
    {
      reader = vtkSmartPointer<vtkPNGReader>::New();
    }

    // If image is defined via bufferview index
    if (image.BufferView >= 0 &&
      image.BufferView < static_cast<int>(this->InternalModel->BufferViews.size()))
    {
      BufferView& bufferView = this->InternalModel->BufferViews[image.BufferView];
      int bufferId = bufferView.Buffer;
      if (bufferId < 0 || bufferId >= static_cast<int>(this->InternalModel->Buffers.size()))
      {
        vtkErrorMacro("Invalid bufferView.buffer value for bufferView " << bufferView.Name);
        return false;
      }
      reader->SetMemoryBufferLength(
        static_cast<vtkIdType>(this->InternalModel->Buffers[bufferId].size()));
      reader->SetMemoryBuffer(
        this->InternalModel->Buffers[bufferId].data() + bufferView.ByteOffset);
    }
    else // If image is defined via uri
    {
      // Check for data-uri
      if (vtksys::SystemTools::StringStartsWith(image.Uri, "data:"))
      {
        vtkGLTFUtils::GetBinaryBufferFromUri(
          image.Uri, this->InternalModel->FileName, buffer, image.Uri.size());
        // If mime-type is defined, get appropriate reader here (only two possible values)
        std::string type = vtkGLTFUtils::GetDataUriMimeType(image.Uri);
        if (type == "image/jpeg")
        {
          reader = vtkSmartPointer<vtkJPEGReader>::New();
        }
        else if (type == "image/png")
        {
          reader = vtkSmartPointer<vtkPNGReader>::New();
        }
        else
        {
          vtkErrorMacro("Invalid MIME-Type for image");
          return false;
        }
        reader->SetMemoryBufferLength(static_cast<vtkIdType>(image.Uri.size()));
        reader->SetMemoryBuffer(buffer.data());
      }
      // Read from file
      else
      {
        std::string imageFilePath(
          vtkGLTFUtils::GetResourceFullPath(image.Uri, this->InternalModel->FileName));
        reader.TakeReference(factory->CreateImageReader2(imageFilePath.c_str()));
        if (reader == nullptr)
        {
          vtkErrorMacro("Invalid format for image " << image.Uri);
          return false;
        }
        reader->SetFileName(imageFilePath.c_str());
      }
    }
    if (reader == nullptr)
    {
      vtkErrorMacro("Invalid image object");
      return false;
    }
    reader->Update();
    image.ImageData = reader->GetOutput();
  }
  return true;
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::LoadSkinMatrixData()
{
  AccessorLoadingWorker worker;
  worker.Accessors = &(this->InternalModel->Accessors);
  worker.BufferViews = &(this->InternalModel->BufferViews);
  worker.Buffers = &(this->InternalModel->Buffers);

  using AttributeArrayTypes = vtkTypeList::Create<vtkFloatArray, vtkIntArray>;

  for (Skin& skin : this->InternalModel->Skins)
  {
    if (skin.InverseBindMatricesAccessorId < 0)
    {
      // Default is an identity matrix
      vtkNew<vtkMatrix4x4> id;
      id->Identity();
      skin.InverseBindMatrices.emplace_back(id);
      continue;
    }
    vtkNew<vtkFloatArray> matrixValues;
    worker.Setup(skin.InverseBindMatricesAccessorId, AccessorType::MAT4);
    vtkArrayDispatch::DispatchByArray<AttributeArrayTypes>::Execute(matrixValues, worker);

    size_t totalNumberOfComponents =
      skin.Joints.size() * vtkGLTFDocumentLoader::GetNumberOfComponentsForType(AccessorType::MAT4);
    if (!worker.Result ||
      static_cast<size_t>(matrixValues->GetNumberOfValues()) != totalNumberOfComponents)
    {
      vtkErrorMacro("Error loading skin.invertBindMatrices data");
      return false;
    }

    for (unsigned int matrixId = 0; matrixId < skin.Joints.size(); matrixId++)
    {
      vtkNew<vtkMatrix4x4> matrix;
      matrix->DeepCopy(matrixValues->GetTuple(matrixId));
      matrix->Transpose();
      skin.InverseBindMatrices.emplace_back(matrix);
    }
  }
  return true;
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::LoadModelData(const std::vector<char>& glbBuffer)
{
  vtkGLTFDocumentLoaderInternals impl;
  impl.Self = this;

  if (this->InternalModel == nullptr)
  {
    vtkErrorMacro("Error loading model data: metadata was not loaded");
    return false;
  }

  // Push optional glB buffer
  if (!glbBuffer.empty())
  {
    this->InternalModel->Buffers.push_back(glbBuffer);
  }

  impl.LoadBuffers(!glbBuffer.empty());

  // Read primitive attributes from buffers
  size_t numberOfMeshes = this->InternalModel->Meshes.size();
  for (size_t i = 0; i < numberOfMeshes; i++)
  {
    for (Primitive& primitive : this->InternalModel->Meshes[i].Primitives)
    {
      this->ExtractPrimitiveAccessorData(primitive);
    }
    double progress = (i + 1) / static_cast<double>(numberOfMeshes);
    this->InvokeEvent(vtkCommand::ProgressEvent, static_cast<void*>(&progress));
  }
  // Read additional buffer data
  if (!this->LoadAnimationData())
  {
    return false;
  }
  if (!this->LoadImageData())
  {
    return false;
  }
  return this->LoadSkinMatrixData();
}

/** vtk object building and animation operations **/
//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::ApplyAnimation(float t, int animationId, bool forceStep)
{
  const Animation& animation = this->InternalModel->Animations[animationId];
  for (const Animation::Channel& channel : animation.Channels)
  {
    Node& node = this->InternalModel->Nodes[channel.TargetNode];
    const Animation::Sampler& sampler = animation.Samplers[channel.Sampler];

    std::vector<float>* target;

    size_t numberOfComponents = 0;
    switch (channel.TargetPath)
    {
      case Animation::Channel::PathType::ROTATION:
        numberOfComponents =
          vtkGLTFDocumentLoader::GetNumberOfComponentsForType(AccessorType::VEC4);
        target = &(node.Rotation);
        break;
      case Animation::Channel::PathType::TRANSLATION:
        numberOfComponents =
          vtkGLTFDocumentLoader::GetNumberOfComponentsForType(AccessorType::VEC3);
        target = &(node.Translation);
        break;
      case Animation::Channel::PathType::SCALE:
        numberOfComponents =
          vtkGLTFDocumentLoader::GetNumberOfComponentsForType(AccessorType::VEC3);
        target = &(node.Scale);
        break;
      case Animation::Channel::PathType::WEIGHTS:
        numberOfComponents = node.InitialWeights.size();
        if (numberOfComponents == 0)
        {
          int nbMeshes = static_cast<int>(this->InternalModel->Meshes.size());
          if (node.Mesh < 0 || node.Mesh > nbMeshes)
          {
            vtkErrorMacro("Invalid node.mesh value.");
            return false;
          }
          numberOfComponents = this->InternalModel->Meshes[node.Mesh].Weights.size();
        }
        target = &(node.Weights);
        break;
      default:
        vtkErrorMacro(
          "Invalid animation.channel.target.path value for animation " << animation.Name);
        return false;
    }
    target->clear();
    target->reserve(numberOfComponents);
    sampler.GetInterpolatedData(t, numberOfComponents, target, forceStep,
      channel.TargetPath == Animation::Channel::PathType::ROTATION);
    node.UpdateTransform();
  }
  return true;
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::ResetAnimation(int animationId)
{
  const Animation& animation = this->InternalModel->Animations[animationId];
  for (const Animation::Channel& channel : animation.Channels)
  {
    Node& node = this->InternalModel->Nodes[channel.TargetNode];
    switch (channel.TargetPath)
    {
      case Animation::Channel::PathType::ROTATION:
        node.Rotation = node.InitialRotation;
        break;
      case Animation::Channel::PathType::TRANSLATION:
        node.Translation = node.InitialTranslation;
        break;
      case Animation::Channel::PathType::SCALE:
        node.Scale = node.InitialScale;
        break;
      case Animation::Channel::PathType::WEIGHTS:
        node.Weights = node.InitialWeights;
        break;
      default:
        vtkErrorMacro(
          "Invalid animation.channel.target.path value for animation " << animation.Name);
    }
    node.UpdateTransform();
  }
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::BuildPolyDataFromPrimitive(Primitive& primitive)
{
  // Positions
  primitive.Geometry = vtkSmartPointer<vtkPolyData>::New();
  if (primitive.AttributeValues.count("POSITION"))
  {
    primitive.Geometry->SetPoints(vtkSmartPointer<vtkPoints>::New());
    primitive.Geometry->GetPoints()->SetData(primitive.AttributeValues["POSITION"]);
    primitive.AttributeValues.erase("POSITION");
  }

  // Connectivity
  if (primitive.Indices == nullptr)
  {
    GenerateIndicesForPrimitive(primitive);
  }
  switch (primitive.Mode)
  {
    case vtkGLTFDocumentLoaderInternals::GL_TRIANGLES:
    case vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_FAN:
      primitive.Geometry->SetPolys(primitive.Indices);
      break;
    case vtkGLTFDocumentLoaderInternals::GL_LINES:
    case vtkGLTFDocumentLoaderInternals::GL_LINE_STRIP:
    case vtkGLTFDocumentLoaderInternals::GL_LINE_LOOP:
      primitive.Geometry->SetLines(primitive.Indices);
      break;
    case vtkGLTFDocumentLoaderInternals::GL_POINTS:
      primitive.Geometry->SetVerts(primitive.Indices);
      break;
    case vtkGLTFDocumentLoaderInternals::GL_TRIANGLE_STRIP:
      primitive.Geometry->SetStrips(primitive.Indices);
      break;
    default:
      vtkWarningMacro("Invalid primitive draw mode. Ignoring connectivity.");
  }

  // Other attributes

  // Set array names
  for (const auto& it : primitive.AttributeValues)
  {
    it.second->SetName(it.first.c_str());
  }

  auto pointData = primitive.Geometry->GetPointData();
  if (primitive.AttributeValues.count("NORMAL"))
  {
    pointData->SetNormals(primitive.AttributeValues["NORMAL"]);
    primitive.AttributeValues.erase("NORMAL");
  }
  if (primitive.AttributeValues.count("TANGENT"))
  {
    pointData->SetTangents(primitive.AttributeValues["TANGENT"]);
    primitive.AttributeValues.erase("TANGENT");
  }
  if (primitive.AttributeValues.count("COLOR_0"))
  {
    pointData->SetScalars(primitive.AttributeValues["COLOR_0"]);
    primitive.AttributeValues.erase("COLOR_0");
  }
  if (primitive.AttributeValues.count("TEXCOORD_0"))
  {
    pointData->SetTCoords(primitive.AttributeValues["TEXCOORD_0"]);
    primitive.AttributeValues.erase("TEXCOORD_0");
  }
  if (primitive.AttributeValues.count("TEXCOORD_1"))
  {
    primitive.AttributeValues["TEXCOORD_1"]->SetName("texcoord_1");
    pointData->AddArray(primitive.AttributeValues["TEXCOORD_1"]);
    primitive.AttributeValues.erase("TEXCOORD_1");
  }
  // Spec only requires 1 set of 4 joints/weights per vert.
  // only those are loaded for now.
  if (primitive.AttributeValues.count("JOINTS_0"))
  {
    pointData->AddArray(primitive.AttributeValues["JOINTS_0"]);
    primitive.AttributeValues.erase("JOINTS_0");
  }
  if (primitive.AttributeValues.count("WEIGHTS_0"))
  {
    pointData->AddArray(primitive.AttributeValues["WEIGHTS_0"]);
    primitive.AttributeValues.erase("WEIGHTS_0");
  }
  // Add remaining attributes
  if (!primitive.AttributeValues.empty())
  {
    for (auto& attributePair : primitive.AttributeValues)
    {
      attributePair.second->SetName(attributePair.first.c_str());
      pointData->AddArray(attributePair.second);
    }
  }

  // Add morph targets
  int targetId = 0;
  for (auto& target : primitive.Targets)
  {
    std::string name;
    if (target.AttributeValues.count("POSITION"))
    {
      name = "target" + value_to_string(targetId) + "_position";
      target.AttributeValues["POSITION"]->SetName(name.c_str());
      pointData->AddArray(target.AttributeValues["POSITION"]);
    }
    if (target.AttributeValues.count("NORMAL"))
    {
      name = "target" + value_to_string(targetId) + "_normal";
      target.AttributeValues["NORMAL"]->SetName(name.c_str());
      pointData->AddArray(target.AttributeValues["NORMAL"]);
    }
    if (target.AttributeValues.count("TANGENT"))
    {
      name = "target" + value_to_string(targetId) + "_tangent";
      target.AttributeValues["TANGENT"]->SetName(name.c_str());
      pointData->AddArray(target.AttributeValues["TANGENT"]);
    }
    targetId++;
  }
  return true;
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::Node::UpdateTransform()
{
  this->Transform->Identity();

  if (this->TRSLoaded)
  {
    std::vector<float> rotationValues = this->InitialRotation;
    std::vector<float> scale = this->InitialScale;
    std::vector<float> translation = this->InitialTranslation;

    if (!this->Translation.empty())
    {
      translation = this->Translation;
    }
    if (!this->Rotation.empty())
    {
      rotationValues = this->Rotation;
    }
    if (!this->Scale.empty())
    {
      scale = this->Scale;
    }
    // Rotate quaternions to match vtk's representation
    std::rotate(
      std::begin(rotationValues), std::begin(rotationValues) + 3, std::end(rotationValues));
    // Initialize quaternion
    vtkQuaternion<float> rotation;
    rotation.Normalize();
    rotation.Set(rotationValues.data());

    float rotationMatrix[3][3];
    rotation.ToMatrix3x3(rotationMatrix);

    // Apply transformations
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        this->Transform->SetElement(i, j, scale[j] * rotationMatrix[i][j]);
      }
      this->Transform->SetElement(i, 3, translation[i]);
    }
  }
  else
  {
    this->Transform->DeepCopy(this->Matrix);
  }
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::Animation::Sampler::GetInterpolatedData(float t,
  size_t numberOfComponents, std::vector<float>* output, bool forceStep, bool isRotation) const
{
  // linear or spline interpolation
  if (this->Interpolation != Animation::Sampler::InterpolationMode::STEP && !forceStep)
  {
    vtkIdType numberOfKeyFrames = this->InputData->GetNumberOfTuples();

    // Find the previous and following keyframes
    vtkIdType nextKeyFrameId =
      std::lower_bound(this->InputData->Begin(), this->InputData->End(), t) -
      this->InputData->Begin();
    vtkIdType prevKeyFrameId = 0;

    // If we didn't find the next keyframe, that means t is over the animation's duration.
    if (nextKeyFrameId == numberOfKeyFrames)
    {
      nextKeyFrameId = numberOfKeyFrames - 1;
      prevKeyFrameId = nextKeyFrameId;
    }
    // Animation hasn't started yet.
    else if (nextKeyFrameId == 0)
    {
      prevKeyFrameId = 0;
    }
    else
    {
      prevKeyFrameId = nextKeyFrameId - 1;
    }

    // Get time values

    // Normalize t. Set to zero when at the first keyframe, and set to one when at the last keyframe
    float tNorm = 0;
    float tDelta = 0;
    if (prevKeyFrameId == 0 && nextKeyFrameId == 0)
    {
      tNorm = 0;
    }
    else if (prevKeyFrameId == numberOfKeyFrames - 1 && nextKeyFrameId == numberOfKeyFrames - 1)
    {
      tNorm = 1;
    }
    else
    {
      const float prevTime = this->InputData->GetValue(prevKeyFrameId);
      const float nextTime = this->InputData->GetValue(nextKeyFrameId);
      tDelta = nextTime - prevTime;
      tNorm = (t - prevTime) / tDelta;
    }

    if (this->Interpolation == Animation::Sampler::InterpolationMode::LINEAR)
    {
      std::vector<float> prevTuple(numberOfComponents);
      std::vector<float> nextTuple(numberOfComponents);
      this->OutputData->GetTypedTuple(prevKeyFrameId, prevTuple.data());
      this->OutputData->GetTypedTuple(nextKeyFrameId, nextTuple.data());

      // If interpolating rotations, we need to use SLERP,
      if (isRotation)
      {
        std::rotate(std::begin(prevTuple), std::begin(prevTuple) + 3, std::end(prevTuple));
        std::rotate(std::begin(nextTuple), std::begin(nextTuple) + 3, std::end(nextTuple));

        vtkQuaternion<float> prevQuaternion(prevTuple.data());
        vtkQuaternion<float> nextQuaternion(nextTuple.data());

        auto interpolatedQuat = prevQuaternion.Slerp(tNorm, nextQuaternion);
        interpolatedQuat.Normalize();

        output->insert(output->end(), interpolatedQuat.GetData(), interpolatedQuat.GetData() + 4);
        std::rotate(output->begin(), output->begin() + 1, output->end());
      }
      else
      {
        // Linear interpolation between the previous and following tuples
        for (size_t i = 0; i < numberOfComponents; i++)
        {
          output->push_back((1 - tNorm) * prevTuple[i] + tNorm * nextTuple[i]);
        }
      }
    }
    // Cubic spline interpolation
    // This implementation follows the glTF specification:
    // https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#appendix-c-spline-interpolation
    else
    {
      std::vector<float> v0(numberOfComponents);
      std::vector<float> v1(numberOfComponents);
      std::vector<float> a(numberOfComponents);
      std::vector<float> b(numberOfComponents);

      // Three tuples per frame: in-tangent, point, out-tangent
      this->OutputData->GetTypedTuple(3 * prevKeyFrameId + 1, v0.data());
      this->OutputData->GetTypedTuple(3 * nextKeyFrameId + 1, v1.data());
      this->OutputData->GetTypedTuple(3 * nextKeyFrameId, a.data());
      this->OutputData->GetTypedTuple(3 * prevKeyFrameId + 2, b.data());

      const float tSquare = tNorm * tNorm;
      const float tCube = tSquare * tNorm;

      const float c0 = 2 * tCube - 3 * tSquare + 1;
      const float c1 = tDelta * (tCube - 2 * tSquare + tNorm);
      const float c2 = -2 * tCube + 3 * tSquare;
      const float c3 = tDelta * (tCube - tSquare);

      for (size_t i = 0; i < numberOfComponents; i++)
      {
        output->push_back(c0 * v0[i] + c1 * b[i] + c2 * v1[i] + c3 * a[i]);
      }

      // Normalize the resulting quaternion
      if (isRotation)
      {
        std::rotate(output->begin(), output->begin() + 3, output->end());
        vtkQuaternion<float> quaternion(output->data());
        quaternion.Normalize();
        float* data = quaternion.GetData();
        std::copy(data, data + 4, output->begin());
        std::rotate(output->begin(), output->begin() + 1, output->end());
      }
    }
  }
  else
  {
    // step interpolation
    // get frame index
    size_t lower = std::lower_bound(this->InputData->Begin(), this->InputData->End(), t) -
      this->InputData->Begin();
    if (lower > 0)
    {
      lower--;
    }

    for (size_t i = lower * numberOfComponents; i < numberOfComponents * (lower + 1); i++)
    {
      output->push_back(this->OutputData->GetValue(static_cast<vtkIdType>(i)));
    }
  }
}

/** File operations **/
//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::LoadFileBuffer(
  const std::string& fileName, std::vector<char>& glbBuffer)
{
  // Get base information
  std::string magic;
  uint32_t version;
  uint32_t fileLength;
  std::vector<vtkGLTFUtils::ChunkInfoType> chunkInfo;
  if (!vtkGLTFUtils::ExtractGLBFileInformation(fileName, magic, version, fileLength, chunkInfo))
  {
    vtkErrorMacro("Invalid .glb file " << fileName);
    return false;
  }

  // Open the file in binary mode
  vtksys::ifstream fin;
  fin.open(fileName.c_str(), std::ios::binary | std::ios::in);
  if (!fin.is_open())
  {
    vtkErrorMacro("Error opening file " << fileName);
    return false;
  }

  // Look for BIN chunk while updating fstream position
  fin.seekg(vtkGLTFUtils::GLBHeaderSize + vtkGLTFUtils::GLBChunkHeaderSize);
  const std::string binaryHeader("BIN\0", 4);
  for (auto& chunk : chunkInfo)
  {
    if (chunk.first == binaryHeader)
    {
      // Read chunk data into output vector
      std::vector<char> BINData(chunk.second);
      fin.read(BINData.data(), chunk.second);
      glbBuffer.insert(glbBuffer.end(), BINData.begin(), BINData.begin() + chunk.second);
      return true;
    }
    // Jump to next chunk
    fin.seekg(chunk.second + vtkGLTFUtils::GLBChunkHeaderSize, std::ios::cur);
  }
  vtkErrorMacro("Could not find any valid BIN chunks in file " << fileName);
  return false;
}

//------------------------------------------------------------------------------
bool vtkGLTFDocumentLoader::BuildModelVTKGeometry()
{
  if (this->InternalModel == nullptr)
  {
    vtkErrorMacro("Error building model data: metadata was not loaded");
    return false;
  }

  // Build poly data
  for (Mesh& mesh : this->InternalModel->Meshes)
  {
    for (Primitive& primitive : mesh.Primitives)
    {
      this->BuildPolyDataFromPrimitive(primitive);
    }
  }
  // Compute global transforms
  for (const auto& scene : this->InternalModel->Scenes)
  {
    for (unsigned int node : scene.Nodes)
    {
      this->BuildGlobalTransforms(node, nullptr);
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::BuildGlobalTransforms(
  unsigned int nodeIndex, vtkSmartPointer<vtkMatrix4x4> parentTransform)
{
  if (nodeIndex >= this->InternalModel->Nodes.size())
  {
    return;
  }
  Node& node = this->InternalModel->Nodes[nodeIndex];

  node.GlobalTransform = vtkSmartPointer<vtkMatrix4x4>::New();
  node.GlobalTransform->DeepCopy(node.Transform);
  if (parentTransform != nullptr)
  {
    vtkMatrix4x4::Multiply4x4(parentTransform, node.GlobalTransform, node.GlobalTransform);
  }
  for (auto childId : node.Children)
  {
    this->BuildGlobalTransforms(childId, node.GlobalTransform);
  }
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::BuildGlobalTransforms()
{
  for (const auto& scene : this->InternalModel->Scenes)
  {
    for (unsigned int node : scene.Nodes)
    {
      this->BuildGlobalTransforms(node, nullptr);
    }
  }
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
std::shared_ptr<vtkGLTFDocumentLoader::Model> vtkGLTFDocumentLoader::GetInternalModel()
{
  return std::shared_ptr<Model>(this->InternalModel);
}

//------------------------------------------------------------------------------
const std::vector<std::string>& vtkGLTFDocumentLoader::GetSupportedExtensions()
{
  return vtkGLTFDocumentLoader::SupportedExtensions;
}

//------------------------------------------------------------------------------
const std::vector<std::string>& vtkGLTFDocumentLoader::GetUsedExtensions()
{
  return this->UsedExtensions;
}

/** types and enums **/
//------------------------------------------------------------------------------
unsigned int vtkGLTFDocumentLoader::GetNumberOfComponentsForType(
  vtkGLTFDocumentLoader::AccessorType type)
{
  switch (type)
  {
    case AccessorType::SCALAR:
      return 1;
    case AccessorType::VEC2:
      return 2;
    case AccessorType::VEC3:
      return 3;
    case AccessorType::VEC4:
      return 4;
    case AccessorType::MAT2:
      return 4;
    case AccessorType::MAT3:
      return 9;
    case AccessorType::MAT4:
      return 16;
    default:
      return 0;
  }
}

//------------------------------------------------------------------------------
void vtkGLTFDocumentLoader::ComputeJointMatrices(const Model& model, const Skin& skin, Node& node,
  std::vector<vtkSmartPointer<vtkMatrix4x4>>& jointMats)
{
  jointMats.clear();
  jointMats.reserve(skin.Joints.size());

  vtkNew<vtkMatrix4x4> inverseMeshGlobal;
  vtkMatrix4x4::Invert(node.GlobalTransform, inverseMeshGlobal);

  for (unsigned int jointId = 0; jointId < skin.Joints.size(); jointId++)
  {
    const Node& jointNode = model.Nodes[skin.Joints[jointId]];

    /**
     * Joint matrices:
     * jointMatrix(j) =
     * globalTransformOfNodeThatTheMeshIsAttachedTo^-1 *
     * globalTransformOfJointNode(j) *
     * inverseBindMatrixForJoint(j);
     * The mesh will be transformed (using vtkWeightedTransformFilter) using this matrix:
     * mat4 skinMat =
     * weight.x * jointMatrix[joint.x] +
     * weight.y * jointMatrix[joint.y] +
     * weight.z * jointMatrix[joint.z] +
     * weight.w * jointMatrix[joint.w];
     */

    vtkNew<vtkMatrix4x4> jointMat;
    vtkMatrix4x4::Multiply4x4(
      jointNode.GlobalTransform, skin.InverseBindMatrices[jointId], jointMat);
    vtkMatrix4x4::Multiply4x4(inverseMeshGlobal, jointMat, jointMat);

    jointMats.emplace_back(jointMat);
  }
}
