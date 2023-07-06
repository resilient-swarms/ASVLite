/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkBitArray.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkBitArray.h"

#include "vtkBitArrayIterator.h"
#include "vtkIdList.h"
#include "vtkObjectFactory.h"

namespace
{
constexpr unsigned char InitializationMaskForUnusedBitsOfLastByte[8] = { 0x80, 0xc0, 0xe0, 0xf0,
  0xf8, 0xfc, 0xfe, 0xff };
} // anonymous namespace

//------------------------------------------------------------------------------
class vtkBitArrayLookup
{
public:
  vtkBitArrayLookup()
    : Rebuild(true)
  {
    this->ZeroArray = nullptr;
    this->OneArray = nullptr;
  }
  ~vtkBitArrayLookup()
  {
    if (this->ZeroArray)
    {
      this->ZeroArray->Delete();
      this->ZeroArray = nullptr;
    }
    if (this->OneArray)
    {
      this->OneArray->Delete();
      this->OneArray = nullptr;
    }
  }
  vtkIdList* ZeroArray;
  vtkIdList* OneArray;
  bool Rebuild;
};

vtkStandardNewMacro(vtkBitArray);

//------------------------------------------------------------------------------
// Instantiate object.
vtkBitArray::vtkBitArray()
{
  this->Array = nullptr;
  this->TupleSize = 3;
  this->Tuple = new double[this->TupleSize]; // used for conversion
  this->DeleteFunction = ::operator delete[];
  this->Lookup = nullptr;
}

//------------------------------------------------------------------------------
vtkBitArray::~vtkBitArray()
{
  if (this->DeleteFunction)
  {
    this->DeleteFunction(this->Array);
  }
  delete[] this->Tuple;
  delete this->Lookup;
}

void vtkBitArray::InitializeUnusedBitsInLastByte()
{
  if (this->MaxId > -1)
  {
    this->Array[this->MaxId / 8] &= InitializationMaskForUnusedBitsOfLastByte[this->MaxId % 8];
  }
}

//------------------------------------------------------------------------------
unsigned char* vtkBitArray::WritePointer(vtkIdType id, vtkIdType number)
{
  vtkIdType newSize = id + number;
  if (newSize > this->Size)
  {
    this->ResizeAndExtend(newSize);
  }
  if ((--newSize) > this->MaxId)
  {
    this->MaxId = newSize;
    this->InitializeUnusedBitsInLastByte();
  }
  this->DataChanged();
  return this->Array + id / 8;
}

//------------------------------------------------------------------------------
// This method lets the user specify data to be held by the array.  The
// array argument is a pointer to the data.  size is the size of
// the array supplied by the user.  Set save to 1 to keep the class
// from deleting the array when it cleans up or reallocates memory.
// The class uses the actual array provided; it does not copy the data
// from the supplied array.
void vtkBitArray::SetArray(unsigned char* array, vtkIdType size, int save, int deleteMethod)
{
  if ((this->Array) && (this->DeleteFunction))
  {
    vtkDebugMacro(<< "Deleting the array...");
    this->DeleteFunction(this->Array);
  }
  else
  {
    vtkDebugMacro(<< "Warning, array not deleted, but will point to new array.");
  }

  vtkDebugMacro(<< "Setting array to: " << array);

  this->Array = array;
  this->Size = size;
  this->MaxId = size - 1;
  this->InitializeUnusedBitsInLastByte();

  if (save != 0)
  {
    this->DeleteFunction = nullptr;
  }
  else if (deleteMethod == VTK_DATA_ARRAY_DELETE || deleteMethod == VTK_DATA_ARRAY_USER_DEFINED)
  {
    this->DeleteFunction = ::operator delete[];
  }
  else if (deleteMethod == VTK_DATA_ARRAY_ALIGNED_FREE)
  {
#ifdef _WIN32
    this->DeleteFunction = _aligned_free;
#else
    this->DeleteFunction = free;
#endif
  }
  else if (deleteMethod == VTK_DATA_ARRAY_FREE)
  {
    this->DeleteFunction = free;
  }

  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::SetArrayFreeFunction(void (*callback)(void*))
{
  this->DeleteFunction = callback;
}

//------------------------------------------------------------------------------
// Get the data at a particular index.
int vtkBitArray::GetValue(vtkIdType id) const
{
  return (this->Array[id / 8] & (0x80 >> (id % 8))) != 0;
}

//------------------------------------------------------------------------------
// Allocate memory for this array. Delete old storage only if necessary.
vtkTypeBool vtkBitArray::Allocate(vtkIdType sz, vtkIdType vtkNotUsed(ext))
{
  if (sz > this->Size)
  {
    if (this->DeleteFunction)
    {
      this->DeleteFunction(this->Array);
    }
    this->Size = (sz > 0 ? sz : 1);
    if ((this->Array = new unsigned char[(this->Size + 7) / 8]) == nullptr)
    {
      return 0;
    }
    this->DeleteFunction = ::operator delete[];
  }

  this->MaxId = -1;
  this->DataChanged();

  return 1;
}

//------------------------------------------------------------------------------
// Release storage and reset array to initial state.
void vtkBitArray::Initialize()
{
  if (this->DeleteFunction)
  {
    this->DeleteFunction(this->Array);
  }
  this->Array = nullptr;
  this->Size = 0;
  this->MaxId = -1;
  this->DeleteFunction = ::operator delete[];
  this->DataChanged();
}

//------------------------------------------------------------------------------
// Deep copy of another bit array.
void vtkBitArray::DeepCopy(vtkDataArray* ia)
{
  // Do nothing on a nullptr input.
  if (ia == nullptr)
  {
    return;
  }

  this->DataChanged();

  if (ia->GetDataType() != VTK_BIT)
  {
    vtkIdType numTuples = ia->GetNumberOfTuples();
    this->NumberOfComponents = ia->GetNumberOfComponents();
    this->SetNumberOfTuples(numTuples);

    for (vtkIdType i = 0; i < numTuples; i++)
    {
      this->SetTuple(i, ia->GetTuple(i));
    }
    return;
  }

  if (this != ia)
  {
    if (this->DeleteFunction)
    {
      this->DeleteFunction(this->Array);
    }

    this->NumberOfComponents = ia->GetNumberOfComponents();
    this->MaxId = ia->GetMaxId();
    this->Size = ia->GetSize();
    this->DeleteFunction = ::operator delete[];

    this->Array = new unsigned char[(this->Size + 7) / 8];
    memcpy(this->Array, static_cast<unsigned char*>(ia->GetVoidPointer(0)),
      static_cast<size_t>((this->Size + 7) / 8) * sizeof(unsigned char));
  }
}

//------------------------------------------------------------------------------
void vtkBitArray::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  if (this->Array)
  {
    os << indent << "Array: " << this->Array << "\n";
  }
  else
  {
    os << indent << "Array: (null)\n";
  }
}

//------------------------------------------------------------------------------
// Private function does "reallocate". Sz is the number of "bits", and we
// can allocate only 8-bit bytes.
unsigned char* vtkBitArray::ResizeAndExtend(vtkIdType sz)
{
  unsigned char* newArray;
  vtkIdType newSize;

  if (sz > this->Size)
  {
    newSize = this->Size + sz;
  }
  else if (sz == this->Size)
  {
    return this->Array;
  }
  else
  {
    newSize = sz;
  }

  if (newSize <= 0)
  {
    this->Initialize();
    return nullptr;
  }

  if ((newArray = new unsigned char[(newSize + 7) / 8]) == nullptr)
  {
    vtkErrorMacro(<< "Cannot allocate memory\n");
    return nullptr;
  }

  if (this->Array)
  {
    vtkIdType usedSize = (sz < this->Size) ? sz : this->Size;

    memcpy(newArray, this->Array, static_cast<size_t>((usedSize + 7) / 8) * sizeof(unsigned char));
    if (this->DeleteFunction)
    {
      this->DeleteFunction(this->Array);
    }
  }

  this->Array = newArray;
  if (newSize < this->Size)
  {
    this->MaxId = newSize - 1;
    this->InitializeUnusedBitsInLastByte();
  }
  this->Size = newSize;
  this->DeleteFunction = ::operator delete[];
  this->DataChanged();

  return this->Array;
}

//------------------------------------------------------------------------------
vtkTypeBool vtkBitArray::Resize(vtkIdType sz)
{
  unsigned char* newArray;
  vtkIdType newSize = sz * this->NumberOfComponents;

  if (newSize == this->Size)
  {
    return 1;
  }

  if (newSize <= 0)
  {
    this->Initialize();
    return 1;
  }

  if ((newArray = new unsigned char[(newSize + 7) / 8]) == nullptr)
  {
    vtkErrorMacro(<< "Cannot allocate memory\n");
    return 0;
  }

  if (this->Array)
  {
    vtkIdType usedSize = (newSize < this->Size) ? newSize : this->Size;

    memcpy(newArray, this->Array, static_cast<size_t>((usedSize + 7) / 8) * sizeof(unsigned char));
    if (this->DeleteFunction)
    {
      this->DeleteFunction(this->Array);
    }
  }

  this->Array = newArray;
  if (newSize < this->Size)
  {
    this->MaxId = newSize - 1;
    this->InitializeUnusedBitsInLastByte();
  }
  this->Size = newSize;
  this->DeleteFunction = ::operator delete[];
  this->DataChanged();

  return 1;
}

//------------------------------------------------------------------------------
// Set the number of n-tuples in the array.
void vtkBitArray::SetNumberOfTuples(vtkIdType number)
{
  this->SetNumberOfValues(number * this->NumberOfComponents);
}

//------------------------------------------------------------------------------
bool vtkBitArray::SetNumberOfValues(vtkIdType number)
{
  if (!this->Superclass::SetNumberOfValues(number))
  {
    return false;
  }
  this->InitializeUnusedBitsInLastByte();
  return true;
}

//------------------------------------------------------------------------------
// Description:
// Set the tuple at the ith location using the jth tuple in the source array.
// This method assumes that the two arrays have the same type
// and structure. Note that range checking and memory allocation is not
// performed; use in conjunction with SetNumberOfTuples() to allocate space.
void vtkBitArray::SetTuple(vtkIdType i, vtkIdType j, vtkAbstractArray* source)
{
  vtkBitArray* ba = vtkArrayDownCast<vtkBitArray>(source);
  if (!ba)
  {
    vtkWarningMacro("Input and output arrays types do not match.");
    return;
  }

  vtkIdType loci = i * this->NumberOfComponents;
  vtkIdType locj = j * ba->GetNumberOfComponents();
  for (vtkIdType cur = 0; cur < this->NumberOfComponents; cur++)
  {
    this->SetValue(loci + cur, ba->GetValue(locj + cur));
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
// Description:
// Insert the jth tuple in the source array, at ith location in this array.
// Note that memory allocation is performed as necessary to hold the data.
void vtkBitArray::InsertTuple(vtkIdType i, vtkIdType j, vtkAbstractArray* source)
{
  vtkBitArray* ba = vtkArrayDownCast<vtkBitArray>(source);
  if (!ba)
  {
    vtkWarningMacro("Input and output arrays types do not match.");
    return;
  }

  vtkIdType loci = i * this->NumberOfComponents;
  vtkIdType locj = j * ba->GetNumberOfComponents();
  vtkIdType previousMaxId = this->MaxId;
  for (vtkIdType cur = 0; cur < this->NumberOfComponents; cur++)
  {
    this->InsertValue(loci + cur, ba->GetValue(locj + cur));
  }
  if (previousMaxId / 8 != this->MaxId / 8)
  {
    this->InitializeUnusedBitsInLastByte();
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::InsertTuples(vtkIdList* dstIds, vtkIdList* srcIds, vtkAbstractArray* source)
{
  vtkBitArray* ba = vtkArrayDownCast<vtkBitArray>(source);
  if (!ba)
  {
    vtkWarningMacro("Input and output arrays types do not match.");
    return;
  }

  if (ba->NumberOfComponents != this->NumberOfComponents)
  {
    vtkWarningMacro("Number of components do not match.");
    return;
  }

  vtkIdType numIds = dstIds->GetNumberOfIds();
  if (srcIds->GetNumberOfIds() != numIds)
  {
    vtkWarningMacro("Input and output id array sizes do not match.");
    return;
  }

  vtkIdType previousMaxId = this->MaxId;
  for (vtkIdType idIndex = 0; idIndex < numIds; ++idIndex)
  {
    vtkIdType numComp = this->NumberOfComponents;
    vtkIdType srcLoc = srcIds->GetId(idIndex) * this->NumberOfComponents;
    vtkIdType dstLoc = dstIds->GetId(idIndex) * this->NumberOfComponents;
    while (numComp-- > 0)
    {
      this->InsertValue(dstLoc++, ba->GetValue(srcLoc++));
    }
  }
  if (previousMaxId / 8 != this->MaxId / 8)
  {
    this->InitializeUnusedBitsInLastByte();
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::InsertTuples(
  vtkIdType dstStart, vtkIdType n, vtkIdType srcStart, vtkAbstractArray* source)
{
  vtkBitArray* sa = vtkArrayDownCast<vtkBitArray>(source);
  if (!sa)
  {
    vtkWarningMacro("Input and outputs array data types do not match.");
    return;
  }

  if (this->NumberOfComponents != source->GetNumberOfComponents())
  {
    vtkWarningMacro("Input and output component sizes do not match.");
    return;
  }

  vtkIdType srcEnd = srcStart + n;
  if (srcEnd > source->GetNumberOfTuples())
  {
    vtkWarningMacro("Source range exceeds array size (srcStart="
      << srcStart << ", n=" << n << ", numTuples=" << source->GetNumberOfTuples() << ").");
    return;
  }

  vtkIdType previousMaxId = this->MaxId;
  for (vtkIdType i = 0; i < n; ++i)
  {
    vtkIdType numComp = this->NumberOfComponents;
    vtkIdType srcLoc = (srcStart + i) * this->NumberOfComponents;
    vtkIdType dstLoc = (dstStart + i) * this->NumberOfComponents;
    while (numComp-- > 0)
    {
      this->InsertValue(dstLoc++, sa->GetValue(srcLoc++));
    }
  }
  if (previousMaxId / 8 != this->MaxId / 8)
  {
    this->InitializeUnusedBitsInLastByte();
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
// Description:
// Insert the jth tuple in the source array, at the end in this array.
// Note that memory allocation is performed as necessary to hold the data.
// Returns the location at which the data was inserted.
vtkIdType vtkBitArray::InsertNextTuple(vtkIdType j, vtkAbstractArray* source)
{
  vtkBitArray* ba = vtkArrayDownCast<vtkBitArray>(source);
  if (!ba)
  {
    vtkWarningMacro("Input and output arrays types do not match.");
    return -1;
  }

  vtkIdType locj = j * ba->GetNumberOfComponents();
  for (vtkIdType cur = 0; cur < this->NumberOfComponents; cur++)
  {
    this->InsertNextValue(ba->GetValue(locj + cur));
  }
  this->DataChanged();
  return (this->GetNumberOfTuples() - 1);
}

//------------------------------------------------------------------------------
// Get a pointer to a tuple at the ith location. This is a dangerous method
// (it is not thread safe since a pointer is returned).
double* vtkBitArray::GetTuple(vtkIdType i)
{
  if (this->TupleSize < this->NumberOfComponents)
  {
    this->TupleSize = this->NumberOfComponents;
    delete[] this->Tuple;
    this->Tuple = new double[this->TupleSize];
  }

  vtkIdType loc = this->NumberOfComponents * i;
  for (int j = 0; j < this->NumberOfComponents; j++)
  {
    this->Tuple[j] = static_cast<double>(this->GetValue(loc + j));
  }

  return this->Tuple;
}

//------------------------------------------------------------------------------
// Copy the tuple value into a user-provided array.
void vtkBitArray::GetTuple(vtkIdType i, double* tuple)
{
  vtkIdType loc = this->NumberOfComponents * i;

  for (int j = 0; j < this->NumberOfComponents; j++)
  {
    tuple[j] = static_cast<double>(this->GetValue(loc + j));
  }
}

//------------------------------------------------------------------------------
// Set the tuple value at the ith location in the array.
void vtkBitArray::SetTuple(vtkIdType i, const float* tuple)
{
  vtkIdType loc = i * this->NumberOfComponents;

  for (int j = 0; j < this->NumberOfComponents; j++)
  {
    this->SetValue(loc + j, static_cast<int>(tuple[j]));
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::SetTuple(vtkIdType i, const double* tuple)
{
  vtkIdType loc = i * this->NumberOfComponents;

  for (int j = 0; j < this->NumberOfComponents; j++)
  {
    this->SetValue(loc + j, static_cast<int>(tuple[j]));
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
// Insert (memory allocation performed) the tuple into the ith location
// in the array.
void vtkBitArray::InsertTuple(vtkIdType i, const float* tuple)
{
  vtkIdType loc = this->NumberOfComponents * i;

  for (int j = 0; j < this->NumberOfComponents; j++)
  {
    this->InsertValue(loc + j, static_cast<int>(tuple[j]));
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::InsertTuple(vtkIdType i, const double* tuple)
{
  vtkIdType loc = this->NumberOfComponents * i;

  for (int j = 0; j < this->NumberOfComponents; j++)
  {
    this->InsertValue(loc + j, static_cast<int>(tuple[j]));
  }
  this->DataChanged();
}

//------------------------------------------------------------------------------
// Insert (memory allocation performed) the tuple onto the end of the array.
vtkIdType vtkBitArray::InsertNextTuple(const float* tuple)
{
  for (int i = 0; i < this->NumberOfComponents; i++)
  {
    this->InsertNextValue(static_cast<int>(tuple[i]));
  }

  this->DataChanged();
  return this->MaxId / this->NumberOfComponents;
}

//------------------------------------------------------------------------------
vtkIdType vtkBitArray::InsertNextTuple(const double* tuple)
{
  for (int i = 0; i < this->NumberOfComponents; i++)
  {
    this->InsertNextValue(static_cast<int>(tuple[i]));
  }

  this->DataChanged();
  return this->MaxId / this->NumberOfComponents;
}

//------------------------------------------------------------------------------
void vtkBitArray::InsertComponent(vtkIdType i, int j, double c)
{
  this->InsertValue(i * this->NumberOfComponents + j, static_cast<int>(c));
  this->DataChanged();
}

//------------------------------------------------------------------------------
// Set the data component at the ith tuple and jth component location.
// Note that i<NumberOfTuples and j<NumberOfComponents. Make sure enough
// memory has been allocated (use SetNumberOfTuples() and
// SetNumberOfComponents()).
void vtkBitArray::SetComponent(vtkIdType i, int j, double c)
{
  this->SetValue(i * this->NumberOfComponents + j, static_cast<int>(c));
  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::RemoveTuple(vtkIdType id)
{
  if (id < 0 || id >= this->GetNumberOfTuples())
  {
    // Nothing to be done
    return;
  }
  if (id == this->GetNumberOfTuples() - 1)
  {
    // To remove last item, just decrease the size by one
    this->RemoveLastTuple();
    return;
  }
  this->DataChanged();
  vtkErrorMacro("Not yet implemented...");
}

//------------------------------------------------------------------------------
void vtkBitArray::RemoveFirstTuple()
{
  vtkErrorMacro("Not yet implemented...");
  this->RemoveTuple(0);
  this->DataChanged();
}

//------------------------------------------------------------------------------
void vtkBitArray::RemoveLastTuple()
{
  this->Resize(this->GetNumberOfTuples() - 1);
  this->DataChanged();
}

//------------------------------------------------------------------------------
vtkArrayIterator* vtkBitArray::NewIterator()
{
  vtkArrayIterator* iter = vtkBitArrayIterator::New();
  iter->Initialize(this);
  return iter;
}

//------------------------------------------------------------------------------
void vtkBitArray::UpdateLookup()
{
  if (!this->Lookup)
  {
    this->Lookup = new vtkBitArrayLookup();
    this->Lookup->ZeroArray = vtkIdList::New();
    this->Lookup->OneArray = vtkIdList::New();
  }
  if (this->Lookup->Rebuild)
  {
    int numComps = this->GetNumberOfComponents();
    vtkIdType numTuples = this->GetNumberOfTuples();
    this->Lookup->ZeroArray->Allocate(numComps * numTuples);
    this->Lookup->OneArray->Allocate(numComps * numTuples);
    for (vtkIdType i = 0; i < numComps * numTuples; i++)
    {
      if (this->GetValue(i))
      {
        this->Lookup->OneArray->InsertNextId(i);
      }
      else
      {
        this->Lookup->ZeroArray->InsertNextId(i);
      }
    }
    this->Lookup->Rebuild = false;
  }
}

//------------------------------------------------------------------------------
vtkIdType vtkBitArray::LookupValue(vtkVariant var)
{
  return this->LookupValue(var.ToInt());
}

//------------------------------------------------------------------------------
void vtkBitArray::LookupValue(vtkVariant var, vtkIdList* ids)
{
  this->LookupValue(var.ToInt(), ids);
}

//------------------------------------------------------------------------------
vtkIdType vtkBitArray::LookupValue(int value)
{
  this->UpdateLookup();

  if (value == 1 && this->Lookup->OneArray->GetNumberOfIds() > 0)
  {
    return this->Lookup->OneArray->GetId(0);
  }
  else if (value == 0 && this->Lookup->ZeroArray->GetNumberOfIds() > 0)
  {
    return this->Lookup->ZeroArray->GetId(0);
  }
  return -1;
}

//------------------------------------------------------------------------------
void vtkBitArray::LookupValue(int value, vtkIdList* ids)
{
  this->UpdateLookup();

  if (value == 1)
  {
    ids->DeepCopy(this->Lookup->OneArray);
  }
  else if (value == 0)
  {
    ids->DeepCopy(this->Lookup->ZeroArray);
  }
  else
  {
    ids->Reset();
  }
}

//------------------------------------------------------------------------------
void vtkBitArray::DataChanged()
{
  if (this->Lookup)
  {
    this->Lookup->Rebuild = true;
  }
}

//------------------------------------------------------------------------------
void vtkBitArray::ClearLookup()
{
  delete this->Lookup;
  this->Lookup = nullptr;
}
