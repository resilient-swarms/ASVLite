/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkSMPThreadLocalBackend.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "SMP/OpenMP/vtkSMPThreadLocalBackend.h"

#include <omp.h>

#include <algorithm>
#include <cmath> // For std::floor & std::log2

namespace vtk
{
namespace detail
{
namespace smp
{
namespace OpenMP
{

static ThreadIdType GetThreadId()
{
  static int threadPrivateData;
#pragma omp threadprivate(threadPrivateData)
  return &threadPrivateData;
}

// 32 bit FNV-1a hash function
inline HashType GetHash(ThreadIdType id)
{
  const HashType offset_basis = 2166136261u;
  const HashType FNV_prime = 16777619u;

  unsigned char* bp = reinterpret_cast<unsigned char*>(&id);
  unsigned char* be = bp + sizeof(id);
  HashType hval = offset_basis;
  while (bp < be)
  {
    hval ^= static_cast<HashType>(*bp++);
    hval *= FNV_prime;
  }

  return hval;
}

class LockGuard
{
public:
  LockGuard(omp_lock_t& lock, bool wait)
    : Lock(lock)
    , Status(0)
  {
    if (wait)
    {
      omp_set_lock(&this->Lock);
      this->Status = 1;
    }
    else
    {
      this->Status = omp_test_lock(&this->Lock);
    }
  }

  bool Success() const { return this->Status != 0; }

  void Release()
  {
    if (this->Status)
    {
      omp_unset_lock(&this->Lock);
      this->Status = 0;
    }
  }

  ~LockGuard() { this->Release(); }

private:
  // not copyable
  LockGuard(const LockGuard&);
  void operator=(const LockGuard&);

  omp_lock_t& Lock;
  int Status;
};

Slot::Slot()
  : ThreadId(0)
  , Storage(0)
{
  omp_init_lock(&this->ModifyLock);
}

Slot::~Slot()
{
  omp_destroy_lock(&this->ModifyLock);
}

HashTableArray::HashTableArray(size_t sizeLg)
  : Size(1u << sizeLg)
  , SizeLg(sizeLg)
  , NumberOfEntries(0)
  , Prev(nullptr)
{
  this->Slots = new Slot[this->Size];
}

HashTableArray::~HashTableArray()
{
  delete[] this->Slots;
}

// Recursively lookup the slot containing threadId in the HashTableArray
// linked list -- array
static Slot* LookupSlot(HashTableArray* array, ThreadIdType threadId, size_t hash)
{
  if (!array)
  {
    return nullptr;
  }

  size_t mask = array->Size - 1u;
  Slot* slot = nullptr;

  // since load factor is maintained below 0.5, this loop should hit an
  // empty slot if the queried slot does not exist in this array
  for (size_t idx = hash & mask;; idx = (idx + 1) & mask) // linear probing
  {
    slot = array->Slots + idx;
    ThreadIdType slotThreadId = slot->ThreadId.load(); // atomic read
    if (!slotThreadId) // empty slot means threadId doesn't exist in this array
    {
      slot = LookupSlot(array->Prev, threadId, hash);
      break;
    }
    else if (slotThreadId == threadId)
    {
      break;
    }
  }

  return slot;
}

// Lookup threadId. Try to acquire a slot if it doesn't already exist.
// Does not block. Returns nullptr if acquire fails due to high load factor.
// Returns true in 'firstAccess' if threadID did not exist previously.
static Slot* AcquireSlot(
  HashTableArray* array, ThreadIdType threadId, size_t hash, bool& firstAccess)
{
  size_t mask = array->Size - 1u;
  Slot* slot = nullptr;
  firstAccess = false;

  for (size_t idx = hash & mask;; idx = (idx + 1) & mask)
  {
    slot = array->Slots + idx;
    ThreadIdType slotThreadId = slot->ThreadId.load(); // atomic read
    if (!slotThreadId)                                 // unused?
    {
      // empty slot means threadId does not exist, try to acquire the slot
      LockGuard lguard(slot->ModifyLock, false); // try to get exclusive access
      if (lguard.Success())
      {
        size_t size = ++array->NumberOfEntries; // atomic
        if ((size * 2) > array->Size)           // load factor is above threshold
        {
          --array->NumberOfEntries; // atomic revert
          return nullptr;           // indicate need for resizing
        }

        if (!slot->ThreadId.load()) // not acquired in the meantime?
        {
          slot->ThreadId.store(threadId); // atomically acquire
          // check previous arrays for the entry
          Slot* prevSlot = LookupSlot(array->Prev, threadId, hash);
          if (prevSlot)
          {
            slot->Storage = prevSlot->Storage;
            // Do not clear PrevSlot's ThreadId as our technique of stopping
            // linear probing at empty slots relies on slots not being
            // "freed". Instead, clear previous slot's storage pointer as
            // ThreadSpecificStorageIterator relies on this information to
            // ensure that it doesn't iterate over the same thread's storage
            // more than once.
            prevSlot->Storage = nullptr;
          }
          else // first time access
          {
            slot->Storage = nullptr;
            firstAccess = true;
          }
          break;
        }
      }
    }
    else if (slotThreadId == threadId)
    {
      break;
    }
  }

  return slot;
}

ThreadSpecific::ThreadSpecific(unsigned numThreads)
  : Count(0)
{
  const int lastSetBit = (numThreads != 0 ? std::floor(std::log2(numThreads)) : 0);
  // initial size should be more than twice the number of threads
  const size_t initSizeLg = (lastSetBit + 2);
  this->Root = new HashTableArray(initSizeLg);
}

ThreadSpecific::~ThreadSpecific()
{
  HashTableArray* array = this->Root;
  while (array)
  {
    HashTableArray* tofree = array;
    array = array->Prev;
    delete tofree;
  }
}

StoragePointerType& ThreadSpecific::GetStorage()
{
  ThreadIdType threadId = GetThreadId();
  size_t hash = GetHash(threadId);

  Slot* slot = nullptr;
  while (!slot)
  {
    bool firstAccess = false;
    HashTableArray* array = this->Root.load();
    slot = AcquireSlot(array, threadId, hash, firstAccess);
    if (!slot) // not enough room, resize
    {
#pragma omp critical(HashTableResize)
      if (this->Root == array)
      {
        HashTableArray* newArray = new HashTableArray(array->SizeLg + 1);
        newArray->Prev = array;
        this->Root.store(newArray); // atomic copy
      }
    }
    else if (firstAccess)
    {
      ++this->Count; // atomic increment
    }
  }
  return slot->Storage;
}

} // OpenMP
} // namespace smp
} // namespace detail
} // namespace vtk
