/*=========================================================================

 Program:   Visualization Toolkit
 Module:    vtkSMPThreadLocalImpl.h

 Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
 All rights reserved.
 See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

    This software is distributed WITHOUT ANY WARRANTY; without even
    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
    PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkSMPThreadLocal - A simple thread local implementation for sequential operations.
// .SECTION Description
//
// Note that this particular implementation is designed to work in sequential
// mode and supports only 1 thread.

#ifndef SequentialvtkSMPThreadLocalImpl_h
#define SequentialvtkSMPThreadLocalImpl_h

#include "SMP/Common/vtkSMPThreadLocalImplAbstract.h"
#include "vtkSystemIncludes.h"

#include <iterator>
#include <utility> // For std::move
#include <vector>

namespace vtk
{
namespace detail
{
namespace smp
{

template <typename T>
class vtkSMPThreadLocalImpl<BackendType::Sequential, T> : public vtkSMPThreadLocalImplAbstract<T>
{
  typedef std::vector<T> TLS;
  typedef typename TLS::iterator TLSIter;
  typedef typename vtkSMPThreadLocalImplAbstract<T>::ItImpl ItImplAbstract;

public:
  vtkSMPThreadLocalImpl()
    : NumInitialized(0)
  {
    this->Initialize();
  }

  explicit vtkSMPThreadLocalImpl(const T& exemplar)
    : NumInitialized(0)
    , Exemplar(exemplar)
  {
    this->Initialize();
  }

  T& Local() override
  {
    int tid = this->GetThreadID();
    if (!this->Initialized[tid])
    {
      this->Internal[tid] = this->Exemplar;
      this->Initialized[tid] = true;
      ++this->NumInitialized;
    }
    return this->Internal[tid];
  }

  size_t size() const override { return this->NumInitialized; }

  class ItImpl : public vtkSMPThreadLocalImplAbstract<T>::ItImpl
  {
  public:
    void Increment() override
    {
      this->InitIter++;
      this->Iter++;

      // Make sure to skip uninitialized
      // entries.
      while (this->InitIter != this->EndIter)
      {
        if (*this->InitIter)
        {
          break;
        }
        this->InitIter++;
        this->Iter++;
      }
    }

    bool Compare(ItImplAbstract* other) override
    {
      return this->Iter == static_cast<ItImpl*>(other)->Iter;
    }

    T& GetContent() override { return *this->Iter; }

    T* GetContentPtr() override { return &*this->Iter; }

  protected:
    virtual ItImpl* CloneImpl() const override { return new ItImpl(*this); };

  private:
    friend class vtkSMPThreadLocalImpl<BackendType::Sequential, T>;
    std::vector<bool>::iterator InitIter;
    std::vector<bool>::iterator EndIter;
    TLSIter Iter;
  };

  std::unique_ptr<ItImplAbstract> begin() override
  {
    TLSIter iter = this->Internal.begin();
    std::vector<bool>::iterator iter2 = this->Initialized.begin();
    std::vector<bool>::iterator enditer = this->Initialized.end();
    // fast forward to first initialized
    // value
    while (iter2 != enditer)
    {
      if (*iter2)
      {
        break;
      }
      iter2++;
      iter++;
    }
    // XXX(c++14): use std::make_unique
    auto retVal = std::unique_ptr<ItImpl>(new ItImpl());
    retVal->InitIter = iter2;
    retVal->EndIter = enditer;
    retVal->Iter = iter;
    // XXX(c++14): remove std::move and cast variable
    std::unique_ptr<ItImplAbstract> abstractIt(std::move(retVal));
    return abstractIt;
  };

  std::unique_ptr<ItImplAbstract> end() override
  {
    // XXX(c++14): use std::make_unique
    auto retVal = std::unique_ptr<ItImpl>(new ItImpl());
    retVal->InitIter = this->Initialized.end();
    retVal->EndIter = this->Initialized.end();
    retVal->Iter = this->Internal.end();
    // XXX(c++14): remove std::move and cast variable
    std::unique_ptr<ItImplAbstract> abstractIt(std::move(retVal));
    return abstractIt;
  }

private:
  TLS Internal;
  std::vector<bool> Initialized;
  size_t NumInitialized;
  T Exemplar;

  void Initialize()
  {
    this->Internal.resize(this->GetNumberOfThreads());
    this->Initialized.resize(this->GetNumberOfThreads());
    std::fill(this->Initialized.begin(), this->Initialized.end(), false);
  }

  inline int GetNumberOfThreads() { return 1; }

  inline int GetThreadID() { return 0; }

  // disable copying
  vtkSMPThreadLocalImpl(const vtkSMPThreadLocalImpl&) = delete;
  void operator=(const vtkSMPThreadLocalImpl&) = delete;
};

} // namespace smp
} // namespace detail
} // namespace vtk

#endif
