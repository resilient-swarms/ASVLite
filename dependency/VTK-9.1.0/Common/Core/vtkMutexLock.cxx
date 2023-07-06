/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMutexLock.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// Hide VTK_DEPRECATED_IN_9_1_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkMutexLock.h"
#include "vtkObjectFactory.h"

#ifdef VTK_USE_WIN32_THREADS
#include "vtkWindows.h"
#endif

vtkStandardNewMacro(vtkMutexLock);

// New for the SimpleMutex
vtkSimpleMutexLock* vtkSimpleMutexLock::New()
{
  return new vtkSimpleMutexLock;
}

// Construct a new vtkMutexLock
vtkSimpleMutexLock::vtkSimpleMutexLock()
{
#ifdef VTK_USE_WIN32_THREADS
  this->MutexLock = CreateMutex(nullptr, FALSE, nullptr);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_init(&(this->MutexLock), nullptr);
#endif
}

vtkSimpleMutexLock::~vtkSimpleMutexLock()
{
#ifdef VTK_USE_WIN32_THREADS
  CloseHandle(this->MutexLock);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_destroy(&this->MutexLock);
#endif
}

// Lock the vtkMutexLock
void vtkSimpleMutexLock::Lock()
{
#ifdef VTK_USE_WIN32_THREADS
  WaitForSingleObject(this->MutexLock, INFINITE);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_lock(&this->MutexLock);
#endif
}

// Unlock the vtkMutexLock
void vtkSimpleMutexLock::Unlock()
{
#ifdef VTK_USE_WIN32_THREADS
  ReleaseMutex(this->MutexLock);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_unlock(&this->MutexLock);
#endif
}

void vtkMutexLock::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
