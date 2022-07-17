/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCriticalSection.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// Hide VTK_DEPRECATED_IN_9_1_0() warnings for this class.
#define VTK_DEPRECATION_LEVEL 0

#include "vtkSimpleCriticalSection.h"

void vtkSimpleCriticalSection::Init()
{
#ifdef VTK_USE_WIN32_THREADS
  // this->MutexLock = CreateMutex( nullptr, FALSE, nullptr );
  InitializeCriticalSection(&this->CritSec);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_init(&(this->CritSec), nullptr);
#endif
}

vtkSimpleCriticalSection::~vtkSimpleCriticalSection()
{
#ifdef VTK_USE_WIN32_THREADS
  // CloseHandle(this->MutexLock);
  DeleteCriticalSection(&this->CritSec);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_destroy(&this->CritSec);
#endif
}

// Lock the vtkCriticalSection
void vtkSimpleCriticalSection::Lock()
{
#ifdef VTK_USE_WIN32_THREADS
  // WaitForSingleObject( this->MutexLock, INFINITE );
  EnterCriticalSection(&this->CritSec);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_lock(&this->CritSec);
#endif
}

// Unlock the vtkCriticalSection
void vtkSimpleCriticalSection::Unlock()
{
#ifdef VTK_USE_WIN32_THREADS
  // ReleaseMutex( this->MutexLock );
  LeaveCriticalSection(&this->CritSec);
#endif

#ifdef VTK_USE_PTHREADS
  pthread_mutex_unlock(&this->CritSec);
#endif
}
