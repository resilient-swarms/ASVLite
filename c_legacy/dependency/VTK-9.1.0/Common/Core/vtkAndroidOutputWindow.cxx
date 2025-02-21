/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkAndroidOutputWindow.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkAndroidOutputWindow.h"

#include "vtkCommand.h"
#include "vtkObjectFactory.h"
#include <sstream>

#include <android/log.h>

vtkStandardNewMacro(vtkAndroidOutputWindow);

//------------------------------------------------------------------------------
vtkAndroidOutputWindow::vtkAndroidOutputWindow() {}

//------------------------------------------------------------------------------
vtkAndroidOutputWindow::~vtkAndroidOutputWindow() {}

//------------------------------------------------------------------------------
void vtkAndroidOutputWindow::DisplayErrorText(const char* someText)
{
  if (!someText)
  {
    return;
  }

  std::istringstream stream(someText);
  std::string line;
  while (std::getline(stream, line))
  {
    __android_log_print(ANDROID_LOG_ERROR, "VTK", "%s", line.c_str());
  }
  this->InvokeEvent(vtkCommand::ErrorEvent, (void*)someText);
}

//------------------------------------------------------------------------------
void vtkAndroidOutputWindow::DisplayWarningText(const char* someText)
{
  if (!someText)
  {
    return;
  }

  std::istringstream stream(someText);
  std::string line;
  while (std::getline(stream, line))
  {
    __android_log_print(ANDROID_LOG_WARN, "VTK", "%s", line.c_str());
  }
  this->InvokeEvent(vtkCommand::WarningEvent, (void*)someText);
}

//------------------------------------------------------------------------------
void vtkAndroidOutputWindow::DisplayGenericWarningText(const char* someText)
{
  if (!someText)
  {
    return;
  }

  std::istringstream stream(someText);
  std::string line;
  while (std::getline(stream, line))
  {
    __android_log_print(ANDROID_LOG_WARN, "VTK", "%s", line.c_str());
  }
}

//------------------------------------------------------------------------------
void vtkAndroidOutputWindow::DisplayDebugText(const char* someText)
{
  if (!someText)
  {
    return;
  }

  std::istringstream stream(someText);
  std::string line;
  while (std::getline(stream, line))
  {
    __android_log_print(ANDROID_LOG_DEBUG, "VTK", "%s", line.c_str());
  }
}

//------------------------------------------------------------------------------
void vtkAndroidOutputWindow::DisplayText(const char* someText)
{
  if (!someText)
  {
    return;
  }

  std::istringstream stream(someText);
  std::string line;
  while (std::getline(stream, line))
  {
    __android_log_print(ANDROID_LOG_INFO, "VTK", "%s", line.c_str());
  }
}

//------------------------------------------------------------------------------
void vtkAndroidOutputWindow::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
