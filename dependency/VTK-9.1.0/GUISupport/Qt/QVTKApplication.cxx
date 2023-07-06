/*=========================================================================

  Program:   Visualization Toolkit
  Module:    QVTKApplication.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "QVTKApplication.h"

#if defined(VTK_USE_TDX) && (defined(Q_WS_X11) || defined(Q_OS_LINUX))
#include "vtkTDxQtUnixDevices.h"
#include <QWidget>
#include <X11/Xlib.h> // Needed for X types used in the public interface
#endif

//------------------------------------------------------------------------------
QVTKApplication::QVTKApplication(int& Argc, char** Argv)
  : QApplication(Argc, Argv)
{
#if defined(VTK_USE_TDX) && (defined(Q_WS_X11) || defined(Q_OS_LINUX))
  this->Devices = new vtkTDxQtUnixDevices;
  QObject::connect(
    this->Devices, SIGNAL(CreateDevice(vtkTDxDevice*)), this, SLOT(setDevice(vtkTDxDevice*)));
#endif
}

//------------------------------------------------------------------------------
#if defined(VTK_USE_TDX) && (defined(Q_WS_X11) || defined(Q_OS_LINUX))
QVTKApplication::~QVTKApplication()
{
  delete this->Devices;
}
#else
QVTKApplication::~QVTKApplication() = default;
#endif

//------------------------------------------------------------------------------
#if defined(VTK_USE_TDX) && (defined(Q_WS_X11) || defined(Q_OS_LINUX))
bool QVTKApplication::x11EventFilter(XEvent* event)
{
  // the only lines required in this method
  this->Devices->ProcessEvent(static_cast<vtkTDxUnixDeviceXEvent*>(event));
  return false;
}
#endif

//------------------------------------------------------------------------------
#ifdef VTK_USE_TDX
void QVTKApplication::setDevice(vtkTDxDevice* device)
{
#ifdef Q_WS_X11 || Q_OS_LINUX
  Q_EMIT CreateDevice(device);
#else
  (void)device; // to avoid warnings.
#endif
}
#endif
