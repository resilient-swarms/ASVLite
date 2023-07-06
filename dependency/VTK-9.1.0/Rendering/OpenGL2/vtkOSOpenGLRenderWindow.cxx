/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkOSOpenGLRenderWindow.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtk_glew.h"
#include <GL/gl.h>

#ifndef GLAPI
#define GLAPI extern
#endif

#ifndef GLAPIENTRY
#define GLAPIENTRY
#endif

#ifndef APIENTRY
#define APIENTRY GLAPIENTRY
#endif
#include <GL/osmesa.h>

#include "vtkOSOpenGLRenderWindow.h"
#include "vtkOpenGLActor.h"
#include "vtkOpenGLCamera.h"
#include "vtkOpenGLLight.h"
#include "vtkOpenGLProperty.h"
#include "vtkOpenGLRenderer.h"
#include "vtkOpenGLTexture.h"

#include "vtkCommand.h"
#include "vtkIdList.h"
#include "vtkObjectFactory.h"
#include "vtkRendererCollection.h"

#include "vtksys/SystemTools.hxx"
#include <sstream>

class vtkOSOpenGLRenderWindow;
class vtkRenderWindow;

typedef OSMesaContext GLAPIENTRY (*OSMesaCreateContextAttribs_func)(
  const int* attribList, OSMesaContext sharelist);

class vtkOSOpenGLRenderWindowInternal
{
  friend class vtkOSOpenGLRenderWindow;

private:
  vtkOSOpenGLRenderWindowInternal();

  // OffScreen stuff
  OSMesaContext OffScreenContextId;
  void* OffScreenWindow;
};

vtkOSOpenGLRenderWindowInternal::vtkOSOpenGLRenderWindowInternal()
{
  // OpenGL specific
  this->OffScreenContextId = nullptr;
  this->OffScreenWindow = nullptr;
}

vtkStandardNewMacro(vtkOSOpenGLRenderWindow);

// a couple of routines for offscreen rendering
void vtkOSMesaDestroyWindow(void* Window)
{
  free(Window);
}

void* vtkOSMesaCreateWindow(int width, int height)
{
  return malloc(width * height * 4);
}

vtkOSOpenGLRenderWindow::vtkOSOpenGLRenderWindow()
{
  //   this->ParentId = (Window)nullptr;
  this->ScreenSize[0] = 1280;
  this->ScreenSize[1] = 1024;
  this->OwnDisplay = 0;
  this->CursorHidden = 0;
  this->ForceMakeCurrent = 0;
  this->OwnWindow = 0;
  this->ShowWindow = false;
  this->UseOffScreenBuffers = true;

  this->Internal = new vtkOSOpenGLRenderWindowInternal();
}

// free up memory & close the window
vtkOSOpenGLRenderWindow::~vtkOSOpenGLRenderWindow()
{
  // close-down all system-specific drawing resources
  this->Finalize();
  vtkRenderer* ren;
  vtkCollectionSimpleIterator rit;
  this->Renderers->InitTraversal(rit);
  while ((ren = this->Renderers->GetNextRenderer(rit)))
  {
    ren->SetRenderWindow(nullptr);
  }

  delete this->Internal;
}

// End the rendering process and display the image.
void vtkOSOpenGLRenderWindow::Frame()
{
  this->MakeCurrent();
  this->Superclass::Frame();
}

//
// Set the variable that indicates that we want a stereo capable window
// be created. This method can only be called before a window is realized.
//
void vtkOSOpenGLRenderWindow::SetStereoCapableWindow(vtkTypeBool capable)
{
  if (!this->Internal->OffScreenContextId)
  {
    vtkOpenGLRenderWindow::SetStereoCapableWindow(capable);
  }
  else
  {
    vtkWarningMacro(<< "Requesting a StereoCapableWindow must be performed "
                    << "before the window is realized, i.e. before a render.");
  }
}

void vtkOSOpenGLRenderWindow::CreateAWindow()
{
  this->CreateOffScreenWindow(this->Size[0], this->Size[1]);
}

void vtkOSOpenGLRenderWindow::DestroyWindow()
{
  this->MakeCurrent();
  this->ReleaseGraphicsResources(this);

  delete[] this->Capabilities;
  this->Capabilities = 0;

  this->DestroyOffScreenWindow();

  // make sure all other code knows we're not mapped anymore
  this->Mapped = 0;
}

void vtkOSOpenGLRenderWindow::CreateOffScreenWindow(int width, int height)
{
  this->DoubleBuffer = 0;

  if (!this->Internal->OffScreenWindow)
  {
    this->Internal->OffScreenWindow = vtkOSMesaCreateWindow(width, height);
    this->OwnWindow = 1;
  }
#if (OSMESA_MAJOR_VERSION * 100 + OSMESA_MINOR_VERSION >= 1102) &&                                 \
  defined(OSMESA_CONTEXT_MAJOR_VERSION)
  if (!this->Internal->OffScreenContextId)
  {
    static const int attribs[] = { OSMESA_FORMAT, OSMESA_RGBA, OSMESA_DEPTH_BITS, 32,
      OSMESA_STENCIL_BITS, 0, OSMESA_ACCUM_BITS, 0, OSMESA_PROFILE, OSMESA_CORE_PROFILE,
      OSMESA_CONTEXT_MAJOR_VERSION, 3, OSMESA_CONTEXT_MINOR_VERSION, 2, 0 };

    OSMesaCreateContextAttribs_func OSMesaCreateContextAttribs =
      (OSMesaCreateContextAttribs_func)OSMesaGetProcAddress("OSMesaCreateContextAttribs");

    if (OSMesaCreateContextAttribs != nullptr)
    {
      this->Internal->OffScreenContextId = OSMesaCreateContextAttribs(attribs, nullptr);
    }
  }
#endif
  // if we still have no context fall back to the generic signature
  if (!this->Internal->OffScreenContextId)
  {
    this->Internal->OffScreenContextId = OSMesaCreateContext(GL_RGBA, nullptr);
  }
  this->MakeCurrent();

  this->Mapped = 0;
  this->Size[0] = width;
  this->Size[1] = height;

  this->MakeCurrent();

  // tell our renderers about us
  vtkRenderer* ren;
  for (this->Renderers->InitTraversal(); (ren = this->Renderers->GetNextItem());)
  {
    ren->SetRenderWindow(0);
    ren->SetRenderWindow(this);
  }

  this->OpenGLInit();
}

void vtkOSOpenGLRenderWindow::DestroyOffScreenWindow()
{
  // Release graphic resources.

  // First release graphics resources on the window itself
  // since call to Renderer's SetRenderWindow(nullptr), just
  // calls ReleaseGraphicsResources on vtkProps. And also
  // this call invokes Renderer's ReleaseGraphicsResources
  // method which only invokes ReleaseGraphicsResources on
  // rendering passes.
  this->ReleaseGraphicsResources(this);

  if (this->Internal->OffScreenContextId)
  {
    OSMesaDestroyContext(this->Internal->OffScreenContextId);
    this->Internal->OffScreenContextId = nullptr;
    vtkOSMesaDestroyWindow(this->Internal->OffScreenWindow);
    this->Internal->OffScreenWindow = nullptr;
  }
}

void vtkOSOpenGLRenderWindow::ResizeOffScreenWindow(int width, int height)
{
  if (this->Internal->OffScreenContextId)
  {
    this->DestroyOffScreenWindow();
    this->CreateOffScreenWindow(width, height);
  }
}

// Initialize the window for rendering.
void vtkOSOpenGLRenderWindow::WindowInitialize(void)
{
  this->CreateAWindow();

  this->MakeCurrent();

  // tell our renderers about us
  vtkRenderer* ren;
  for (this->Renderers->InitTraversal(); (ren = this->Renderers->GetNextItem());)
  {
    ren->SetRenderWindow(0);
    ren->SetRenderWindow(this);
  }

  this->OpenGLInit();
}

// Initialize the rendering window.
void vtkOSOpenGLRenderWindow::Initialize(void)
{
  if (!(this->Internal->OffScreenContextId))
  {
    // initialize offscreen window
    int width = ((this->Size[0] > 0) ? this->Size[0] : 300);
    int height = ((this->Size[1] > 0) ? this->Size[1] : 300);
    this->CreateOffScreenWindow(width, height);
  }
}

void vtkOSOpenGLRenderWindow::Finalize(void)
{
  // clean and destroy window
  this->DestroyWindow();
}

// Change the window to fill the entire screen.
void vtkOSOpenGLRenderWindow::SetFullScreen(vtkTypeBool arg)
{
  (void)arg;
  this->Modified();
}

// Resize the window.
void vtkOSOpenGLRenderWindow::WindowRemap()
{
  // shut everything down
  this->Finalize();

  // set everything up again
  this->Initialize();
}

// Specify the size of the rendering window.
void vtkOSOpenGLRenderWindow::SetSize(int width, int height)
{
  if ((this->Size[0] != width) || (this->Size[1] != height))
  {
    this->Superclass::SetSize(width, height);
    this->ResizeOffScreenWindow(width, height);
    this->Modified();
  }
}

void vtkOSOpenGLRenderWindow::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "OffScreenContextId: " << this->Internal->OffScreenContextId << "\n";
}

void vtkOSOpenGLRenderWindow::MakeCurrent()
{
  // set the current window
  if (this->Internal->OffScreenContextId)
  {
    if (OSMesaMakeCurrent(this->Internal->OffScreenContextId, this->Internal->OffScreenWindow,
          GL_UNSIGNED_BYTE, this->Size[0], this->Size[1]) != GL_TRUE)
    {
      vtkWarningMacro("failed call to OSMesaMakeCurrent");
    }
  }
}

//------------------------------------------------------------------------------
// Description:
// Tells if this window is the current OpenGL context for the calling thread.
bool vtkOSOpenGLRenderWindow::IsCurrent()
{
  bool result = false;
  if (this->Internal->OffScreenContextId)
  {
    result = this->Internal->OffScreenContextId == OSMesaGetCurrentContext();
  }
  return result;
}

void vtkOSOpenGLRenderWindow::SetForceMakeCurrent()
{
  this->ForceMakeCurrent = 1;
}

void* vtkOSOpenGLRenderWindow::GetGenericContext()
{
  return (void*)this->Internal->OffScreenContextId;
}

vtkTypeBool vtkOSOpenGLRenderWindow::GetEventPending()
{
  return 0;
}

// Get the size of the screen in pixels
int* vtkOSOpenGLRenderWindow::GetScreenSize()
{
  this->ScreenSize[0] = 1280;
  this->ScreenSize[1] = 1024;
  return this->ScreenSize;
}

// Get the position in screen coordinates (pixels) of the window.
int* vtkOSOpenGLRenderWindow::GetPosition(void)
{
  return this->Position;
}

// Move the window to a new position on the display.
void vtkOSOpenGLRenderWindow::SetPosition(int x, int y)
{
  if ((this->Position[0] != x) || (this->Position[1] != y))
  {
    this->Modified();
  }
  this->Position[0] = x;
  this->Position[1] = y;
}

// Set this RenderWindow's X window id to a pre-existing window.
void vtkOSOpenGLRenderWindow::SetWindowInfo(const char* info)
{
  int tmp;

  this->OwnDisplay = 1;

  sscanf(info, "%i", &tmp);
}

// Set this RenderWindow's X window id to a pre-existing window.
void vtkOSOpenGLRenderWindow::SetNextWindowInfo(const char* info)
{
  int tmp;
  sscanf(info, "%i", &tmp);

  //   this->SetNextWindowId((Window)tmp);
}

// Sets the X window id of the window that WILL BE created.
void vtkOSOpenGLRenderWindow::SetParentInfo(const char* info)
{
  int tmp;

  // get the default display connection
  this->OwnDisplay = 1;

  sscanf(info, "%i", &tmp);

  //   this->SetParentId(tmp);
}

void vtkOSOpenGLRenderWindow::SetWindowId(void* arg)
{
  (void)arg;
  //   this->SetWindowId((Window)arg);
}
void vtkOSOpenGLRenderWindow::SetParentId(void* arg)
{
  (void)arg;
  //   this->SetParentId((Window)arg);
}

const char* vtkOSOpenGLRenderWindow::ReportCapabilities()
{
  MakeCurrent();

  //   int scrnum = DefaultScreen(this->DisplayId);

  const char* glVendor = (const char*)glGetString(GL_VENDOR);
  const char* glRenderer = (const char*)glGetString(GL_RENDERER);
  const char* glVersion = (const char*)glGetString(GL_VERSION);
  const char* glExtensions = (const char*)glGetString(GL_EXTENSIONS);

  std::ostringstream strm;
  strm << "OpenGL vendor string:  " << glVendor << endl;
  strm << "OpenGL renderer string:  " << glRenderer << endl;
  strm << "OpenGL version string:  " << glVersion << endl;
  strm << "OpenGL extensions:  " << glExtensions << endl;
  delete[] this->Capabilities;
  size_t len = strm.str().length();
  this->Capabilities = new char[len + 1];
  strncpy(this->Capabilities, strm.str().c_str(), len);
  this->Capabilities[len] = 0;
  return this->Capabilities;
}

int vtkOSOpenGLRenderWindow::SupportsOpenGL()
{
  MakeCurrent();
  return 1;
}

vtkTypeBool vtkOSOpenGLRenderWindow::IsDirect()
{
  MakeCurrent();
  return 0;
}

void vtkOSOpenGLRenderWindow::SetWindowName(const char* cname)
{
  char* name = new char[strlen(cname) + 1];
  strcpy(name, cname);
  vtkOpenGLRenderWindow::SetWindowName(name);
  delete[] name;
}

void vtkOSOpenGLRenderWindow::SetNextWindowId(void* arg)
{
  (void)arg;
}

// This probably has been moved to superclass.
void* vtkOSOpenGLRenderWindow::GetGenericWindowId()
{
  return (void*)this->Internal->OffScreenWindow;
}
