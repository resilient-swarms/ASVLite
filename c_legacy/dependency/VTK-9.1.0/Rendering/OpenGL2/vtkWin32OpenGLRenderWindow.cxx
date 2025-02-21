/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkWin32OpenGLRenderWindow.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkWin32OpenGLRenderWindow.h"

#include "vtkCommand.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkIdList.h"
#include "vtkImageData.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLError.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkOpenGLRenderer.h"
#include "vtkOpenGLShaderCache.h"
#include "vtkOpenGLState.h"
#include "vtkOpenGLVertexBufferObjectCache.h"
#include "vtkRendererCollection.h"
#include "vtkStringOutputWindow.h"
#include "vtkWin32RenderWindowInteractor.h"

#include <cmath>
#include <sstream>
#include <vector>

#include "vtkOpenGLError.h"

#include "vtksys/Encoding.hxx"

// Mouse wheel support
// In an ideal world we would just have to include <zmouse.h>, but it is not
// always available with all compilers/headers
#ifndef WM_MOUSEWHEEL
#define WM_MOUSEWHEEL 0x020A
#endif // WM_MOUSEWHEEL

vtkStandardNewMacro(vtkWin32OpenGLRenderWindow);

const std::string vtkWin32OpenGLRenderWindow::DEFAULT_BASE_WINDOW_NAME =
  "Visualization Toolkit - Win32OpenGL #";

vtkWin32OpenGLRenderWindow::vtkWin32OpenGLRenderWindow()
{
  this->ApplicationInstance = nullptr;
  this->Palette = nullptr;
  this->ContextId = 0;
  this->WindowId = 0;
  this->ParentId = 0;
  this->NextWindowId = 0;
  this->DeviceContext = (HDC)0;   // hsr
  this->MFChandledWindow = FALSE; // hsr
  this->StereoType = VTK_STEREO_CRYSTAL_EYES;
  this->CursorHidden = 0;
  this->Resizing = 0;
  this->Repositioning = 0;

  this->WindowIdReferenceCount = 0;

  this->SetWindowName(DEFAULT_BASE_WINDOW_NAME.c_str());
}

//------------------------------------------------------------------------------
vtkWin32OpenGLRenderWindow::~vtkWin32OpenGLRenderWindow()
{
  this->Finalize();

  vtkRenderer* ren;
  vtkCollectionSimpleIterator rit;
  this->Renderers->InitTraversal(rit);
  while ((ren = this->Renderers->GetNextRenderer(rit)))
  {
    ren->SetRenderWindow(nullptr);
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::Clean()
{
  /* finish OpenGL rendering */
  if (this->OwnContext && this->ContextId)
  {
    this->MakeCurrent();
    this->CleanUpRenderers();

    // Note: wglMakeCurrent(nullptr,nullptr) is valid according to the documentation
    // and works with nVidia and ATI but not with Intel. Passing an existing
    // device context works in any case.
    // see VTK Bug 7119.
    if (wglMakeCurrent(this->DeviceContext, nullptr) != TRUE)
    {
      vtkErrorMacro("wglMakeCurrent failed in Clean(), error: " << GetLastError());
    }
    if (wglDeleteContext(this->ContextId) != TRUE)
    {
      vtkErrorMacro("wglDeleteContext failed in Clean(), error: " << GetLastError());
    }
  }
  this->ContextId = nullptr;

  if (this->Palette)
  {
    SelectPalette(this->DeviceContext, this->OldPalette, FALSE); // SVA delete the old palette
    DeleteObject(this->Palette);
    this->Palette = nullptr;
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::CleanUpRenderers()
{
  // tell each of the renderers that this render window/graphics context
  // is being removed (the RendererCollection is removed by vtkRenderWindow's
  // destructor)
  this->ReleaseGraphicsResources(this);
}

//------------------------------------------------------------------------------
LRESULT APIENTRY vtkWin32OpenGLRenderWindow::WndProc(
  HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  LRESULT res;

  vtkWin32OpenGLRenderWindow* me =
    (vtkWin32OpenGLRenderWindow*)vtkGetWindowLong(hWnd, sizeof(vtkLONG));

  if (me && me->GetReferenceCount() > 0)
  {
    me->Register(me);
    res = me->MessageProc(hWnd, message, wParam, lParam);
    me->UnRegister(me);
  }
  else
  {
    res = DefWindowProc(hWnd, message, wParam, lParam);
  }

  return res;
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetWindowName(const char* _arg)
{
  vtkWindow::SetWindowName(_arg);
  if (this->WindowId)
  {
    std::wstring wname = vtksys::Encoding::ToWide(this->WindowName);
    SetWindowTextW(this->WindowId, wname.c_str());
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetIcon(vtkImageData* img)
{
  int dim[3];
  img->GetDimensions(dim);

  int nbComp = img->GetNumberOfScalarComponents();

  if (img->GetScalarType() != VTK_UNSIGNED_CHAR || dim[2] != 1 || nbComp < 3 || nbComp > 4)
  {
    vtkErrorMacro(
      "Icon image should be 2D, have 3 or 4 components, and its type must be unsigned char.");
    return;
  }

  unsigned char* imgScalars = static_cast<unsigned char*>(img->GetScalarPointer());

  std::vector<unsigned char> pixels(nbComp * dim[0] * dim[1]);

  // Convert vtkImageData buffer to HBITMAP.
  // We need to flip Y and swap R and B channel
  for (int col = 0; col < dim[1]; col++)
  {
    for (int line = 0; line < dim[0]; line++)
    {
      unsigned char* inPixel = imgScalars + nbComp * ((dim[0] - col - 1) * dim[1] + line); // flip Y
      unsigned char* outPixel = pixels.data() + nbComp * (col * dim[1] + line);
      outPixel[0] = inPixel[2]; // swap R and B channel
      outPixel[1] = inPixel[1];
      outPixel[2] = inPixel[0]; // swap R and B channel
      outPixel[3] = inPixel[3];
    }
  }

  HBITMAP bmp = CreateBitmap(dim[0], dim[1], 1, nbComp * 8, pixels.data());

  HDC dc = GetDC(NULL);
  HBITMAP bmpMask = CreateCompatibleBitmap(dc, dim[0], dim[1]);

  ICONINFO ii;
  ii.fIcon = TRUE;
  ii.hbmMask = bmpMask;
  ii.hbmColor = bmp;

  HICON icon = CreateIconIndirect(&ii);

  SendMessage(this->WindowId, WM_SETICON, ICON_BIG, (LPARAM)icon);

  DeleteObject(bmpMask);
  DeleteObject(bmp);
  DestroyIcon(icon);
  ReleaseDC(NULL, dc);
}

//------------------------------------------------------------------------------
vtkTypeBool vtkWin32OpenGLRenderWindow::GetEventPending()
{
  MSG msg;
  if (PeekMessage(&msg, this->WindowId, WM_MOUSEFIRST, WM_MOUSELAST, PM_NOREMOVE))
  {
    if (msg.message == WM_MOUSEMOVE)
    {
      PeekMessage(&msg, this->WindowId, WM_MOUSEFIRST, WM_MOUSELAST, PM_REMOVE);
    }
    if ((msg.message == WM_LBUTTONDOWN) || (msg.message == WM_RBUTTONDOWN) ||
      (msg.message == WM_MBUTTONDOWN) || (msg.message == WM_MOUSEWHEEL))
    {
      return 1;
    }
  }

  return 0;
}

//------------------------------------------------------------------------------
bool vtkWin32OpenGLRenderWindow::InitializeFromCurrentContext()
{
  HGLRC currentContext = wglGetCurrentContext();
  if (currentContext != nullptr)
  {
    this->SetWindowId(WindowFromDC(wglGetCurrentDC()));
    this->SetDeviceContext(wglGetCurrentDC());
    this->SetContextId(currentContext);
    return this->Superclass::InitializeFromCurrentContext();
  }
  return false;
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::MakeCurrent()
{
  // Try to avoid doing anything (for performance).
  HGLRC current = wglGetCurrentContext();
  if (this->ContextId != current)
  {
    if (wglMakeCurrent(this->DeviceContext, this->ContextId) != TRUE)
    {
      LPVOID lpMsgBuf;
      ::FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
        (LPWSTR)&lpMsgBuf, 0, nullptr);
      if (lpMsgBuf)
      {
        std::string message = vtksys::Encoding::ToNarrow((LPWSTR)&lpMsgBuf);
        vtkErrorMacro("wglMakeCurrent failed in MakeCurrent(), error: " << message.c_str());
        ::LocalFree(lpMsgBuf);
      }
    }
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::ReleaseCurrent()
{
  // Try to avoid doing anything (for performance).
  HGLRC current = wglGetCurrentContext();
  if (this->ContextId == current && this->DeviceContext)
  {
    wglMakeCurrent(this->DeviceContext, nullptr);
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::PushContext()
{
  HGLRC current = wglGetCurrentContext();
  this->ContextStack.push(current);
  this->DCStack.push(wglGetCurrentDC());
  if (current != this->ContextId)
  {
    this->MakeCurrent();
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::PopContext()
{
  HGLRC current = wglGetCurrentContext();
  HGLRC target = this->ContextStack.top();
  HDC dc = this->DCStack.top();
  this->ContextStack.pop();
  this->DCStack.pop();
  if (target != current)
  {
    wglMakeCurrent(dc, target);
  }
}

//------------------------------------------------------------------------------
// Description:
// Tells if this window is the current OpenGL context for the calling thread.
bool vtkWin32OpenGLRenderWindow::IsCurrent()
{
  return this->ContextId != 0 && this->ContextId == wglGetCurrentContext();
}

//------------------------------------------------------------------------------
bool vtkWin32OpenGLRenderWindow::SetSwapControl(int i)
{
  if (!wglewIsSupported("WGL_EXT_swap_control"))
  {
    return false;
  }

  if (i < 0)
  {
    if (wglewIsSupported("WGL_EXT_swap_control_tear"))
    {
      wglSwapIntervalEXT(i);
      return true;
    }
    return false;
  }

  wglSwapIntervalEXT(i);
  return true;
}

//------------------------------------------------------------------------------
namespace
{
void AdjustWindowRectForBorders(
  HWND hwnd, DWORD style, const int x, const int y, const int width, const int height, RECT& r)
{
  if (!style && hwnd)
  {
    style = GetWindowLong(hwnd, GWL_STYLE);
  }
  r.left = x;
  r.top = y;
  r.right = r.left + width;
  r.bottom = r.top + height;
  BOOL result = AdjustWindowRect(&r, style, FALSE);
  if (!result)
  {
    vtkGenericWarningMacro("AdjustWindowRect failed, error: " << GetLastError());
  }
}
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetSize(int width, int height)
{
  if ((this->Size[0] != width) || (this->Size[1] != height))
  {
    this->Superclass::SetSize(width, height);

    if (this->Interactor)
    {
      this->Interactor->SetSize(width, height);
    }

    if (!this->UseOffScreenBuffers)
    {
      if (!this->Resizing)
      {
        this->Resizing = 1;

        if (this->ParentId)
        {
          SetWindowExtEx(this->DeviceContext, width, height, nullptr);
          SetViewportExtEx(this->DeviceContext, width, height, nullptr);
          SetWindowPos(this->WindowId, HWND_TOP, 0, 0, width, height, SWP_NOMOVE | SWP_NOZORDER);
        }
        else
        {
          RECT r;
          AdjustWindowRectForBorders(this->WindowId, 0, 0, 0, width, height, r);
          SetWindowPos(this->WindowId, HWND_TOP, 0, 0, r.right - r.left, r.bottom - r.top,
            SWP_NOMOVE | SWP_NOZORDER);
        }

        this->Resizing = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetPosition(int x, int y)
{
  if ((this->Position[0] != x) || (this->Position[1] != y))
  {
    this->Modified();
    this->Position[0] = x;
    this->Position[1] = y;
    if (this->Mapped)
    {
      if (!this->Repositioning)
      {
        this->Repositioning = 1;

        SetWindowPos(this->WindowId, HWND_TOP, x, y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

        this->Repositioning = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
// End the rendering process and display the image.
void vtkWin32OpenGLRenderWindow::Frame(void)
{
  this->MakeCurrent();
  this->Superclass::Frame();

  if (!this->AbortRender && this->DoubleBuffer && this->SwapBuffers)
  {
    // If this check is not enforced, we crash in offscreen rendering
    if (this->DeviceContext && !this->UseOffScreenBuffers)
    {
      // use global scope to get Win32 API SwapBuffers and not be
      // confused with this->SwapBuffers
      ::SwapBuffers(this->DeviceContext);
      vtkDebugMacro(<< " SwapBuffers\n");
    }
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::VTKRegisterClass()
{
  // has the class been registered ?
  WNDCLASSA wndClass;
  if (!GetClassInfoA(this->ApplicationInstance, "vtkOpenGL", &wndClass))
  {
    wndClass.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
    wndClass.lpfnWndProc = vtkWin32OpenGLRenderWindow::WndProc;
    wndClass.cbClsExtra = 0;
    wndClass.hInstance = this->ApplicationInstance;
    wndClass.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    wndClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wndClass.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wndClass.lpszMenuName = nullptr;
    wndClass.lpszClassName = "vtkOpenGL";
    // vtk doesn't use the first extra vtkLONG's worth of bytes,
    // but app writers may want them, so we provide them. VTK
    // does use the second vtkLONG's worth of bytes of extra space.
    wndClass.cbWndExtra = 2 * sizeof(vtkLONG);
    RegisterClassA(&wndClass);
  }
}

//------------------------------------------------------------------------------
vtkTypeBool vtkWin32OpenGLRenderWindow::IsDirect()
{
  this->MakeCurrent();
  if (!this->DeviceContext)
  {
    return 0;
  }

  int pixelFormat = GetPixelFormat(this->DeviceContext);
  PIXELFORMATDESCRIPTOR pfd;

  DescribePixelFormat(this->DeviceContext, pixelFormat, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

  return (pfd.dwFlags & PFD_GENERIC_FORMAT) ? 0 : 1;
}

//------------------------------------------------------------------------------
const char* vtkWin32OpenGLRenderWindow::ReportCapabilities()
{
  this->MakeCurrent();

  if (!this->DeviceContext)
  {
    return "no device context";
  }

  int pixelFormat = GetPixelFormat(this->DeviceContext);
  PIXELFORMATDESCRIPTOR pfd;

  DescribePixelFormat(this->DeviceContext, pixelFormat, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

  const char* glVendor = (const char*)glGetString(GL_VENDOR);
  const char* glRenderer = (const char*)glGetString(GL_RENDERER);
  const char* glVersion = (const char*)glGetString(GL_VERSION);

  std::ostringstream strm;
  if (glVendor)
    strm << "OpenGL vendor string:  " << glVendor << endl;
  if (glRenderer)
    strm << "OpenGL renderer string:  " << glRenderer << endl;
  if (glVersion)
    strm << "OpenGL version string:  " << glVersion << endl;

  strm << "OpenGL extensions:  " << endl;
  GLint n, i;
  glGetIntegerv(GL_NUM_EXTENSIONS, &n);
  for (i = 0; i < n; i++)
  {
    const char* ext = (const char*)glGetStringi(GL_EXTENSIONS, i);
    strm << "  " << ext << endl;
  }
  strm << "PixelFormat Descriptor:" << endl;
  strm << "depth:  " << static_cast<int>(pfd.cDepthBits) << endl;
  if (pfd.cColorBits <= 8)
  {
    strm << "class:  PseudoColor" << endl;
  }
  else
  {
    strm << "class:  TrueColor" << endl;
  }
  strm << "buffer size:  " << static_cast<int>(pfd.cColorBits) << endl;
  strm << "level:  " << static_cast<int>(pfd.bReserved) << endl;
  if (pfd.iPixelType == PFD_TYPE_RGBA)
  {
    strm << "renderType:  rgba" << endl;
  }
  else
  {
    strm << "renderType:  ci" << endl;
  }
  if (pfd.dwFlags & PFD_DOUBLEBUFFER)
  {
    strm << "double buffer:  True" << endl;
  }
  else
  {
    strm << "double buffer:  False" << endl;
  }
  if (pfd.dwFlags & PFD_STEREO)
  {
    strm << "stereo:  True" << endl;
  }
  else
  {
    strm << "stereo:  False" << endl;
  }
  if (pfd.dwFlags & PFD_GENERIC_FORMAT)
  {
    strm << "hardware acceleration:  False" << endl;
  }
  else
  {
    strm << "hardware acceleration:  True" << endl;
  }
  strm << "rgba:  redSize=" << static_cast<int>(pfd.cRedBits)
       << " greenSize=" << static_cast<int>(pfd.cGreenBits)
       << "blueSize=" << static_cast<int>(pfd.cBlueBits)
       << "alphaSize=" << static_cast<int>(pfd.cAlphaBits) << endl;
  strm << "aux buffers:  " << static_cast<int>(pfd.cAuxBuffers) << endl;
  strm << "depth size:  " << static_cast<int>(pfd.cDepthBits) << endl;
  strm << "stencil size:  " << static_cast<int>(pfd.cStencilBits) << endl;
  strm << "accum:  redSize=" << static_cast<int>(pfd.cAccumRedBits)
       << " greenSize=" << static_cast<int>(pfd.cAccumGreenBits)
       << "blueSize=" << static_cast<int>(pfd.cAccumBlueBits)
       << "alphaSize=" << static_cast<int>(pfd.cAccumAlphaBits) << endl;

  delete[] this->Capabilities;

  size_t len = strm.str().length() + 1;
  this->Capabilities = new char[len];
  strncpy(this->Capabilities, strm.str().c_str(), len);

  return this->Capabilities;
}

typedef bool(APIENTRY* wglChoosePixelFormatARBType)(
  HDC, const int*, const float*, unsigned int, int*, unsigned int*);

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetupPixelFormatPaletteAndContext(
  HDC hDC, DWORD dwFlags, int debug, int bpp, int zbpp)
{
  // Create a dummy window, needed for calling wglGetProcAddress.
  HWND tempId = CreateWindowA("vtkOpenGL", 0, 0, 0, 0, 1, 1, 0, 0, this->ApplicationInstance, 0);
  HDC tempDC = GetDC(tempId);
  PIXELFORMATDESCRIPTOR tempPfd;
  memset(&tempPfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
  tempPfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
  tempPfd.nVersion = 1;
  tempPfd.dwFlags = PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
  tempPfd.iPixelType = PFD_TYPE_RGBA;
  int tempPixelFormat = ChoosePixelFormat(tempDC, &tempPfd);
  SetPixelFormat(tempDC, tempPixelFormat, &tempPfd);
  HGLRC tempContext = wglCreateContext(tempDC);
  if (!wglMakeCurrent(tempDC, tempContext))
  {
    vtkErrorMacro(
      "failed to create temporary windows OpenGL context with error: " << GetLastError());
  }

  // make sure glew is initialized with fake window
  GLenum result = glewInit();
  bool m_valid = (result == GLEW_OK);
  if (!m_valid)
  {
    vtkErrorMacro("GLEW could not be initialized.");
    return;
  }

  // First we try to use the newer wglChoosePixelFormatARB
  PIXELFORMATDESCRIPTOR pfd;
  int pixelFormat = 0;
  if (wglChoosePixelFormatARB)
  {
    int attrib[] = { WGL_ACCELERATION_ARB, WGL_FULL_ACCELERATION_ARB, WGL_SUPPORT_OPENGL_ARB, TRUE,
      WGL_DRAW_TO_WINDOW_ARB, TRUE, WGL_DOUBLE_BUFFER_ARB, TRUE, WGL_COLOR_BITS_ARB, bpp / 4 * 3,
      WGL_ALPHA_BITS_ARB, bpp / 4, WGL_DEPTH_BITS_ARB, zbpp / 4 * 3, WGL_PIXEL_TYPE_ARB,
      WGL_TYPE_RGBA_ARB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    unsigned int n = 16;
    if (this->StencilCapable)
    {
      attrib[n] = WGL_STENCIL_BITS_ARB;
      attrib[n + 1] = 8;
      n += 2;
    }
    unsigned int stereoAttributeIndex = 0;
    if (dwFlags & PFD_STEREO)
    {
      attrib[n] = WGL_STEREO_ARB;
      attrib[n + 1] = TRUE;
      stereoAttributeIndex = n + 1;
      n += 2;
    }
    if (this->UseSRGBColorSpace && WGLEW_EXT_framebuffer_sRGB)
    {
      attrib[n++] = WGL_FRAMEBUFFER_SRGB_CAPABLE_EXT;
      attrib[n++] = TRUE;
    }
    else if (this->UseSRGBColorSpace && WGLEW_ARB_framebuffer_sRGB)
    {
      attrib[n++] = WGL_FRAMEBUFFER_SRGB_CAPABLE_ARB;
      attrib[n++] = TRUE;
    }

    unsigned int numFormats;
    if (!wglChoosePixelFormatARB(hDC, attrib, 0, 1, &pixelFormat, &numFormats) || numFormats == 0)
    {
      // try dropping stereo when requested
      if (stereoAttributeIndex && numFormats == 0)
      {
        attrib[stereoAttributeIndex] = FALSE;
        wglChoosePixelFormatARB(hDC, attrib, 0, 1, &pixelFormat, &numFormats);
      }
    }

    DescribePixelFormat(hDC, pixelFormat, sizeof(pfd), &pfd);
    if (!SetPixelFormat(hDC, pixelFormat, &pfd))
    {
      pixelFormat = 0;
    }
    else
    {
      if (debug && (dwFlags & PFD_STEREO) && !(pfd.dwFlags & PFD_STEREO))
      {
        vtkGenericWarningMacro("No Stereo Available!");
        this->StereoCapableWindow = 0;
      }
    }
  }
  else
  {
    vtkErrorMacro("failed to get wglChoosePixelFormatARB");
  }

  // see if we can get a 3.2 context
  if (pixelFormat)
  {
    this->SetupPalette(hDC);

    // create a context
    PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB =
      reinterpret_cast<PFNWGLCREATECONTEXTATTRIBSARBPROC>(
        wglGetProcAddress("wglCreateContextAttribsARB"));
    this->ContextId = 0;
    if (wglCreateContextAttribsARB)
    {
      // we believe that these later versions are all compatible with
      // OpenGL 3.2 so get a more recent context if we can.
      int attemptedVersions[] = { 4, 5, 4, 4, 4, 3, 4, 2, 4, 1, 4, 0, 3, 3, 3, 2, 3, 1 };
      int iContextAttribs[] = {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 3, WGL_CONTEXT_MINOR_VERSION_ARB, 2, WGL_CONTEXT_FLAGS_ARB,
        0,
        // WGL_CONTEXT_PROFILE_MASK_ARB,
        // WGL_CONTEXT_COMPATIBILITY_PROFILE_BIT_ARB,
        // WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
        0 // End of attributes list
      };
      for (int i = 0; i < 9 && !this->ContextId; i++)
      {
        iContextAttribs[1] = attemptedVersions[i * 2];
        iContextAttribs[3] = attemptedVersions[i * 2 + 1];
        this->ContextId = wglCreateContextAttribsARB(hDC, 0, iContextAttribs);
      }
      if (this->ContextId)
      {
        // if it is a 3.1 context check for systems that we allow
        if (iContextAttribs[1] == 3 && iContextAttribs[3] == 1)
        {
          std::string vendor = (const char*)glGetString(GL_VENDOR);
          std::string renderer = (const char*)glGetString(GL_RENDERER);
          std::string version = (const char*)glGetString(GL_VERSION);
          if (vendor.find("Intel") != std::string::npos &&
            (renderer.find("HD Graphics 3000") != std::string::npos ||
              renderer.find("HD Graphics 2000") != std::string::npos))
          {
            vtkErrorMacro("We have determined that your graphics system is"
                          " an Intel SandyBridge based system. These systems only partially "
                          " support VTK. If you encounter any issues please make sure"
                          " your graphics drivers from Intel are up to date.");
          }
          else
          {
            wglDeleteContext(this->ContextId);
            this->ContextId = nullptr;
          }
        }
      }
    }
    // fallback on old approach
    if (!this->ContextId)
    {
      this->ContextId = wglCreateContext(hDC);
    }
    if (this->ContextId == nullptr)
    {
      vtkErrorMacro("wglCreateContext failed in CreateAWindow(), error: " << GetLastError());
    }
  }

  // Delete the dummy window
  wglMakeCurrent(tempDC, 0);
  wglDeleteContext(tempContext);
  ReleaseDC(tempId, tempDC);
  ::DestroyWindow(tempId); // windows api

  // If we got a valid pixel format in the process, we are done.
  // Otherwise fail as the OpenGL does not support even 2.1
  if (!pixelFormat)
  {
    vtkErrorMacro("failed to get valid pixel format.");
  }

  return;
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetupPalette(HDC hDC)
{
  int pixelFormat = GetPixelFormat(hDC);
  PIXELFORMATDESCRIPTOR pfd;
  LOGPALETTE* pPal;
  int paletteSize;

  DescribePixelFormat(hDC, pixelFormat, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

  if (pfd.dwFlags & PFD_NEED_PALETTE)
  {
    paletteSize = 1 << pfd.cColorBits;
  }
  else
  {
    return;
  }

  pPal = (LOGPALETTE*)malloc(sizeof(LOGPALETTE) + paletteSize * sizeof(PALETTEENTRY));
  pPal->palVersion = 0x300;
  pPal->palNumEntries = paletteSize;

  /* build a simple RGB color palette */
  {
    int redMask = (1 << pfd.cRedBits) - 1;
    int greenMask = (1 << pfd.cGreenBits) - 1;
    int blueMask = (1 << pfd.cBlueBits) - 1;
    int i;

    for (i = 0; i < paletteSize; ++i)
    {
      pPal->palPalEntry[i].peRed = (((i >> pfd.cRedShift) & redMask) * 255) / redMask;
      pPal->palPalEntry[i].peGreen = (((i >> pfd.cGreenShift) & greenMask) * 255) / greenMask;
      pPal->palPalEntry[i].peBlue = (((i >> pfd.cBlueShift) & blueMask) * 255) / blueMask;
      pPal->palPalEntry[i].peFlags = 0;
    }
  }

  this->Palette = CreatePalette(pPal);
  free(pPal);

  if (this->Palette)
  {
    this->OldPalette = SelectPalette(hDC, this->Palette, FALSE);
    RealizePalette(hDC);
  }
}

//------------------------------------------------------------------------------
LRESULT vtkWin32OpenGLRenderWindow::MessageProc(
  HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  switch (message)
  {
    case WM_CREATE:
    {
      // nothing to be done here, opengl is initialized after the call to
      // create now
      return 0;
    }
    case WM_DESTROY:
      this->Clean();
      if (this->DeviceContext)
      {
        ReleaseDC(this->WindowId, this->DeviceContext);
        this->DeviceContext = nullptr;
        this->WindowId = nullptr;
      }
      return 0;
    case WM_SIZE:
      /* track window size changes */
      if (this->ContextId)
      {
        this->SetSize((int)LOWORD(lParam), (int)HIWORD(lParam));
        return 0;
      }
      break;
    case WM_PALETTECHANGED:
      /* realize palette if this is *not* the current window */
      if (this->ContextId && this->Palette && (HWND)wParam != hWnd)
      {
        SelectPalette(this->DeviceContext, this->OldPalette, FALSE);
        UnrealizeObject(this->Palette);
        this->OldPalette = SelectPalette(this->DeviceContext, this->Palette, FALSE);
        RealizePalette(this->DeviceContext);
        this->Render();
      }
      break;
    case WM_QUERYNEWPALETTE:
      /* realize palette if this is the current window */
      if (this->ContextId && this->Palette)
      {
        SelectPalette(this->DeviceContext, this->OldPalette, FALSE);
        UnrealizeObject(this->Palette);
        this->OldPalette = SelectPalette(this->DeviceContext, this->Palette, FALSE);
        RealizePalette(this->DeviceContext);
        this->Render();
        return TRUE;
      }
      break;
    case WM_PAINT:
    {
      PAINTSTRUCT ps;
      BeginPaint(hWnd, &ps);
      if (this->ContextId)
      {
        this->Render();
      }
      EndPaint(hWnd, &ps);
      return 0;
    }
    break;
    case WM_ERASEBKGND:
      return TRUE;
    case WM_SETCURSOR:
      if (HTCLIENT == LOWORD(lParam))
      {
        this->SetCurrentCursor(this->GetCurrentCursor());
        return TRUE;
      }
      break;
    default:
      this->InvokeEvent(vtkCommand::RenderWindowMessageEvent, &message);
      break;
  }
  return DefWindowProc(hWnd, message, wParam, lParam);
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::InitializeApplication()
{
  // get the application instance if we don't have one already
  if (!this->ApplicationInstance)
  {
    // if we have a parent window get the app instance from it
    if (this->ParentId)
    {
      this->ApplicationInstance = (HINSTANCE)vtkGetWindowLong(this->ParentId, vtkGWL_HINSTANCE);
    }
    else
    {
      this->ApplicationInstance = GetModuleHandle(nullptr); /*AfxGetInstanceHandle();*/
    }
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetShowWindow(bool val)
{
  if (val == this->ShowWindow)
  {
    return;
  }

  if (this->WindowId)
  {
    ::ShowWindow(this->WindowId, val ? SW_SHOW : SW_HIDE);
    this->Mapped = val;
  }
  this->Superclass::SetShowWindow(val);
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::CreateAWindow()
{
  this->VTKRegisterClass();

  if (this->WindowIdReferenceCount == 0)
  {
    if (!this->WindowId)
    {
      this->DeviceContext = 0;

      if (this->GetWindowName() == DEFAULT_BASE_WINDOW_NAME)
      {
        static int count = 1;
        this->SetWindowName((DEFAULT_BASE_WINDOW_NAME + std::to_string(count++)).c_str());
      }

      int x = this->Position[0];
      int y = this->Position[1];
      int height = ((this->Size[1] > 0) ? this->Size[1] : 300);
      int width = ((this->Size[0] > 0) ? this->Size[0] : 300);

      std::wstring wname = vtksys::Encoding::ToWide(this->WindowName);
      /* create window */
      if (this->ParentId)
      {
        this->WindowId = CreateWindowW(L"vtkOpenGL", wname.c_str(),
          WS_CHILD | WS_CLIPCHILDREN /*| WS_CLIPSIBLINGS*/, x, y, width, height, this->ParentId,
          nullptr, this->ApplicationInstance, nullptr);
      }
      else
      {
        DWORD style;
        if (this->Borders)
        {
          style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN /*| WS_CLIPSIBLINGS*/;
        }
        else
        {
          style = WS_POPUP | WS_CLIPCHILDREN /*| WS_CLIPSIBLINGS*/;
        }
        RECT r;
        AdjustWindowRectForBorders(0, style, x, y, width, height, r);
        this->WindowId = CreateWindowW(L"vtkOpenGL", wname.c_str(), style, x, y, r.right - r.left,
          r.bottom - r.top, nullptr, nullptr, this->ApplicationInstance, nullptr);
      }

      if (!this->WindowId)
      {
        vtkErrorMacro("Could not create window, error:  " << GetLastError());
        return;
      }
      // extract the create info

      /* display window */
      if (this->ShowWindow)
      {
        ::ShowWindow(this->WindowId, SW_SHOW);
      }
      // UpdateWindow(this->WindowId);
      this->OwnWindow = 1;
      vtkSetWindowLong(this->WindowId, sizeof(vtkLONG), (intptr_t)this);
    }
    if (!this->DeviceContext)
    {
      this->DeviceContext = GetDC(this->WindowId);
    }
    if (this->StereoCapableWindow)
    {
      this->SetupPixelFormatPaletteAndContext(this->DeviceContext,
        PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW | PFD_DOUBLEBUFFER | PFD_STEREO, this->GetDebug(),
        32, 32);
    }
    else
    {
      this->SetupPixelFormatPaletteAndContext(this->DeviceContext,
        PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW | PFD_DOUBLEBUFFER, this->GetDebug(), 32, 32);
    }
    this->MakeCurrent();

    // wipe out any existing display lists
    this->ReleaseGraphicsResources(this);
    this->OpenGLInit();
    this->Mapped = 1;
    this->WindowIdReferenceCount = 1;
  }
  else
  {
    ++this->WindowIdReferenceCount;
  }
}

//------------------------------------------------------------------------------
// Initialize the window for rendering.
void vtkWin32OpenGLRenderWindow::WindowInitialize()
{
  // create our own window if not already set
  this->OwnWindow = 0;
  if (!this->MFChandledWindow)
  {
    this->InitializeApplication();
    this->CreateAWindow();
  }
  else
  {
    this->MakeCurrent(); // hsr
    this->OpenGLInit();
  }
}

//------------------------------------------------------------------------------
// Initialize the rendering window.
void vtkWin32OpenGLRenderWindow::Initialize(void)
{
  // make sure we haven't already been initialized
  if (!this->ContextId)
  {
    this->WindowInitialize();
  }

  if (this->SharedRenderWindow)
  {
    vtkWin32OpenGLRenderWindow* renWin =
      vtkWin32OpenGLRenderWindow::SafeDownCast(this->SharedRenderWindow);
    if (renWin && renWin->Initialized)
    {
      bool result = wglShareLists(renWin->ContextId, this->ContextId) == TRUE;
      if (result)
      {
        this->GetState()->SetVBOCache(renWin->GetVBOCache());
      }
    }
    else
    {
      // when sharing with a Generic window we rely on
      // the generic window context being current
      vtkGenericOpenGLRenderWindow* grenWin =
        vtkGenericOpenGLRenderWindow::SafeDownCast(this->SharedRenderWindow);
      grenWin->MakeCurrent();
      HGLRC current = wglGetCurrentContext();
      if (grenWin && current)
      {
        bool result = wglShareLists(current, this->ContextId) == TRUE;
        if (result)
        {
          this->GetState()->SetVBOCache(grenWin->GetVBOCache());
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::Finalize(void)
{
  if (this->CursorHidden)
  {
    this->ShowCursor();
  }

  this->DestroyWindow();
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::DestroyWindow()
{
  if (this->WindowIdReferenceCount > 0)
  {
    --this->WindowIdReferenceCount;
    if (this->WindowIdReferenceCount == 0)
    {
      this->Clean();
      if (this->WindowId)
      {
        ReleaseDC(this->WindowId, this->DeviceContext);
        // can't set WindowId=nullptr, needed for DestroyWindow
        this->DeviceContext = nullptr;

        // clear the extra data before calling destroy
        vtkSetWindowLong(this->WindowId, sizeof(vtkLONG), (vtkLONG)0);
        if (this->OwnWindow)
        {
          ::DestroyWindow(this->WindowId); // windows api
          this->WindowId = 0;
        }
        this->Mapped = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
// Get the current size of the window.
int* vtkWin32OpenGLRenderWindow::GetSize(void)
{
  // if we aren't mapped then just call super
  if (this->WindowId && !this->UseOffScreenBuffers)
  {
    RECT rect;

    // Find the current window size
    if (GetClientRect(this->WindowId, &rect))
    {
      this->Size[0] = rect.right;
      this->Size[1] = rect.bottom;
    }
    else
    {
      this->Size[0] = 0;
      this->Size[1] = 0;
    }
  }

  return this->Superclass::GetSize();
}

//------------------------------------------------------------------------------
// Get the size of the whole screen.
int* vtkWin32OpenGLRenderWindow::GetScreenSize(void)
{
  HDC hDC = ::GetDC(nullptr);
  if (hDC)
  {
    // This technique yields the screen size of the primary monitor
    // only in a multi-monitor configuration...
    this->ScreenSize[0] = ::GetDeviceCaps(hDC, HORZRES);
    this->ScreenSize[1] = ::GetDeviceCaps(hDC, VERTRES);
    ::ReleaseDC(nullptr, hDC);
  }
  else
  {
    // This technique gets the "work area" (the whole screen except
    // for the bit covered by the Windows task bar) -- use it as a
    // fallback if there's an error calling GetDC.
    RECT rect;
    SystemParametersInfo(SPI_GETWORKAREA, 0, &rect, 0);

    this->ScreenSize[0] = rect.right - rect.left;
    this->ScreenSize[1] = rect.bottom - rect.top;
  }

  return this->ScreenSize;
}

//------------------------------------------------------------------------------
// Get the position in screen coordinates of the window.
int* vtkWin32OpenGLRenderWindow::GetPosition(void)
{
  // if we aren't mapped then just return the ivar
  if (!this->Mapped)
  {
    return this->Position;
  }

  //  Find the current window position
  RECT rect;
  GetWindowRect(this->WindowId, &rect);
  this->Position[0] = rect.left;
  this->Position[1] = rect.top;

  return this->Position;
}

//------------------------------------------------------------------------------
// Change the window to fill the entire screen.
void vtkWin32OpenGLRenderWindow::SetFullScreen(vtkTypeBool arg)
{
  if (this->FullScreen == arg)
  {
    return;
  }

  if (!this->Mapped)
  {
    this->PrefFullScreen();
    return;
  }

  // set the mode
  this->FullScreen = arg;
  if (this->FullScreen <= 0)
  {
    this->Position[0] = this->OldScreen[0];
    this->Position[1] = this->OldScreen[1];
    this->Size[0] = this->OldScreen[2];
    this->Size[1] = this->OldScreen[3];
    this->Borders = this->OldScreen[4];
  }
  else
  {
    // if window already up get its values
    if (this->WindowId)
    {
      const int* temp = this->GetPosition();
      this->OldScreen[0] = temp[0];
      this->OldScreen[1] = temp[1];

      this->OldScreen[4] = this->Borders;
      this->PrefFullScreen();
    }
  }

  // remap the window
  this->WindowRemap();

  this->Modified();
}

//------------------------------------------------------------------------------
// Set the variable that indicates that we want a stereo capable window
// be created. This method can only be called before a window is realized.
//
void vtkWin32OpenGLRenderWindow::SetStereoCapableWindow(vtkTypeBool capable)
{
  if (this->ContextId == 0)
  {
    vtkRenderWindow::SetStereoCapableWindow(capable);
  }
  else
  {
    vtkWarningMacro(<< "Requesting a StereoCapableWindow must be performed "
                    << "before the window is realized, i.e. before a render.");
  }
}

//------------------------------------------------------------------------------
// Set the preferred window size to full screen.
void vtkWin32OpenGLRenderWindow::PrefFullScreen()
{
  const int* size = this->GetScreenSize();

  // don't show borders
  this->Borders = 0;

  RECT r;
  AdjustWindowRectForBorders(this->WindowId, 0, 0, 0, size[0], size[1], r);

  // use full screen
  this->Position[0] = 0;
  this->Position[1] = 0;
  this->Size[0] = r.right - r.left;
  this->Size[1] = r.bottom - r.top;
}

//------------------------------------------------------------------------------
// Remap the window.
void vtkWin32OpenGLRenderWindow::WindowRemap()
{
  // close everything down
  this->Finalize();

  // set the default windowid
  this->WindowId = this->NextWindowId;
  this->NextWindowId = 0;

  // and set it up!
  this->Initialize();
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "ContextId: " << this->ContextId << "\n";
  os << indent << "Next Window Id: " << this->NextWindowId << "\n";
  os << indent << "Window Id: " << this->WindowId << "\n";
}

//------------------------------------------------------------------------------
// Get the window id.
HWND vtkWin32OpenGLRenderWindow::GetWindowId()
{
  vtkDebugMacro(<< "Returning WindowId of " << this->WindowId << "\n");

  return this->WindowId;
}

//------------------------------------------------------------------------------
// Set the window id to a pre-existing window.
void vtkWin32OpenGLRenderWindow::SetWindowId(HWND arg)
{
  vtkDebugMacro(<< "Setting WindowId to " << arg << "\n");

  if (arg != this->WindowId)
  {
    this->WindowId = arg;
    if (this->ContextId)
    {
      wglDeleteContext(this->ContextId);
    }
    this->ContextId = 0;
    this->DeviceContext = 0;
  }
}

//------------------------------------------------------------------------------
// Set this RenderWindow's X window id to a pre-existing window.
void vtkWin32OpenGLRenderWindow::SetWindowInfo(const char* info)
{
  int tmp;

  sscanf(info, "%i", &tmp);

  this->WindowId = (HWND)tmp;
  vtkDebugMacro(<< "Setting WindowId to " << this->WindowId << "\n");
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetNextWindowInfo(const char* info)
{
  int tmp;

  sscanf(info, "%i", &tmp);

  this->SetNextWindowId((HWND)tmp);
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetDisplayId(void* arg)
{
  this->DeviceContext = (HDC)arg;
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetContextId(HGLRC arg)
{
  this->ContextId = arg;
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetDeviceContext(HDC arg)
{
  this->DeviceContext = arg;
  this->MFChandledWindow = TRUE;
}

//------------------------------------------------------------------------------
// Sets the HWND id of the window that WILL BE created.
void vtkWin32OpenGLRenderWindow::SetParentInfo(const char* info)
{
  int tmp;

  sscanf(info, "%i", &tmp);

  this->ParentId = (HWND)tmp;
  vtkDebugMacro(<< "Setting ParentId to " << this->ParentId << "\n");
}

//------------------------------------------------------------------------------
// Set the window id to a pre-existing window.
void vtkWin32OpenGLRenderWindow::SetParentId(HWND arg)
{
  vtkDebugMacro(<< "Setting ParentId to " << arg << "\n");

  this->ParentId = arg;
}

//------------------------------------------------------------------------------
// Set the window id of the new window once a WindowRemap is done.
void vtkWin32OpenGLRenderWindow::SetNextWindowId(HWND arg)
{
  vtkDebugMacro(<< "Setting NextWindowId to " << arg << "\n");

  this->NextWindowId = arg;
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetNextWindowId(void* arg)
{
  this->SetNextWindowId((HWND)arg);
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::HideCursor()
{
  if (this->CursorHidden)
  {
    return;
  }
  this->CursorHidden = 1;

  ::ShowCursor(!this->CursorHidden);
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::ShowCursor()
{
  if (!this->CursorHidden)
  {
    return;
  }
  this->CursorHidden = 0;

  ::ShowCursor(!this->CursorHidden);
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetCursorPosition(int x, int y)
{
  const int* size = this->GetSize();

  POINT point;
  point.x = x;
  point.y = size[1] - y - 1;

  if (ClientToScreen(this->WindowId, &point))
  {
    SetCursorPos(point.x, point.y);
  }
}

//------------------------------------------------------------------------------
void vtkWin32OpenGLRenderWindow::SetCurrentCursor(int shape)
{
  if (this->InvokeEvent(vtkCommand::CursorChangedEvent, &shape))
  {
    return;
  }
  this->Superclass::SetCurrentCursor(shape);
  LPCTSTR cursorName = 0;
  switch (shape)
  {
    case VTK_CURSOR_DEFAULT:
    case VTK_CURSOR_ARROW:
      cursorName = IDC_ARROW;
      break;
    case VTK_CURSOR_SIZENE:
    case VTK_CURSOR_SIZESW:
      cursorName = IDC_SIZENESW;
      break;
    case VTK_CURSOR_SIZENW:
    case VTK_CURSOR_SIZESE:
      cursorName = IDC_SIZENWSE;
      break;
    case VTK_CURSOR_SIZENS:
      cursorName = IDC_SIZENS;
      break;
    case VTK_CURSOR_SIZEWE:
      cursorName = IDC_SIZEWE;
      break;
    case VTK_CURSOR_SIZEALL:
      cursorName = IDC_SIZEALL;
      break;
    case VTK_CURSOR_HAND:
#if (WINVER >= 0x0500)
      cursorName = IDC_HAND;
#else
      cursorName = IDC_ARROW;
#endif
      break;
    case VTK_CURSOR_CROSSHAIR:
      cursorName = IDC_CROSS;
      break;
    case VTK_CURSOR_CUSTOM:
      cursorName = static_cast<LPCTSTR>(this->GetCursorFileName());
      break;
    default:
      cursorName = 0;
      break;
  }

  if (cursorName)
  {
    UINT fuLoad = LR_SHARED | LR_DEFAULTSIZE;
    if (shape == VTK_CURSOR_CUSTOM)
    {
      fuLoad |= LR_LOADFROMFILE;
    }
    HANDLE cursor = LoadImage(0, cursorName, IMAGE_CURSOR, 0, 0, fuLoad);
    if (!cursor)
    {
      vtkErrorMacro("failed to load requested cursor shape " << GetLastError());
    }
    else
    {
      SetCursor((HCURSOR)cursor);
      DestroyCursor((HCURSOR)cursor);
    }
  }
}

//------------------------------------------------------------------------------
bool vtkWin32OpenGLRenderWindow::DetectDPI()
{
  this->SetDPI(GetDeviceCaps(this->DeviceContext, LOGPIXELSY));
  return true;
}
