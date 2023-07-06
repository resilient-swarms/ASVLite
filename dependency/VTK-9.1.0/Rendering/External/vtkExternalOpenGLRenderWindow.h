/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkExternalOpenGLRenderWindow.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkExternalOpenGLRenderWindow
 * @brief   OpenGL render window that allows using
 * an external window to render vtk objects
 *
 * vtkExternalOpenGLRenderWindow is a concrete implementation of the abstract
 * class vtkRenderWindow. vtkExternalOpenGLRenderer interfaces to the OpenGL
 * graphics library.
 *
 * This class extends vtkGenericOpenGLRenderWindow to allow sharing the
 * same OpenGL context by various visualization applications. Basically, this
 * class prevents VTK from creating a new OpenGL context. Thus, it requires that
 * an OpenGL context be initialized before Render is called.
 * \sa Render()
 *
 * It is a generic implementation; this window is platform agnostic. However,
 * the application user must explicitly make sure the window size is
 * synchronized when the external application window/viewport resizes.
 * \sa SetSize()
 *
 * It has the same requirements as the vtkGenericOpenGLRenderWindow, whereby,
 * one must register an observer for WindowMakeCurrentEvent,
 * WindowIsCurrentEvent and WindowFrameEvent.
 * \sa vtkGenericOpenGLRenderWindow
 */

#ifndef vtkExternalOpenGLRenderWindow_h
#define vtkExternalOpenGLRenderWindow_h

#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkRenderingExternalModule.h" // For export macro

class VTKRENDERINGEXTERNAL_EXPORT vtkExternalOpenGLRenderWindow
  : public vtkGenericOpenGLRenderWindow
{
public:
  static vtkExternalOpenGLRenderWindow* New();
  vtkTypeMacro(vtkExternalOpenGLRenderWindow, vtkGenericOpenGLRenderWindow);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Begin the rendering process using the existing context.
   */
  void Start(void) override;

  /**
   * Tells if this window is the current graphics context for the calling
   * thread.
   */
  bool IsCurrent() override;

  ///@{
  /**
   * Turn on/off a flag which enables/disables automatic positioning and
   * resizing of the render window. By default, vtkExternalOpenGLRenderWindow
   * queries the viewport position and size (glViewport) from the OpenGL state
   * and uses it to resize itself. However, in special circumstances this
   * feature is undesirable. One such circumstance may be to avoid performance
   * penalty of querying OpenGL state variables. So the following boolean is
   * provided to disable automatic window resize.
   * (Turn AutomaticWindowPositionAndResize off if you do not want the viewport
   * to be queried from the OpenGL state.)
   */
  vtkGetMacro(AutomaticWindowPositionAndResize, int);
  vtkSetMacro(AutomaticWindowPositionAndResize, int);
  vtkBooleanMacro(AutomaticWindowPositionAndResize, int);
  ///@}

  ///@{
  /**
   * Turn on/off a flag which enables/disables using the content from an
   * outside applicaiton.  When on the active read buffer is first blitted
   * into VTK and becomes the starting poiint for VTK's rendering.
   */
  vtkGetMacro(UseExternalContent, bool);
  vtkSetMacro(UseExternalContent, bool);
  vtkBooleanMacro(UseExternalContent, bool);
  ///@}

protected:
  vtkExternalOpenGLRenderWindow();
  ~vtkExternalOpenGLRenderWindow() override;

  int AutomaticWindowPositionAndResize;
  bool UseExternalContent;

private:
  vtkExternalOpenGLRenderWindow(const vtkExternalOpenGLRenderWindow&) = delete;
  void operator=(const vtkExternalOpenGLRenderWindow&) = delete;
};
#endif // vtkExternalOpenGLRenderWindow_h
