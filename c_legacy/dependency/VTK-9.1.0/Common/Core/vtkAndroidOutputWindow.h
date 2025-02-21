/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkAndroidOutputWindow.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkAndroidOutputWindow
 * @brief   Win32 Specific output window class
 *
 * This class is used for error and debug message output on the windows
 * platform.   It creates a read only EDIT control to display the
 * output.   This class should not be used directly.   It should
 * only be used through the interface of vtkOutputWindow.  This class
 * only handles one output window per process.  If the window is destroyed,
 * the vtkObject::GlobalWarningDisplayOff() function is called.  The
 * window is created the next time text is written to the window.
 */

#ifndef vtkAndroidOutputWindow_h
#define vtkAndroidOutputWindow_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkOutputWindow.h"

class VTKCOMMONCORE_EXPORT vtkAndroidOutputWindow : public vtkOutputWindow
{
public:
  // Methods from vtkObject
  vtkTypeMacro(vtkAndroidOutputWindow, vtkOutputWindow);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Create a vtkAndroidOutputWindow.
   */
  static vtkAndroidOutputWindow* New();

  ///@{
  /**
   * New lines are converted to carriage return new lines.
   */
  void DisplayText(const char*) override;
  void DisplayErrorText(const char*) override;
  void DisplayWarningText(const char*) override;
  void DisplayGenericWarningText(const char*) override;
  ///@}

  void DisplayDebugText(const char*) override;

protected:
  vtkAndroidOutputWindow();
  ~vtkAndroidOutputWindow() override;

private:
  vtkAndroidOutputWindow(const vtkAndroidOutputWindow&) = delete;
  void operator=(const vtkAndroidOutputWindow&) = delete;
};

#endif
