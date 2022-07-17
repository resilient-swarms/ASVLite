/*=========================================================================

  Program:   Visualization Toolkit
  Module:    QVTKOpenGLStereoWidget.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#ifndef QVTKOpenGLStereoWidget_h
#define QVTKOpenGLStereoWidget_h

#include "vtkGUISupportQtModule.h" // For export macro
#include <QWidget>

#include "QVTKOpenGLWindow.h" // needed for ivar
#include <QPointer>           // needed for ivar

#include "vtkDeprecation.h" // For VTK_DEPRECATED_IN_9_0_0

// Forward Qt class declarations
class QSurfaceFormat;
class QOpenGLContext;

// class QVTKInteractor;
class QVTKInteractorAdapter;
class QVTKOpenGLWindow;
class vtkGenericOpenGLRenderWindow;
class vtkRenderWindow;
class vtkRenderWindowInteractor;

/**
 * @class QVTKOpenGLStereoWidget
 * @brief QWidget for displaying a vtkRenderWindow in a Qt Application.
 *
 * QVTKOpenGLStereoWidget simplifies using a QVTKOpenGLWindow as a widget in Qt
 * application so it can be embedded in a layout rather than being a top-level
 * window. QVTKOpenGLWindow has all the limitations posed by Qt with
 * `QWidget::createWindowContainer` hence developers are advised to refer to Qt
 * docs for more details.
 *
 * In general QVTKOpenGLNativeWidget may be a better choice, however
 * QVTKOpenGLWindow-based QVTKOpenGLStereoWidget may be better choice for applications
 * requiring quad-buffer stereo.
 *
 * Due to Qt limitations, QVTKOpenGLStereoWidget does not support being a
 * native widget. But native widget are sometimes mandatory, for example within
 * QScrollArea and QMDIArea, so the QVTKOpenGLNativeWidget should be
 * used when in needs of VTK rendering in the context of Qt native widget.
 *
 * If a QVTKOpenGLStereoWidget is used in a QScrollArea or in a QMDIArea, it
 * will force it to be native and this is *NOT* supported.
 *
 * Unlike QVTKOpenGLNativeWidget, QVTKOpenGLStereoWidget does not require that the
 * default surface format for the application be changed. One can simply specify
 * the needed QSurfaceFormat for the specific QVTKOpenGLStereoWidget instance by
 * calling `QVTKOpenGLStereoWidget::setFormat` before the widget is initialized.
 *
 * @sa QVTKOpenGLWindow QVTKOpenGLNativeWidget QVTKRenderWidget
 */
class VTKGUISUPPORTQT_EXPORT QVTKOpenGLStereoWidget : public QWidget
{
  Q_OBJECT
  typedef QWidget Superclass;

public:
  QVTKOpenGLStereoWidget(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
  QVTKOpenGLStereoWidget(
    QOpenGLContext* shareContext, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
  QVTKOpenGLStereoWidget(vtkGenericOpenGLRenderWindow* w, QWidget* parent = nullptr,
    Qt::WindowFlags f = Qt::WindowFlags());
  QVTKOpenGLStereoWidget(vtkGenericOpenGLRenderWindow* w, QOpenGLContext* shareContext,
    QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
  ~QVTKOpenGLStereoWidget() override;

  ///@{
  /**
   * @copydoc QVTKOpenGLWindow::setRenderWindow()
   */
  void setRenderWindow(vtkGenericOpenGLRenderWindow* win)
  {
    this->VTKOpenGLWindow->setRenderWindow(win);
  }
  void setRenderWindow(vtkRenderWindow* win) { this->VTKOpenGLWindow->setRenderWindow(win); }
  ///@}

  /**
   * @copydoc QVTKOpenGLWindow::renderWindow()
   */
  vtkRenderWindow* renderWindow() const { return this->VTKOpenGLWindow->renderWindow(); }

  /**
   * @copydoc QVTKOpenGLWindow::interactor()
   */
  QVTKInteractor* interactor() const { return this->VTKOpenGLWindow->interactor(); }

  /**
   * @copydoc QVTKRenderWindowAdapter::defaultFormat(bool)
   */
  static QSurfaceFormat defaultFormat(bool stereo_capable = false)
  {
    return QVTKOpenGLWindow::defaultFormat(stereo_capable);
  }

  /**
   * @copydoc QVTKOpenGLWindow::setEnableHiDPI()
   */
  void setEnableHiDPI(bool enable) { this->VTKOpenGLWindow->setEnableHiDPI(enable); }
  bool enableHiDPI() const { return this->VTKOpenGLWindow->enableHiDPI(); }

  ///@{
  /**
   * Set/Get unscaled DPI value. Defaults to 72, which is also the default value
   * in vtkWindow.
   */
  void setUnscaledDPI(int dpi) { this->VTKOpenGLWindow->setUnscaledDPI(dpi); }
  int unscaledDPI() const { return this->VTKOpenGLWindow->unscaledDPI(); }
  ///@}

  ///@{
  /**
   * Set/Get a custom device pixel ratio to use to map Qt sizes to VTK (or
   * OpenGL) sizes. Thus, when the QWidget is resized, it called
   * `vtkRenderWindow::SetSize` on the internal vtkRenderWindow after
   * multiplying the QWidget's size by this scale factor.
   *
   * By default, this is set to 0. Which means that `devicePixelRatio` obtained
   * from Qt will be used. Set this to a number greater than 0 to override this
   * behaviour and use the custom scale factor instead.
   *
   * `effectiveDevicePixelRatio` can be used to obtain the device-pixel-ratio
   * that will be used given the value for customDevicePixelRatio.
   */
  void setCustomDevicePixelRatio(double cdpr)
  {
    this->VTKOpenGLWindow->setCustomDevicePixelRatio(cdpr);
  };
  double customDevicePixelRatio() const { return this->VTKOpenGLWindow->customDevicePixelRatio(); };
  double effectiveDevicePixelRatio() const
  {
    return this->VTKOpenGLWindow->effectiveDevicePixelRatio();
  };
  ///@}

  ///@{
  /**
   * @copydoc QVTKOpenGLWindow::setDefaultCursor()
   */
  void setDefaultCursor(const QCursor& cursor) { this->VTKOpenGLWindow->setDefaultCursor(cursor); }
  const QCursor& defaultCursor() const { return this->VTKOpenGLWindow->defaultCursor(); }
  ///@}

  /**
   * Returns true if the internal QOpenGLWindow's is valid, i.e. if OpenGL
   * resources, like the context, have been successfully initialized.
   */
  bool isValid() { return this->VTKOpenGLWindow->isValid(); }

  /**
   * Expose internal QVTKOpenGLWindow::grabFramebuffer(). Renders and returns
   * a 32-bit RGB image of the framebuffer.
   */
  QImage grabFramebuffer();

  /**
   * Returns the embedded QVTKOpenGLWindow.
   */
  QVTKOpenGLWindow* embeddedOpenGLWindow() const { return this->VTKOpenGLWindow; }

  /**
   * Sets the requested surface format.
   *
   * When the format is not explicitly set via this function, the format
   * returned by QSurfaceFormat::defaultFormat() will be used. This means that
   * when having multiple OpenGL widgets, individual calls to this function can
   * be replaced by one single call to QSurfaceFormat::setDefaultFormat() before
   * creating the first widget.
   */
  void setFormat(const QSurfaceFormat& fmt) { this->VTKOpenGLWindow->setFormat(fmt); }

  /**
   * Returns the context and surface format used by this widget and its toplevel window.
   */
  QSurfaceFormat format() const { return this->VTKOpenGLWindow->format(); }

  ///@{
  /**
   * @deprecated in VTK 8.3
   */
  VTK_DEPRECATED_IN_9_0_0("Use QVTKOpenGLStereoWidget::setRenderWindow")
  void SetRenderWindow(vtkGenericOpenGLRenderWindow* win);
  VTK_DEPRECATED_IN_9_0_0("Use QVTKOpenGLStereoWidget::setRenderWindow")
  void SetRenderWindow(vtkRenderWindow* win);
  ///@}

  ///@{
  /**
   * These methods have be deprecated to fix naming style. Since
   * QVTKOpenGLNativeWidget is QObject subclass, we follow Qt naming conventions
   * rather than VTK's.
   */
  VTK_DEPRECATED_IN_9_0_0("Use QVTKOpenGLStereoWidget::renderWindow")
  vtkRenderWindow* GetRenderWindow();
  VTK_DEPRECATED_IN_9_0_0("Removed in 9.0.0 (internal)")
  QVTKInteractor* GetInteractor();
  ///@}

  /**
   * @deprecated in VTK 8.3
   * QVTKInteractorAdapter is an internal helper. Hence the API was removed.
   */
  VTK_DEPRECATED_IN_9_0_0("Removed in 9.0.0 (internal)")
  QVTKInteractorAdapter* GetInteractorAdapter();

  /**
   * @deprecated in VTK 8.3. Simply use `QWidget::setCursor` API to change
   * cursor.
   */
  VTK_DEPRECATED_IN_9_0_0("Use QWidget::setCursor")
  void setQVTKCursor(const QCursor& cursor);

  /**
   * @deprecated in VTK 8.3. Use `setDefaultCursor` instead.
   */
  VTK_DEPRECATED_IN_9_0_0("Use QWidget::setDefaultCursor")
  void setDefaultQVTKCursor(const QCursor& cursor);

protected:
  void resizeEvent(QResizeEvent* evt) override;
  void paintEvent(QPaintEvent* evt) override;

private:
  QPointer<QVTKOpenGLWindow> VTKOpenGLWindow;
};

#endif
