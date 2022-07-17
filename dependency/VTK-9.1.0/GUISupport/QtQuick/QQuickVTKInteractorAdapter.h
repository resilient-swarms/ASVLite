/*=========================================================================

  Program:   Visualization Toolkit
  Module:    QQuickVTKInteractorAdapter.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#ifndef QQuickVTKInteractorAdapter_h
#define QQuickVTKInteractorAdapter_h

// VTK includes
#include "QVTKInteractorAdapter.h"
#include "vtkGUISupportQtQuickModule.h" // for export macro

// Qt includes
#include <QList>    // for QList
#include <QPointer> // for QPointer

// Forward declarations
class QEnterEvent;
class QEvent;
class QFocusEvent;
class QHoverEvent;
class QKeyEvent;
class QMouseEvent;
class QQuickItem;
class QQuickWindow;
class QWheelEvent;
class vtkRenderWindowInteractor;
class vtkRenderer;

/**
 * @class QQuickVTKInteractorAdapter
 * @brief Intermediate class that handles relaying Qt events to VTK
 */
class VTKGUISUPPORTQTQUICK_EXPORT QQuickVTKInteractorAdapter : public QVTKInteractorAdapter
{
  Q_OBJECT
  typedef QVTKInteractorAdapter Superclass;

public:
  QQuickVTKInteractorAdapter(QObject* parent = nullptr);

  void setQQuickWindow(QQuickWindow* win);

  void QueueHoverEvent(QQuickItem* item, QHoverEvent* e);
  void QueueKeyEvent(QQuickItem* item, QKeyEvent* e);
  void QueueFocusEvent(QQuickItem* item, QFocusEvent* e);
  void QueueMouseEvent(QQuickItem* item, QMouseEvent* e);
  void QueueGeometryChanged(const QRectF& newGeometry, const QRectF& oldGeometry);
  void QueueWheelEvent(QQuickItem* item, QWheelEvent* e);

  void ProcessEvents(vtkRenderWindowInteractor* interactor);

  /*
   * Map the event position to VTK display coordinates
   * The mapping considers the following:
   *  - VTK widgets expect display coordinates, not viewport/local coordinates
   *  - vtkRenderWindowInteractor flips Y before processing the event.
   * Because of the inherent flip in the superclass, the mapping does not flip Y implicitly.
   * To map and flip Y, use mapEventPositionFlipY.
   *
   * \sa mapEventPositionFlipY
   */
  static QPointF mapEventPosition(QQuickItem* item, const QPointF& localPos);

  /*
   * Map the event position to VTK display coordinates and flip the Y axis to switch the point from
   * the Qt coordinate reference system to VTK's.
   *
   * \sa mapEventPosition
   */
  static QPointF mapEventPositionFlipY(QQuickItem* item, const QPointF& localPos);

protected:
  void QueueEvent(QEvent* e);

private:
  QPointer<QQuickWindow> m_qwindow;
  QList<QEvent*> m_queuedEvents;

  Q_DISABLE_COPY(QQuickVTKInteractorAdapter)
};

#endif // QQuickVTKInteractorAdapter_h
