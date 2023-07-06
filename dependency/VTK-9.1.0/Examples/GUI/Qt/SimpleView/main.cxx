/*
 * Copyright 2007 Sandia Corporation.
 * Under the terms of Contract DE-AC04-94AL85000, there is a non-exclusive
 * license for use of this work by or on behalf of the
 * U.S. Government. Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that this Notice and any
 * statement of authorship are reproduced on all copies.
 */
// QT includes
#include <QtGui/QSurfaceFormat>
#include <QtWidgets/QApplication>

#include "QVTKRenderWidget.h"
#include "SimpleView.h"

extern int qInitResources_icons();

int main(int argc, char** argv)
{
  // needed to ensure appropriate OpenGL context is created for VTK rendering.
  QSurfaceFormat::setDefaultFormat(QVTKRenderWidget::defaultFormat());

  // QT Stuff
  QApplication app(argc, argv);

  QApplication::setStyle("fusion");

  qInitResources_icons();

  SimpleView mySimpleView;
  mySimpleView.show();

  return app.exec();
}
