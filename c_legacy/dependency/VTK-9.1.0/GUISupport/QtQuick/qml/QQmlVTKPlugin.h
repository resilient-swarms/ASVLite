/*=========================================================================

  Program:   Visualization Toolkit
  Module:    QQmlVTKPlugin.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#ifndef QQmlVTKPlugin_h
#define QQmlVTKPlugin_h

// Qt includes
#include <QQmlExtensionPlugin>

/**
 * \class QQmlVTKPlugin
 * \brief Plugin class to expose a VTK C++ module to QML applications
 *
 * QQmlVTKPlugin registers various VTK C++ classes as QML types so that QtQuick applications can
 * directly import and use these types from QML.
 *
 * ## Importing the VTK module in QML
 * As part of VTK's compilation process, it would compile and install a \em \b qmldir file that
 * provides the module definition and relevant plugin information required by QML to load VTK. To
 * load the plugin, set the environment variable
 * [QML2_IMPORT_PATH](https://doc.qt.io/qt-5/qtqml-syntax-imports.html#qml-import-path) to the path
 * of the directory containing the \em qmldir file.
 *
 *  \code
 *  # /projects/Import has a sub-directory VTK.9.0/qmldir
 *  $ export QML2_IMPORT_PATH=/projects/Import
 *  \endcode
 *
 *  Once the import path is set correctly, the module can be imported in the \em .qml file as
 *  follows:
 *
 *  \code
 *  import VTK 9.0
 *  \endcode
 *
 *  ## Registered types
 *  The C++ classes exposed to QML and their associated typenames are as follows:
 *
 *   | VTK C++ class               |   QML type       |
 *   | :--------------:            | :--------------: |
 *   | QQuickVTKRenderWindow       |  VTKRenderWindow |
 *   | QQuickVTKRenderItem         |  VTKRenderItem   |
 *   | QQuickVTKInteractiveWidget  |  VTKWidget       |
 *
 * ## Versioning
 * The VTK QML module follows the version number of the VTK source tree. For example, if compiled
 * against VTK 9.0.x, the VTK module version will be 9.0
 */
class QQmlVTKPlugin : public QQmlExtensionPlugin
{
  Q_OBJECT
  typedef QQmlExtensionPlugin Superclass;

  Q_PLUGIN_METADATA(IID "org.kitware.VTK")

public:
  /**
   * Constructor
   */
  QQmlVTKPlugin() = default;

  /**
   * Destructor
   */
  virtual ~QQmlVTKPlugin() = default;

  /**
   * Register QML types provided by VTK
   */
  void registerTypes(const char* uri);

  /**
   * Initialize the extension using the QQmlEngine
   *
   * \sa cleanup
   */
  void initializeEngine(QQmlEngine* engine, const char* uri);

protected Q_SLOTS:
  /**
   * Destroy any singleton instances that were created during initializeEngine
   *
   * \sa initializeEngine
   */
  void cleanup();

private:
  Q_DISABLE_COPY(QQmlVTKPlugin);
};

#endif // QQmlVTKPlugin_h
