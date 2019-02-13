QT       += core

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = asv_swarm
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS CGAL_USE_BASIC_VIEWER

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

# Destination directories
release: DESTDIR = build/release
debug: DESTDIR = build/debug
test: DESTDIR = build/test
OBJECTS_DIR = $$DESTDIR/obj
MOC_DIR = $$DESTDIR/moc
RCC_DIR = $$DESTDIR/qrc
UI_DIR = $$DESTDIR/ui
MAKEFILE = $$DESTDIR/Makefile

HEADERS += \
    include/exception.h \
    include/geometry.h \
    include/regular_wave.h \
    include/sea_surface_dynamics.h \
    include/units_and_constants.h \
    include/wave_spectrum.h \

INCLUDEPATH += include

SOURCES += \
    source/geometry.cpp \
    source/regular_wave.cpp \
    source/sea_surface_dynamics.cpp \
    source/wave_spectrum.cpp

# Additional source for unit tests
test: SOURCES += \
    tests/test_regular_wave.cpp \
    tests/unit_test_runner.cpp

# Additional lib for unit test
test: LIBS += -lgtest




