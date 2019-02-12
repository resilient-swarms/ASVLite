QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets opengl xml

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


SOURCES += \
    source/regular_wave.cpp \
    source/sea_surface_dynamics.cpp \
    source/wave_spectrum.cpp \
    unit_tests/test_regular_wave.cpp \
    unit_tests/test_sea_surface_dynamics.cpp \
    unit_tests/test_wave_spectrum.cpp \
    unit_tests/unit_test_runner.cpp


HEADERS += \
    include/exception.h \
    include/geometry.h \
    include/regular_wave.h \
    include/sea_surface_dynamics.h \
    include/units_and_constants.h \
    include/wave_spectrum.h

INCLUDEPATH += include

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

unix:!macx: LIBS += -lCGAL \
                    -lgtest \
                    -lCGAL_Qt5 \
                    -lmpfr \
                    -lgmp

