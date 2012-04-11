#-------------------------------------------------
#
# Project created by QtCreator 2012-02-25T17:36:24
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = icu
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv libxml++-2.6
}


SOURCES += main.cpp \
    libviso2/viso_stereo.cpp \
    libviso2/viso_mono.cpp \
    libviso2/viso.cpp \
    libviso2/triangle.cpp \
    libviso2/reconstruction.cpp \
    libviso2/matrix.cpp \
    libviso2/matcher.cpp \
    libviso2/filter.cpp \
    ../icu_calib/device.cpp

HEADERS += \
    libviso2/viso_stereo.h \
    libviso2/viso_mono.h \
    libviso2/viso.h \
    libviso2/triangle.h \
    libviso2/timer.h \
    libviso2/reconstruction.h \
    libviso2/matrix.h \
    libviso2/matcher.h \
    libviso2/filter.h \
    ../icu_calib/device.h

QMAKE_CXXFLAGS += -msse3
