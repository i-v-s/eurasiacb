#-------------------------------------------------
#
# Project created by QtCreator 2012-04-13T21:12:59
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = icu_retifier
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += libxml++-2.6

    INCLUDEPATH += -I/usr/local/include
    LIBS += -L/usr/local/lib \
            -lopencv_core \
            -lopencv_highgui \
            -lopencv_imgproc \
            -lopencv_features2d \
            -lopencv_calib3d
}


SOURCES += main.cpp \
    camerarectifier.cpp \
    ../icu_calib/device.cpp

HEADERS += \
    camerarectifier.h \
    ../icu_calib/device.h
