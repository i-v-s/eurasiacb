#-------------------------------------------------
#
# Project created by QtCreator 2012-02-25T21:54:13
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = icu_calib
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
    cameracalibrator.cpp \
    device.cpp

HEADERS += \
    cameracalibrator.h \
    device.h
