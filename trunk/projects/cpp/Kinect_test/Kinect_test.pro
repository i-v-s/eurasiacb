#-------------------------------------------------
#
# Project created by QtCreator 2012-03-13T20:36:46
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Kinect_test
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

SOURCES += main.cpp

INCLUDEPATH += -I/usr/include/libfreenect

LIBS += -L/usr/lib \
        -lfreenect
