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
    PKGCONFIG += opencv  libxml++-2.6
}


SOURCES += main.cpp \
    cameracalibrator.cpp \
    device.cpp

HEADERS += \
    cameracalibrator.h \
    device.h
