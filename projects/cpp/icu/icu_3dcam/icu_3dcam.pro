#-------------------------------------------------
#
# Project created by QtCreator 2012-03-15T17:15:20
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = icu_3dcam
CONFIG   += console
CONFIG   -= app_bundle

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

TEMPLATE = app

SOURCES += main.cpp

INCLUDEPATH += -I/usr/include/openni
