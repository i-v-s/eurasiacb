#-------------------------------------------------
#
# Project created by QtCreator 2012-04-06T19:58:13
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = icu_ps3eye
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

SOURCES += main.cpp

LIBS += -lusb
