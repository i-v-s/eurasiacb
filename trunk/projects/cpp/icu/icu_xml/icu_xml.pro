#-------------------------------------------------
#
# Project created by QtCreator 2012-04-08T10:52:36
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = icu_xml
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv libxml++-2.6
}


SOURCES += main.cpp \
    xml_handler.cpp

HEADERS +=
