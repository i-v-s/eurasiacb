#-------------------------------------------------
#
# Project created by QtCreator 2011-09-25T01:20:06
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = qstab
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

SOURCES += main.cpp
