#-------------------------------------------------
#
# Project created by QtCreator 2012-02-13T19:18:34
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = csharpblessyou
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    triangle.cpp \
    matrix.cpp \
    matcher.cpp \
    filter.cpp

INCLUDEPATH += /usr/include/libpng/

LIBS += -L/usr/lib \
    -lpng

QMAKE_CXXFLAGS += -msse2 -msse4

HEADERS += \
    triangle.h \
    matrix.h \
    matcher.h \
    filter.h
