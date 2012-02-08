QT       += core

QT       -= gui

TARGET = ochi
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    matcher.cpp \
    matrix.cpp \
    filter.cpp \
    visualodometry.cpp \
    triangle.cpp \
    reconstruction.cpp

HEADERS += \
    matcher.h \
    matrix.h \
    filter.h \
    visualodometry.h \
    triangle.h \
    reconstruction.h \
    timer.h

QMAKE_CXXFLAGS += -msse2 -msse4
