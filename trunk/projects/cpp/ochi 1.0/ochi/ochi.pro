#-------------------------------------------------
#
# Project created by QtCreator 2012-02-16T14:51:11
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ochi
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    viso_stereo.cpp \
    viso_mono.cpp \
    viso.cpp \
    triangle.cpp \
    reconstruction.cpp \
    matrix.cpp \
    matcher.cpp \
    filter.cpp

OTHER_FILES += \
    ochi.pro.user

HEADERS += \
    viso_stereo.h \
    viso_mono.h \
    viso.h \
    triangle.h \
    timer.h \
    reconstruction.h \
    matrix.h \
    matcher.h \
    filter.h


LIBS += -lpng
QMAKE_CXXFLAGS += -msse3
