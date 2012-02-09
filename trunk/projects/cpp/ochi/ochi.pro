QT       += core

QT       -= gui

TARGET = ochi
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv \
                libpng
    INCLUDEPATH += "/usr/include/c++/4.6.1/"
}
Win32 {

    INCLUDEPATH += E:\opencv\binaries\install\include

    LIBS += -LE:\opencv\binaries\install\lib \
    -llibopencv_core231 \
    -llibopencv_highgui231 \
    -llibopencv_imgproc231 \
    -llibopencv_features2d231 \
    -llibopencv_calib3d231
}


SOURCES += main.cpp \
    matcher.cpp \
    matrix.cpp \
    filter.cpp \
    visualodometry.cpp \
    triangle.cpp \
    reconstruction.cpp \
    pngUtils.cpp \
    drawing.cpp \
    rectify.cpp

HEADERS += \
    matcher.h \
    matrix.h \
    filter.h \
    visualodometry.h \
    triangle.h \
    reconstruction.h \
    timer.h \
    pngUtils.h \
    drawing.h \
    rectify.h

QMAKE_CXXFLAGS += -msse2 -msse4
