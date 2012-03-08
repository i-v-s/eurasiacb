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
Win32 {

    INCLUDEPATH += E:\opencv\binaries\install\include

    LIBS += -LE:\opencv\binaries\install\lib \
    -llibopencv_core231 \
    -llibopencv_highgui231 \
    -llibopencv_imgproc231 \
    -llibopencv_features2d231 \
    -llibopencv_calib3d231
}
Win64 {
    INCLUDEPATH += D:\Users\Vasiliy\OpenCV\opencv\build\include

    LIBS += -LD:\Users\Vasiliy\OpenCV\opencv\build\x64\mingw\lib \
    -llibopencv_core231 \
    -llibopencv_highgui231 \
    -llibopencv_imgproc231 \
    -llibopencv_features2d231 \
    -llibopencv_calib3d231
}

SOURCES += main.cpp \
    videoprocessor.cpp \
    qstab.cpp

HEADERS += \
    videoprocessor.h \
    qstab.h
