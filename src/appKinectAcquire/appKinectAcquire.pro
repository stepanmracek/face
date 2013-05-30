#-------------------------------------------------
#
# Project created by QtCreator 2013-03-17T09:59:54
#
#-------------------------------------------------

QT += core gui opengl widgets

TARGET = appKinectAcquire
TEMPLATE = app

INCLUDEPATH += "../faceCommon" "../kinect"

LIBS += `pkg-config --libs opencv` -lGL -lGLU
LIBS += -L../faceCommon -lfaceCommon
LIBS += -L../kinect -lkinect


SOURCES += \
    main.cpp

