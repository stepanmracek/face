QT += core gui opengl widgets
TARGET = kinect
TEMPLATE = lib

INCLUDEPATH += "../faceCommon"
INCLUDEPATH += "/usr/include/libfreenect/"

SOURCES += \
    kinect.cpp

HEADERS += \
    kinect.h
    
LIBS += `pkg-config --libs opencv` -lGL -lGLU -lfreenect -lfreenect_sync


