QT += core gui opengl widgets
TARGET = kinect
TEMPLATE = lib

INCLUDEPATH += "../faceCommon"
INCLUDEPATH += "/usr/include/libfreenect/"

SOURCES += \
    kinect.cpp \
    dlgscanface.cpp

HEADERS += \
    kinect.h \
    dlgscanface.h
    
LIBS += `pkg-config --libs opencv` -lGL -lGLU -lfreenect -lfreenect_sync

FORMS += \
    dlgscanface.ui


