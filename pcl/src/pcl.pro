QT += core gui opengl
TARGET = pcl
TEMPLATE = app

SOURCES += main.cpp \
    kinectacquire.cpp

HEADERS += \
    kinectacquire.h
    
INCLUDEPATH += /usr/include/pcl-1.6/ /usr/include/eigen3/ /usr/include/ni/ /usr/include/vtk-5.10/
#LIBS += `pkg-config --libs opencv` -lGL -lGLU -lfreenect -lfreenect_sync
LIBS += -lpcl_visualization -lpcl_io -lpcl_common -lpcl_filters -lboost_system -lboost_thread -lOpenNI


