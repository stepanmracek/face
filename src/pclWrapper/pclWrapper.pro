QT += core gui opengl
TARGET = pclWrapper
TEMPLATE = lib

SOURCES += \
    kinectacquire.cpp \
    align.cpp \
    common.cpp

HEADERS += \
    kinectacquire.h \
    common.h \
    align.h
    
INCLUDEPATH += /usr/include/pcl-1.6/ /usr/include/eigen3/ /usr/include/ni/ /usr/include/vtk-5.10/
#LIBS += `pkg-config --libs opencv` -lGL -lGLU -lfreenect -lfreenect_sync
LIBS += -lpcl_visualization -lpcl_io -lpcl_common -lpcl_filters -lpcl_features -lpcl_kdtree -lpcl_search
LIBS += -L/usr/lib/vtk-5.10/
LIBS += -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkGraphics
LIBS += -lboost_system -lboost_thread -lOpenNI -lboost_filesystem


