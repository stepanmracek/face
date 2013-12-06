QT          += core opengl
TARGET      = softKinetic
TEMPLATE    = app

INCLUDEPATH += "../faceCommon"
INCLUDEPATH += "/opt/softkinetic/DepthSenseSDK/include"

LIBS += -L../faceCommon -lfaceCommon
LIBS += `pkg-config --libs opencv` -lGL -lGLU

LIBS += -L/opt/softkinetic/DepthSenseSDK/lib -lDepthSense -lDepthSensePlugins -lturbojpeg

SOURCES += \
    main.cpp

