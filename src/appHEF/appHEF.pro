#-------------------------------------------------
#
# Project created by QtCreator 2013-03-17T09:59:54
#
#-------------------------------------------------

QT       += core gui widgets opengl

TARGET = appHEF
TEMPLATE = app

INCLUDEPATH += "../faceCommon"

LIBS += -L../faceCommon -lfaceCommon
LIBS += `pkg-config --libs opencv` -lGL -lGLU

SOURCES += main.cpp \
    winhef.cpp \
    filedata.cpp

HEADERS += \
    winhef.h \
    filedata.h

FORMS += \
    winhef.ui
