#-------------------------------------------------
#
# Project created by QtCreator 2013-03-17T09:59:54
#
#-------------------------------------------------

QT       += core gui widgets opengl

TARGET = appMorphFaceModel
TEMPLATE = app

INCLUDEPATH += "../faceCommon"

LIBS += -L../faceCommon -lfaceCommon
LIBS += `pkg-config --libs opencv` -lGL -lGLU

SOURCES += main.cpp \
    morphable3dfacemodelwidget.cpp \
    morphable3dfacemodel.cpp

FORMS += \
    morphable3dfacemodelwidget.ui

HEADERS += \
    morphable3dfacemodelwidget.h \
    morphable3dfacemodel.h
