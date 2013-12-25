#-------------------------------------------------
#
# Project created by QtCreator 2013-03-17T09:59:54
#
#-------------------------------------------------

QT += core gui opengl widgets

TARGET = appEvaluation
TEMPLATE = app

QMAKE_CXXFLAGS+= -fopenmp
QMAKE_LFLAGS +=  -fopenmp

INCLUDEPATH += "../faceCommon"

LIBS += -L../faceCommon -lfaceCommon
LIBS += `pkg-config --libs opencv` -lGL -lGLU

SOURCES += main.cpp

HEADERS += \
    evaluatethermo2.h \
    evaluatethermo.h \
    evaluatefeaturestability.h \
    evaluatefeatureselection.h \
    evaluatefeaturelevelfusion.h \
    evaluateICA.h \
    evaluate3dfrgc.h \
    evaluatekinect.h \
    evaluatesoftkinetic.h \
    evaluate3dfrgc2.h
