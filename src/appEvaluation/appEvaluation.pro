#-------------------------------------------------
#
# Project created by QtCreator 2013-03-17T09:59:54
#
#-------------------------------------------------

QT += core gui opengl

TARGET = appEvaluation
TEMPLATE = app

INCLUDEPATH += "../faceCommon"

LIBS += -L../faceCommon -lfaceCommon

SOURCES += main.cpp

HEADERS += \
    evaluatethermo2.h \
    evaluatethermo.h \
    evaluatefeaturestability.h \
    evaluatefeatureselection.h \
    evaluatefeaturelevelfusion.h \
    evaluateICA.h \
    evaluate3dfrgc.h
