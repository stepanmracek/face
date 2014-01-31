#-------------------------------------------------
#
# Project created by QtCreator 2013-03-17T09:59:54
#
#-------------------------------------------------

QT += core gui opengl widgets

TARGET = unitTests
TEMPLATE = app

INCLUDEPATH += "../faceCommon"

QMAKE_CXXFLAGS+= -fopenmp
QMAKE_LFLAGS +=  -fopenmp

LIBS += -L../faceCommon -lfaceCommon
LIBS += `pkg-config --libs opencv` -lGL -lGLU -lfreenect -lfreenect_sync

SOURCES += main.cpp

HEADERS += \
    testvector.h \
    testsvm.h \
    testsvd.h \
    testprocrustes.h \
    testpca.h \
    testmorphablefacemodel.h \
    testmetrics.h \
    testmesh.h \
    testlda.h \
    testlandmarks.h \
    testica.h \
    testglwidget.h \
    testgaoptimization.h \
    testgaborwavelet.h \
    testfacefeaturesdetection.h \
    testdiscriminativepotential.h \
    testdelaunay.h \
    testbiodataprocessing.h \
    testanotation.h \
    testmap.h \
    testhistogramfeatures.h \
    testdistance.h \
    testlogisticregression.h \
    testlaguerrewavelet.h \
    testfacealigner.h \
    testtextureprocessing.h \
    test.h \
    testgmm.h
