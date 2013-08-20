QT += core gui opengl widgets
TARGET = faceCommon
TEMPLATE = lib

LIBS += `pkg-config --libs opencv` -lGL -lGLU

SOURCES += \
    biometrics/featurepotentialbase.cpp \
    biometrics/featurelevelfusion.cpp \
    biometrics/eerpotential.cpp \
    biometrics/scorelevefusion.cpp \
    biometrics/template.cpp \
    biometrics/geneticweightoptimization.cpp \
    biometrics/biodataprocessing.cpp \
    biometrics/featureselection.cpp \
    biometrics/evaluation.cpp \
    biometrics/discriminativepotential.cpp \
    linalg/procrustes.cpp \
    linalg/ldaofpca.cpp \
    linalg/vector.cpp \
    linalg/loader.cpp \
    linalg/delaunay.cpp \
    linalg/common.cpp \
    linalg/logisticregression.cpp \
    linalg/dataviz.cpp \
    linalg/lda.cpp \
    linalg/ica.cpp \
    linalg/random.cpp \
    linalg/haardetect.cpp \
    linalg/metrics.cpp \
    linalg/icaofpca.cpp \
    linalg/pca.cpp \
    linalg/normalization.cpp \
    facetrack/realtimetrack.cpp \
    facetrack/bioidprocess.cpp \
    facelib/map.cpp \
    facelib/util.cpp \
    facelib/landmarkdetector.cpp \
    facelib/mesh.cpp \
    facelib/maskedvector.cpp \
    facelib/surfaceprocessor.cpp \
    facelib/glwidget.cpp \
    linalg/pointcloud.cpp \
    facelib/facefeaturesanotation.cpp \
    facelib/landmarks.cpp \
    facelib/morphable3dfacemodel.cpp \
    facelib/morphable3dfacemodelwidget.cpp \
    facelib/widgetmeshselect.cpp \
    linalg/kernelgenerator.cpp \
    linalg/gabor.cpp \
    linalg/filterbank.cpp \
    linalg/gausslaguerre.cpp \
    facelib/facealigner.cpp \
    linalg/serialization.cpp \
    biometrics/isocurveprocessing.cpp \
    biometrics/featureextractor.cpp \
    linalg/histogram.cpp \
    biometrics/histogramfeatures.cpp \
    linalg/adaboost.cpp \
    biometrics/system.cpp \
    biometrics/scorelevelfusionwrapper.cpp \
    biometrics/zpcacorrw.cpp

HEADERS += \
    kinect/kinnect.h \
    biometrics/biodataprocessing.h \
    biometrics/geneticweightoptimization.h \
    biometrics/template.h \
    biometrics/featureselection.h \
    biometrics/featurepotentialbase.h \
    biometrics/featureextractor.h \
    biometrics/featurelevelfusion.h \
    biometrics/eerpotential.h \
    biometrics/discriminativepotential.h \
    biometrics/evaluation.h \
    biometrics/scorelevefusion.h \
    linalg/matrixconverter.h \
    linalg/haardetect.h \
    linalg/metrics.h \
    linalg/procrustes.h \
    linalg/random.h \
    linalg/icaofpca.h \
    linalg/projectionbase.h \
    linalg/vector.h \
    linalg/svd.h \
    linalg/pca.h \
    linalg/normalization.h \
    linalg/loader.h \
    linalg/ldaofpca.h \
    linalg/ica.h \
    linalg/delaunay.h \
    linalg/lda.h \
    linalg/logisticregression.h \
    linalg/dataviz.h \
    linalg/common.h \
    facetrack/bioidprocess.h \
    facetrack/realtimetrack.h \
    evaluation/evaluateICA.h \
    evaluation/evaluate3dfrgc.h \
    evaluation/evaluatefeaturestability.h \
    evaluation/evaluatethermo2.h \
    evaluation/evaluatefeatureselection.h \
    evaluation/evaluatefeaturelevelfusion.h \
    evaluation/evaluatethermo.h \
    facelib/util.h \
    facelib/map.h \
    facelib/landmarkdetector.h \
    facelib/mesh.h \
    facelib/surfaceprocessor.h \
    facelib/maskedvector.h \
    facelib/glwidget.h \
    linalg/pointcloud.h \
    facelib/facefeaturesanotation.h \
    facelib/landmarks.h \
    facelib/morphable3dfacemodel.h \
    facelib/morphable3dfacemodelwidget.h \
    facelib/widgetmeshselect.h \
    linalg/kernelgenerator.h \
    linalg/gabor.h \
    linalg/filterbank.h \
    linalg/gausslaguerre.h \
    facelib/facealigner.h \
    linalg/serialization.h \
    biometrics/isocurveprocessing.h \
    linalg/histogram.h \
    biometrics/histogramfeatures.h \
    linalg/adaboost.h \
    biometrics/system.h \
    biometrics/scorelevelfusionwrapper.h \
    biometrics/zpcacorrw.h

FORMS += \
    facelib/morphable3dfacemodelwidget.ui \
    facelib/widgetmeshselect.ui
