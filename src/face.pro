TEMPLATE = subdirs

SUBDIRS += \
    # common library
    faceCommon \
    # tests
    unitTests \
    integrationTests \
    # wrappers
    faceSensors \
    #pclWrapper \
    #vosm \
    # apps
    appEvaluation \
    appMorphFaceModel \
    appKinectAcquire \
    appHEF
