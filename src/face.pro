TEMPLATE = subdirs

SUBDIRS += \
    # common library
    faceCommon \
    # tests
    unitTests \
    integrationTests \
    # wrappers
    kinect \
    pclWrapper \
    vosm \
    # apps
    appEvaluation \
    appMorphFaceModel \
    appKinectAcquire \
    appHEF
