#ifndef EVALUATEKINECT_H
#define EVALUATEKINECT_H

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>

#include "facelib/mesh.h"
#include "facelib/surfaceprocessor.h"

class EvaluateKinect
{
public:
    static void isoCurves()
    {
        QDir dir("../../test/kinect/", "*.bin");
        QFileInfoList binFiles = dir.entryInfoList();
        foreach(const QFileInfo &info, binFiles)
        {
            //SurfaceProcessor::isoGeodeticCurve()
        }
    }
};

#endif // EVALUATEKINECT_H
