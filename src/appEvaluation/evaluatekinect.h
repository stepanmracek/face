#ifndef EVALUATEKINECT_H
#define EVALUATEKINECT_H

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>

#include "facelib/mesh.h"
#include "facelib/surfaceprocessor.h"
#include "linalg/common.h"

class EvaluateKinect
{
public:
    static void isoCurves()
    {
        QDir dir("../../test/kinect/", "*.bin");
        QFileInfoList binFiles = dir.entryInfoList();
        foreach(const QFileInfo &info, binFiles)
        {
            Mesh face = Mesh::fromBIN(info.absoluteFilePath());
            MapConverter converter;
            Map depth = SurfaceProcessor::depthmap(face, converter, 2, ZCoord);
            cv::Point3d nosetip(0,0,0);
            for (int distance = 20; distance <= 60; distance += 5)
            {
                VectorOfPoints isoCurve = SurfaceProcessor::isoGeodeticCurve(depth, converter, nosetip, distance, 100, 2);
                Common::savePlot(isoCurve, "isoCurves", distance != 20);
            }
            break;
        }
    }
};

#endif // EVALUATEKINECT_H
