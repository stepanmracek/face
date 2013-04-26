#ifndef TESTMAP_H
#define TESTMAP_H

#include <QString>

#include "facelib/mesh.h"
#include "facelib/map.h"
#include "facelib/surfaceprocessor.h"

class TestMap
{
public:
    static void testSerialization(const QString &pathToXYZ)
    {
        Mesh mesh = Mesh::fromXYZFile(pathToXYZ);
        MapConverter c;
        Map map = SurfaceProcessor::depthmap(mesh, c, 1.0, ZCoord);
        map.serialize("serializedMap.map");

        Map deserialized("serializedMap.map");
        Matrix mat = deserialized.toMatrix(0.5);
        cv::imshow("deserialized", mat);
        double min, max;
        Common::getMinMax(mat, min, max);
        qDebug() << min << max;
        cv::waitKey();
    }
};

#endif // TESTMAP_H
