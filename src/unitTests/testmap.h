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
        cv::imshow("deserialized", deserialized.toMatrix(-1000));
        cv::waitKey();
    }
};

#endif // TESTMAP_H
