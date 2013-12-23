#ifndef TESTMAP_H
#define TESTMAP_H

#include <QString>
#include <QDateTime>

#include "facelib/mesh.h"
#include "facelib/map.h"
#include "facelib/surfaceprocessor.h"
#include "facelib/facealigner.h"
#include "facelib/landmarkdetector.h"

class TestMap
{
public:
    static void testSerialization(const QString &pathToXYZ)
    {
        Mesh mesh = Mesh::fromXYZ(pathToXYZ);
        MapConverter c;
        Map map = SurfaceProcessor::depthmap(mesh, c, 1.0, ZCoord);
        map.serialize("serializedMap.map");

        Map deserialized("serializedMap.map");
        Matrix mat = deserialized.toMatrix(0.5);
        cv::imshow("deserialized", mat);
        double min, max;
        cv::minMaxIdx(mat, &min, &max);
        qDebug() << min << max;
        cv::waitKey();
    }

    static void testSmoothing()
    {
        Mesh mean = Mesh::fromOBJ("../../test/meanForAlign.obj");
        FaceAligner aligner(mean);

        Mesh mesh = Mesh::fromBIN("/media/data/frgc/spring2004/bin/02463d652.bin");
        QDateTime dt = QDateTime::currentDateTime();
        aligner.icpAlign(mesh, 10, FaceAligner::NoseTipDetection);
        qDebug() << dt.msecsTo(QDateTime::currentDateTime());
    }
};

#endif // TESTMAP_H
