#ifndef TESTMAP_H
#define TESTMAP_H

#include <QString>
#include <QDateTime>

#include "facedata/mesh.h"
#include "facedata/map.h"
#include "facedata/surfaceprocessor.h"
#include "facedata/facealigner.h"
#include "facedata/landmarkdetector.h"

class TestMap
{
public:
    static void testSerialization(const QString &pathToXYZ)
    {
        Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromXYZ(pathToXYZ);
        Face::FaceData::MapConverter c;
        Face::FaceData::Map map = Face::FaceData::SurfaceProcessor::depthmap(mesh, c, 1.0,
                                                                             Face::FaceData::SurfaceProcessor::ZCoord);
        map.serialize("serializedMap.map");

        Face::FaceData::Map deserialized("serializedMap.map");
        Matrix mat = deserialized.toMatrix(0.5);
        cv::imshow("deserialized", mat);
        double min, max;
        cv::minMaxIdx(mat, &min, &max);
        qDebug() << min << max;
        cv::waitKey();
    }

    static void testSmoothing()
    {
        Face::FaceData::Mesh mean = Face::FaceData::Mesh::fromOBJ("../../test/meanForAlign.obj");
        Face::FaceData::FaceAligner aligner(mean);

        Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBIN("/media/data/frgc/spring2004/bin/02463d652.bin");
        QDateTime dt = QDateTime::currentDateTime();
        aligner.icpAlign(mesh, 10, Face::FaceData::FaceAligner::NoseTipDetection);
        qDebug() << dt.msecsTo(QDateTime::currentDateTime());
    }
};

#endif // TESTMAP_H
