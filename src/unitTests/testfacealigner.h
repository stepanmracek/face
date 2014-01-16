#ifndef TESTFACEALIGNER_H
#define TESTFACEALIGNER_H

#include "facelib/facealigner.h"
#include "facelib/glwidget.h"

#include <QList>

class TestFaceAligner
{
private:
    static void oldRotate(Face::FaceData::Mesh &m, double x, double y, double z)
    {
        Face::FaceData::VectorOfPoints points;
        for (int r = 0; r < m.pointsMat.rows; r++)
        {
            points << cv::Point3d(m.pointsMat(r, 0), m.pointsMat(r, 1), m.pointsMat(r, 2));
        }
        Face::LinAlg::Procrustes3D::rotate(points, x, y, z);
        for (int r = 0; r < m.pointsMat.rows; r++)
        {
            m.pointsMat(r, 0) = points[r].x;
            m.pointsMat(r, 1) = points[r].y;
            m.pointsMat(r, 2) = points[r].z;
        }
    }

public:
    static void test(const QString &frgcPath, int argc, char *argv[])
    {
        Face::FaceData::FaceAligner aligner(Face::FaceData::Mesh::fromOBJ("../../test/meanForAlign.obj"));
        Face::FaceData::Mesh probe = Face::FaceData::Mesh::fromBINZ("../../test/softKinetic/03/DS32528233700098_radim/3000-20131224112242.binz");

        for (int iterations = 10; iterations <= 100; iterations++)
        {
            Face::FaceData::Mesh m(probe);
            QDateTime now = QDateTime::currentDateTime();
            aligner.icpAlign(m, iterations, Face::FaceData::FaceAligner::TemplateMatching);
            qDebug() << iterations << now.msecsTo(QDateTime::currentDateTime());
        }

        /*QApplication app(argc, argv);
        GLWidget w;
        w.addFace(&probe);
        w.addFace(&aligner.referenceFace);
        w.show();
        app.exec();*/
    }
};

#endif // TESTFACEALIGNER_H
