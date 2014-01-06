#ifndef TESTFACEALIGNER_H
#define TESTFACEALIGNER_H

#include "facelib/facealigner.h"
#include "facelib/glwidget.h"

#include <QList>

class TestFaceAligner
{
private:
    static void oldRotate(Mesh &m, double x, double y, double z)
    {
        VectorOfPoints points;
        for (int r = 0; r < m.pointsMat.rows; r++)
        {
            points << cv::Point3d(m.pointsMat(r, 0), m.pointsMat(r, 1), m.pointsMat(r, 2));
        }
        Procrustes3D::rotate(points, x, y, z);
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
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj"));
        Mesh probe = Mesh::fromBINZ("../../test/softKinetic/03/DS32528233700098_radim/3000-20131224112242.binz");

        for (int iterations = 10; iterations <= 100; iterations++)
        {
            Mesh m(probe);
            QDateTime now = QDateTime::currentDateTime();
            aligner.icpAlign(m, iterations, FaceAligner::TemplateMatching);
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
