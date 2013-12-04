#ifndef TESTFACEALIGNER_H
#define TESTFACEALIGNER_H

#include "facelib/facealigner.h"
#include "facelib/glwidget.h"
#include "biometrics/facetemplate.h"

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
        //FaceClassifier classifier("../../test/frgc/classifiers");
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj"));
        Mesh probe = Mesh::fromBIN(frgcPath + "bin/02463d654.bin", true); //"../../test/kinect/02-02.bin"
        probe.rotate(0.1, -0.1, 0);

        QDateTime now = QDateTime::currentDateTime();
        aligner.icpAlign(probe, 20, FaceAligner::NoseTipDetection);
        //Face3DTemplate t(0, probe, classifier);
        qDebug() << now.msecsTo(QDateTime::currentDateTime());

        QApplication app(argc, argv);
        GLWidget w;
        w.addFace(&probe);
        w.addFace(&aligner.referenceFace);
        w.show();
        app.exec();
    }

    static void testOpenMP()
    {
        QDateTime now = QDateTime::currentDateTime();
        for (int i = 0; i < 20; i++)
            innerLoop();
        qDebug() << now.msecsTo(QDateTime::currentDateTime());
    }

    static void innerLoop()
    {
        int n = 10000;
        Matrix result(n, 1);

        //#pragma omp parallel for
        for (int i = 0; i < n; i++)
        {
            Matrix points = Matrix::ones(1, 3);
            Procrustes3D::rotate(points, 1, 1, 1);
            result(i) = cv::sum(points)[0];
        }
    }
};

#endif // TESTFACEALIGNER_H
