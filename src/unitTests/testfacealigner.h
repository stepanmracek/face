#ifndef TESTFACEALIGNER_H
#define TESTFACEALIGNER_H

#include "facelib/facealigner.h"
#include "facelib/glwidget.h"

class TestFaceAligner
{
public:
    static void test(const QString &frgcPath, int argc, char *argv[])
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj"));
        Mesh m1 = Mesh::fromBIN(frgcPath + "bin/02463d654.bin", true); //"../../test/kinect/02-02.bin"
        m1.rotate(-0.1, 0.3, 0.1);

        int iterations = 20;
        //for (int iterations = 1; iterations <= 4; iterations++)
        //{
            int span = 0;
        //    for (int i = 0; i < 3; i++)
        //    {
                Mesh probe(m1);
                QDateTime date = QDateTime::currentDateTime();
                aligner.icpAlign(probe, iterations, FaceAligner::NoseTipDetection);
                span += date.msecsTo(QDateTime::currentDateTime());
        //    }
            span /= 3;
            qDebug() << iterations << span;
        //}


        QApplication app(argc, argv);
        GLWidget w;
        w.addFace(&probe);
        w.addFace(&aligner.referenceFace);
        w.show();
        app.exec();
    }
};

#endif // TESTFACEALIGNER_H
