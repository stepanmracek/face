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
        m1.rotate(0.3, -0.4, 0);

        Mesh probe(m1);
        aligner.icpAlign(probe, 20, FaceAligner::NoseTipDetection);

        QApplication app(argc, argv);
        GLWidget w;
        w.addFace(&probe);
        w.addFace(&aligner.referenceFace);
        w.show();
        app.exec();
    }
};

#endif // TESTFACEALIGNER_H
