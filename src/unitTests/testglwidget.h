#ifndef TESTGLWIDGET_H
#define TESTGLWIDGET_H

#include <QApplication>
#include <QDir>

#include "facelib/glwidget.h"
#include "facelib/mesh.h"
#include "facelib/landmarks.h"
#include "facelib/surfaceprocessor.h"
#include "linalg/serialization.h"
#include "linalg/loader.h"
#include "facelib/facealigner.h"

class TestGlWidget
{
public:

    static int test(int argc, char *argv[])
    {
        Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBINZ("../../test/softKinetic/01/03-02.binz", false);

        //for (int i = 0; i < 10; i++)
        //    SurfaceProcessor::zsmoothFlann(mesh, 50.0, 1, 1);
        Face::FaceData::SurfaceProcessor::zsmooth(mesh, 0.5, 10);

        mesh.printStats();
        //mesh.colors.clear();

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        widget.show();
        return app.exec();
    }
};

#endif // TESTGLWIDGET_H
