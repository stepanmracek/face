#ifndef TESTGLWIDGET_H
#define TESTGLWIDGET_H

#include <QApplication>
#include <QDir>

#include "facelib/glwidget.h"
#include "facelib/mesh.h"
#include "facelib/landmarks.h"
#include "facelib/surfaceprocessor.h"
#include "linalg/serialization.h"

class TestGlWidget
{
public:
    static int test(int argc, char *argv[])
    {
        Mesh mesh = Mesh::fromBIN("../../test/kinect/01-01.bin", false);
        mesh.printStats();

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        widget.show();
        return app.exec();
    }
};

#endif // TESTGLWIDGET_H
