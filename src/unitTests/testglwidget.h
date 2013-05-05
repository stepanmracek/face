#ifndef TESTGLWIDGET_H
#define TESTGLWIDGET_H

#include <QApplication>
#include <QDir>

#include "facelib/glwidget.h"
#include "facelib/mesh.h"
#include "facelib/landmarks.h"
#include "facelib/surfaceprocessor.h"

class TestGlWidget
{
public:
    static int test(int argc, char *argv[], const QString &dir)
    {
        Mesh mesh = Mesh::fromBIN(dir + "bin/02463d652.bin", false);

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        widget.show();
        return app.exec();
    }
};

#endif // TESTGLWIDGET_H
