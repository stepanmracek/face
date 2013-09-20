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
    static int test(int argc, char *argv[], const QString &dir)
    {
        //Mesh align = Mesh::fromOBJ("../../test/meanForAlign.obj");
        //align.translate(cv::Point3d(0, 0, 10));

        Mesh mesh = Mesh::fromBINZ(dir + "zbin-aligned/04765d158.binz", false);
        QVector<VectorOfPoints> isoCurves = Serialization::readVectorOfPointclouds(dir + "zbin-aligned/isocurves2/04765d158.xml");

        //qDebug() << mesh.points.count() << align.points.count();

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");

        widget.addFace(&mesh);
        //widget.addFace(&align);

        for (int i = 0; i < 5; i++)
            widget.addCurve(isoCurves[i]);
        widget.show();
        return app.exec();
    }
};

#endif // TESTGLWIDGET_H
