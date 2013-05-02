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
        Mesh mesh = Mesh::fromXYZ(dir + QDir::separator() + "02463d652.abs.xyz", false);
        mesh.printStats();

        //Landmarks landmarks(dir + "02463d652.xml");
        /*mesh.move(-landmarks.Nosetip);
        landmarks.LeftInnerEye -= landmarks.Nosetip;
        landmarks.RightInnerEye -= landmarks.Nosetip;
        landmarks.Nosetip = cv::Point3d(0,0,0);*/

        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(mesh, converter, 1, ZCoord);
        cv::imshow("depth", depth.toMatrix());

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        //widget.addLandmarks(&landmarks);
        widget.show();
        return app.exec();
    }
};

#endif // TESTGLWIDGET_H
