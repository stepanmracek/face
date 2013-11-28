#ifndef TESTFACEFEATURESDETECTION_H
#define TESTFACEFEATURESDETECTION_H

#include <QDebug>
#include <QApplication>
#include <opencv/cv.h>
#include <QDir>
#include <QFileInfoList>

#include "facelib/glwidget.h"
#include "facelib/mesh.h"
#include "facelib/map.h"
#include "facelib/surfaceprocessor.h"
#include "facelib/landmarkdetector.h"
#include "facelib/facealigner.h"
#include "linalg/vector.h"
#include "linalg/procrustes.h"
#include "linalg/pointcloud.h"
#include "facelib/maskedvector.h"
#include "facelib/facefeaturesanotation.h"
#include "facelib/landmarks.h"
#include "linalg/pca.h"
#include "facelib/widgetmeshselect.h"
#include "linalg/kernelgenerator.h"
#include "linalg/gabor.h"
#include "linalg/gausslaguerre.h"
#include "facelib/morphable3dfacemodel.h"

enum AlignType
{
    simple, isoCurve, triangle
};

class TestFaceFeatuesDetection
{
public:
    static int  testBatchLandmarkDetection(int argc, char *argv[], QString dirPath)
    {
        /*QApplication app(argc, argv);
        WidgetMeshSelect widget;
        QStringList filter; filter << "*.obj";
        widget.setPath(dirPath, filter);
        widget.show();
        return app.exec();*/

        QDir dir(dirPath);
        QStringList filter; filter << "*.obj";
        QFileInfoList list = dir.entryInfoList(filter, QDir::Files);
        foreach (const QFileInfo &info, list)
        {
            Mesh m = Mesh::fromOBJ(info.absoluteFilePath());
            LandmarkDetector detector(m);
            Landmarks l = detector.detect();
            QString lPath = dirPath + QDir::separator() + info.baseName() + "_auto.xml";
            l.serialize(lPath);
        }
    }

    static int  testSuccessBatchLandmarkDetection(QString dirPath)
    {
        QDir dir(dirPath);
        QStringList filter; filter << "*.obj";
        QFileInfoList list = dir.entryInfoList(filter, QDir::Files);
        foreach (const QFileInfo &info, list)
        {
            Mesh m = Mesh::fromOBJ(info.absoluteFilePath());
            QString lPath = dirPath + QDir::separator() + info.baseName() + ".xml";
            Landmarks l(lPath);

            MapConverter converter;
            Map depth = SurfaceProcessor::depthmap(m, converter, 1.0, ZCoord);
            Matrix img = depth.toMatrix() * 255;

            for (int i = 0; i < l.points.size(); i++)
            {
                const cv::Point3d &p = l.points[i];
                cv::Point2d mapPoint = converter.MeshToMapCoords(depth, p);
                cv::circle(img, cv::Point(mapPoint.x, mapPoint.y), 2, 0);

                if (i > 0)
                {
                    cv::Point2d prevPoint = converter.MeshToMapCoords(depth, l.points[i-1]);
                    cv::line(img, prevPoint, mapPoint, 0);
                }
            }

            QString imgPath = dirPath + QDir::separator() + info.baseName() + ".png";
            cv::imwrite(imgPath.toStdString(), img);
        }
    }

    static void exportInitialEstimationsForVOSM(QString dirPath)
    {
        QDir dir(dirPath);
        QStringList filter; filter << "*.obj";
        QFileInfoList list = dir.entryInfoList(filter, QDir::Files);
        foreach (const QFileInfo &info, list)
        {
            Mesh m = Mesh::fromOBJ(info.absoluteFilePath());
            QString lPath = dirPath + QDir::separator() + info.baseName() + "_auto.xml";
            Landmarks l(lPath);

            MapConverter converter;
            Map depthmap = SurfaceProcessor::depthmap(m, converter, 1.0, ZCoord);

            QString ptsPath = dirPath + QDir::separator() + info.baseName() + ".pts";
            QFile ptsFile(ptsPath);
            ptsFile.open(QFile::WriteOnly);
            QTextStream ptsStream(&ptsFile);
            ptsStream << "version: 1\nn_points: 3\n{\n";

            cv::Point2d p2d;
            p2d = converter.MeshToMapCoords(depthmap, l.points[Landmarks::LeftInnerEye]);
            ptsStream << p2d.x << " " << p2d.y << "\n";
            p2d = converter.MeshToMapCoords(depthmap, l.points[Landmarks::RightInnerEye]);
            ptsStream << p2d.x << " " << p2d.y << "\n";
            p2d = converter.MeshToMapCoords(depthmap, l.points[Landmarks::Nosetip]);
            ptsStream << p2d.x << " " << p2d.y << "\n";

            ptsStream << "}\n";
        }
    }

    static int  testLandmarkDetection(int argc, char *argv[], QString pathToXYZ)
    {
        Mesh face = Mesh::fromXYZ(pathToXYZ);
        LandmarkDetector detector(face);
        Landmarks landmarks = detector.detect();

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&face);
        widget.addLandmarks(&landmarks);
        widget.show();

        return app.exec();
    }

    static int  testAlign(int argc, char *argv[], QString dirPath, QString fileName)
    {
        //Mesh mean = Mesh::fromOBJ("../../test/meanForAlign.obj");
        Morphable3DFaceModel model("../../test/morph-pca-zcoord.xml",
                                   "../../test/morph-pca-texture.xml",
                                   "../../test/morph-pca.xml",
                                   "../../test/morph-flags",
                                   "../../test/morph-landmarks.xml",
                                   200);
        Mesh mean = model.mesh;
        mean.translate(-model.landmarks.get(Landmarks::Nosetip));

        FaceAligner aligner(mean);
        Mesh face = Mesh::fromBIN(dirPath + "bin/" + fileName, true);

        //MapConverter c;
        //Map before = SurfaceProcessor::depthmap(face, c, 2, Texture_I);

        //Mesh old = face;
        aligner.icpAlign(face, 10, FaceAligner::NoseTipDetection);

        //Map after = SurfaceProcessor::depthmap(face, c, 2, Texture_I);

        //cv::imshow("before", before.toMatrix());
        //cv::imshow("after", after.toMatrix());
        //cv::waitKey(0);

        //old.translate(cv::Point3d(100,0,0));
        //face.translate(cv::Point3d(-100,0,0));
        //aligner.meanFace.translate(cv::Point3d(-100,0,0));

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&aligner.referenceFace);
        widget.addFace(&face);
        //widget.addFace(&old);
        widget.show();
        return app.exec();
    }

    static void testGoodAnotation(const QString &dirPath)
    {
        QDir dir(dirPath, "*.xml");
        QFileInfoList entries = dir.entryInfoList();
        foreach (const QFileInfo &e, entries)
        {
            Landmarks l(e.absoluteFilePath());
            if (!l.check())
            {
                qDebug() << e.fileName() << "didn't pass the landmark.check()";
            }
        }
    }
};

#endif // TESTFACEFEATURESDETECTION_H
