#ifndef TESTFACEFEATURESDETECTION_H
#define TESTFACEFEATURESDETECTION_H

#include <QDebug>
#include <QApplication>
#include <opencv/cv.h>
#include <QDir>
#include <QFileInfoList>

#include "gui/glwidget.h"
#include "facedata/mesh.h"
#include "facedata/map.h"
#include "facedata/surfaceprocessor.h"
#include "facedata/landmarkdetector.h"
#include "facedata/facealigner.h"
#include "linalg/vector.h"
#include "linalg/procrustes.h"
#include "facedata/maskedvector.h"
#include "facedata/facefeaturesanotation.h"
#include "facedata/landmarks.h"
#include "linalg/pca.h"
#include "gui/widgetmeshselect.h"
#include "linalg/kernelgenerator.h"
#include "linalg/gabor.h"
#include "linalg/gausslaguerre.h"

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
            Face::FaceData::Mesh m = Face::FaceData::Mesh::fromOBJ(info.absoluteFilePath());
            Face::FaceData::LandmarkDetector detector(m);
            Face::FaceData::Landmarks l = detector.detect();
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
            Face::FaceData::Mesh m = Face::FaceData::Mesh::fromOBJ(info.absoluteFilePath());
            QString lPath = dirPath + QDir::separator() + info.baseName() + ".xml";
            Face::FaceData::Landmarks l(lPath);

            Face::FaceData::MapConverter converter;
            Face::FaceData::Map depth = Face::FaceData::SurfaceProcessor::depthmap(m, converter, 1.0,
                                                                                   Face::FaceData::SurfaceProcessor::ZCoord);
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
            Face::FaceData::Mesh m = Face::FaceData::Mesh::fromOBJ(info.absoluteFilePath());
            QString lPath = dirPath + QDir::separator() + info.baseName() + "_auto.xml";
            Face::FaceData::Landmarks l(lPath);

            Face::FaceData::MapConverter converter;
            Face::FaceData::Map depthmap =
                    Face::FaceData::SurfaceProcessor::depthmap(m, converter, 1.0, Face::FaceData::SurfaceProcessor::ZCoord);

            QString ptsPath = dirPath + QDir::separator() + info.baseName() + ".pts";
            QFile ptsFile(ptsPath);
            ptsFile.open(QFile::WriteOnly);
            QTextStream ptsStream(&ptsFile);
            ptsStream << "version: 1\nn_points: 3\n{\n";

            cv::Point2d p2d;
            p2d = converter.MeshToMapCoords(depthmap, l.points[Face::FaceData::Landmarks::LeftInnerEye]);
            ptsStream << p2d.x << " " << p2d.y << "\n";
            p2d = converter.MeshToMapCoords(depthmap, l.points[Face::FaceData::Landmarks::RightInnerEye]);
            ptsStream << p2d.x << " " << p2d.y << "\n";
            p2d = converter.MeshToMapCoords(depthmap, l.points[Face::FaceData::Landmarks::Nosetip]);
            ptsStream << p2d.x << " " << p2d.y << "\n";

            ptsStream << "}\n";
        }
    }

    static int  testLandmarkDetection(int argc, char *argv[], QString pathToXYZ)
    {
        Face::FaceData::Mesh face = Face::FaceData::Mesh::fromXYZ(pathToXYZ);
        Face::FaceData::LandmarkDetector detector(face);
        Face::FaceData::Landmarks landmarks = detector.detect();

        QApplication app(argc, argv);
        Face::GUI::GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&face);
        widget.addLandmarks(&landmarks);
        widget.show();

        return app.exec();
    }

    static void testGoodAnotation(const QString &dirPath)
    {
        QDir dir(dirPath, "*.xml");
        QFileInfoList entries = dir.entryInfoList();
        foreach (const QFileInfo &e, entries)
        {
            Face::FaceData::Landmarks l(e.absoluteFilePath());
            if (!l.check())
            {
                qDebug() << e.fileName() << "didn't pass the landmark.check()";
            }
        }
    }
};

#endif // TESTFACEFEATURESDETECTION_H
