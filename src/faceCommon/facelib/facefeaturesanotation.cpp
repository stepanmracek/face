#include "facefeaturesanotation.h"

#include <QDir>
#include <QSet>
#include <QFile>
#include <QDebug>
#include <QFileInfoList>
#include <cassert>
#include <opencv/cv.h>

#include "mesh.h"
#include "map.h"
#include "surfaceprocessor.h"
#include "landmarks.h"

struct FaceFeaturesAnotationStruct
{
    //Matrix depth;
    //Matrix curvature;
    Matrix texture;
    QVector<cv::Point2d> points;
    std::string windowName;
    //int mixture;
};

void FaceFeaturesAnotationShowFace(FaceFeaturesAnotationStruct &anotationStruct)
{
    // ((anotationStruct.mixture/10.0) * anotationStruct.depth + (1.0-anotationStruct.mixture/10.0)*anotationStruct.curvature);
    Matrix img = anotationStruct.texture.clone();

    foreach(const cv::Point2d &p, anotationStruct.points)
    {
        cv::circle(img, cv::Point(p.x, p.y), 2, cv::Scalar(0));
        cv::circle(img, cv::Point(p.x, p.y), 3, cv::Scalar(1));
    }

    QVector<cv::Point2d> &pts = anotationStruct.points;
    if (anotationStruct.points.count() == 15)
    {
        cv::line(img, pts[0], pts[1], 0);
        cv::line(img, pts[1], pts[2], 0);
        cv::line(img, pts[2], pts[3], 0);
        cv::line(img, pts[3], pts[4], 0);
        cv::line(img, pts[0], pts[11], 0);
        cv::line(img, pts[11], pts[12], 0);
        cv::line(img, pts[12], pts[13], 0);
        cv::line(img, pts[13], pts[14], 0);
        cv::line(img, pts[14], pts[4], 0);
        cv::line(img, pts[2], pts[6], 0);
        cv::line(img, pts[6], pts[8], 0);
        cv::line(img, pts[5], pts[6], 0);
        cv::line(img, pts[6], pts[7], 0);
        cv::line(img, pts[5], pts[8], 0);
        cv::line(img, pts[8], pts[7], 0);
        cv::line(img, pts[8], pts[9], 0);
        cv::line(img, pts[8], pts[10], 0);
        cv::line(img, pts[9], pts[10], 0);
    }

    cv::imshow(anotationStruct.windowName, img);
}

void FaceFeaturesAnotationTrackbarCallback(int pos, void* userdata)
{
    FaceFeaturesAnotationStruct &anotationStruct = *((FaceFeaturesAnotationStruct *)userdata);
    FaceFeaturesAnotationShowFace(anotationStruct);
}

void FaceFeaturesAnotationMouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        FaceFeaturesAnotationStruct &anotationStruct = *((FaceFeaturesAnotationStruct *)userdata);

        anotationStruct.points << cv::Point2d(x,y);
        FaceFeaturesAnotationShowFace(anotationStruct);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        FaceFeaturesAnotationStruct &anotationStruct = *((FaceFeaturesAnotationStruct *)userdata);

        if (anotationStruct.points.count() > 0)
        {
            anotationStruct.points.remove(anotationStruct.points.count() - 1);
            FaceFeaturesAnotationShowFace(anotationStruct);
        }
    }
}

Landmarks FaceFeaturesAnotation::anotate(Mesh &mesh, int desiredLandmarksCount)
{
    std::string windowName = "face";
    MapConverter converter, depthConverter;
    bool useTexture = mesh.colors.count() == mesh.points.count();
    Map map = SurfaceProcessor::depthmap(mesh, converter, 2.0, useTexture);
    Map depth = SurfaceProcessor::depthmap(mesh, depthConverter, 2.0);

    FaceFeaturesAnotationStruct anotationStruct;
    anotationStruct.texture = map.toMatrix();
    anotationStruct.windowName = windowName;

    cv::namedWindow(windowName);
    cv::setMouseCallback(windowName, FaceFeaturesAnotationMouseCallback, &anotationStruct);

    FaceFeaturesAnotationShowFace(anotationStruct);
    cv::waitKey(0);
    cv::destroyWindow(windowName);

    Landmarks l;
    if (anotationStruct.points.count() == desiredLandmarksCount)
    {
        for (int i = 0; i < desiredLandmarksCount; i++)
        {
            l.points[i] = depthConverter.MapToMeshCoords(depth, anotationStruct.points[i]);
        }
    }
    return l;
}

void FaceFeaturesAnotation::anotateOBJ(const QString &dirPath, bool uniqueIDsOnly)
{
    QDir dir(dirPath, "*.obj");
    assert(dir.exists());

    dir.setSorting(QDir::Name);
    QFileInfoList entries = dir.entryInfoList();
    std::string windowName = "face";

    //QSet<QString> usedIDs;

    foreach(const QFileInfo &fileInfo, entries)
    {
        QString filePath = fileInfo.absoluteFilePath();
        QString landmarksPath = dirPath + QDir::separator() + fileInfo.baseName() + ".xml";

        if (QFile::exists(landmarksPath)) continue;
        QString id = fileInfo.baseName().mid(0, 5);
        if (uniqueIDsOnly)
        {
            QDir landmarksDir(dirPath, id + "*.xml");
            if (landmarksDir.entryList().count() > 0)
            {
                continue;
            }
        }

        Mesh mesh = Mesh::fromOBJ(filePath, false);
        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(mesh, converter, 2);
        SurfaceProcessor::smooth(depth, 1, 2);
        CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(depth);

        FaceFeaturesAnotationStruct anotationStruct;
        //anotationStruct.curvature = cs.curvatureIndex.toMatrix();
        //anotationStruct.depth = depth.toMatrix();
        //anotationStruct.mixture = 7;
        anotationStruct.texture = 0.7*cs.curvatureIndex.toMatrix() + 0.3*depth.toMatrix();
        anotationStruct.windowName = windowName;

        cv::namedWindow(windowName);
        //cv::createTrackbar("depth/curvature", windowName, &anotationStruct.mixture, 10,
        //                   FaceFeaturesAnotationTrackbarCallback, &anotationStruct);
        cv::setMouseCallback(windowName, FaceFeaturesAnotationMouseCallback, &anotationStruct);

        FaceFeaturesAnotationShowFace(anotationStruct);
        //cv::imshow(windowName, anotationStruct.curvature);

        char key = cv::waitKey(0);
        cv::destroyWindow(windowName);

        Landmarks l;
        int lCount = l.points.size();
        qDebug() << "Count info" << lCount << anotationStruct.points.count();
        if (anotationStruct.points.count() == lCount)
        {
            for (int i = 0; i < lCount; i++)
            {
                l.points[i] = converter.MapToMeshCoords(depth, anotationStruct.points[i]);
            }
            l.serialize(landmarksPath);
        }

        if (key == 27)
            break;
    }
}
