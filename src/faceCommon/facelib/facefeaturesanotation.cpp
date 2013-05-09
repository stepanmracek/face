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
#include "linalg/kernelgenerator.h"

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

Landmarks FaceFeaturesAnotation::anotate(Mesh &mesh, bool &success)
{
    std::string windowName = "face";
    MapConverter textureConverter;
    Map texture = SurfaceProcessor::depthmap(mesh, textureConverter, 2.0, Texture_I);

    FaceFeaturesAnotationStruct anotationStruct;
    anotationStruct.texture = texture.toMatrix();
    anotationStruct.windowName = windowName;

    cv::namedWindow(windowName);
    cv::setMouseCallback(windowName, FaceFeaturesAnotationMouseCallback, &anotationStruct);

    FaceFeaturesAnotationShowFace(anotationStruct);
    cv::waitKey(0);
    cv::destroyWindow(windowName);

    Landmarks l;
    if (anotationStruct.points.count() == l.points.count())
    {
        MapConverter depthConverter;
        Map depth = SurfaceProcessor::depthmap(mesh, depthConverter, 2.0, ZCoord);

        success = true;
        for (int i = 0; i < l.points.count(); i++)
        {
            l.points[i] = depthConverter.MapToMeshCoords(depth, anotationStruct.points[i]);
        }
        success = l.check();
    }
    else
    {
        success = false;
    }
    return l;
}

void FaceFeaturesAnotation::anotateBINs(const QString &dirPath, bool uniqueIDsOnly, bool overwrite)
{
    QDir dir(dirPath, "*.bin");
    assert(dir.exists());
    QFileInfoList entries = dir.entryInfoList();

    foreach(const QFileInfo &fileInfo, entries)
    {
        qDebug() << fileInfo.baseName();

        QString filePath = fileInfo.absoluteFilePath();
        QString landmarksPath = dirPath + QDir::separator() + fileInfo.baseName() + ".xml";

        if (!overwrite && QFile::exists(landmarksPath))
        {
            // overwrite only if landmarks don't pass the check
            Landmarks lm(landmarksPath);
            if (lm.check()) continue;
        }

        QString id = fileInfo.baseName().split("d")[0];
        if (uniqueIDsOnly)
        {
            QDir landmarksDir(dirPath, id + "*.xml");
            if (landmarksDir.entryList().count() > 0)
            {
                continue;
            }
        }

        Mesh mesh = Mesh::fromBIN(filePath, false);
        bool success;
        Landmarks lm = anotate(mesh, success);
        if (success) lm.serialize(landmarksPath);

        char key = cv::waitKey();
        if (key == 27) break;
    }
}
