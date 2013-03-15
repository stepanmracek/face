#include "landmarks.h"

#include <QDebug>
#include <QFile>
#include <QTextStream>

Landmarks::Landmarks(const QString &path)
{
    cv::FileStorage readStorage(path.toStdString(), cv::FileStorage::READ);
    cv::FileNode landmarksNode = readStorage["landmarks"];
    for (cv::FileNodeIterator it = landmarksNode.begin(); it != landmarksNode.end(); ++it)
    {
        std::vector<double> tmp;
        (*it) >> tmp;
        cv::Point3d p(tmp[0], tmp[1], tmp[2]);
        points << p;
    }
}

void Landmarks::serialize(const QString &path)
{
    cv::FileStorage writeStorage(path.toStdString(), cv::FileStorage::WRITE);
    assert(writeStorage.isOpened());
    writeStorage << "landmarks" << "[";
    foreach (const cv::Point3d &p, points)
    {
        writeStorage << p;
    }
    writeStorage << "]";
    writeStorage.release();
}

bool Landmarks::check()
{
    if (points.count() < 8) return false;
    //qDebug() << "  passed count check";

    // check x-coordinates of the eye-line
    for (int i = 0; i <= 3; i++)
    {
        if (points[i].x >= points[i+1].x) return false;
    }
    //qDebug() << "  passed eye-line check";

    // check x-coordinated of nose-line
    for (int i = 5; i <= 6; i++)
    {
        if (points[i].x >= points[i+1].x) return false;
    }
    //qDebug() << "  passed nose-line check";

    // chek that nose-line is below eye-line
    for (int eyeIndex = 0; eyeIndex <= 4; eyeIndex++)
    {
        for (int noseIndex = 5; noseIndex <=7; noseIndex++)
        {
            if (points[noseIndex].y >= points[eyeIndex].y) return false;
        }
    }
    //qDebug() << "  passed below check";

    // check that nose is in the front
    if (points[6].z <= points[5].z) return false;
    if (points[6].z <= points[7].z) return false;
    //qDebug() << "  passed nose check";

    return true;
}
