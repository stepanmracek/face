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
    if (points.count() != 9)
    {
        qDebug() << "  didn't pass count check";
        return false;
    }

    // check x-coordinates of the eye-line
    for (int i = 0; i <= 3; i++)
    {
        if (points[i].x >= points[i+1].x)
        {
            qDebug() << "  didn't pass eye-line check";
            return false;
        }
    }

    // check x-coordinated of nose-line
    for (int i = 5; i <= 6; i++)
    {
        if (points[i].x >= points[i+1].x)
        {
            qDebug() << "  didn't pass nose-line check";
            return false;
        }
    }

    // chek that nose-line is below eye-line
    for (int eyeIndex = 0; eyeIndex <= 4; eyeIndex++)
    {
        for (int noseIndex = 5; noseIndex <= 8; noseIndex++)
        {
            if (points[noseIndex].y >= points[eyeIndex].y)
            {
                qDebug() << "  didn't pass nose-line below eyes-line check";
                return false;
            }
        }
    }

    // check that below-nose is under nose-tip
    if (points[LowerNose].y >= points[Nosetip].y)
    {
        qDebug() << "  didn't pass nose-tip above lower-nose";
        return false;
    }

    // check that nose is in the front
    if ((points[Nosetip].z <= points[LeftOuterNose].z) || (points[Nosetip].z <= points[RightOuterNose].z))
    {
        qDebug() << "  didn't pass nose in front check";
        return false;
    }

    return true;
}
