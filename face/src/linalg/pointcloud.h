#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QVector>
#include <opencv/cv.h>

class PointCloud
{
public:
    static void toFile(const QVector<cv::Point3d> &points, const QString &path, bool append = false);
};

#endif // POINTCLOUD_H
