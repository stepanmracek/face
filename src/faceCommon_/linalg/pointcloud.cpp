#include "pointcloud.h"

#include <QFile>
#include <QIODevice>
#include <QTextStream>

void PointCloud::toFile(const QVector<cv::Point3d> &points, const QString &path, bool append)
{
    QFile f(path);
    if (append)
        f.open(QIODevice::WriteOnly | QIODevice::Append);
    else
        f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    foreach(const cv::Point3d &p, points)
    {
        out << p.x << ' ' << p.y << ' ' << p.z << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}
