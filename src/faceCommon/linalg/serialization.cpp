#include "serialization.h"

#include <opencv2/core/core.hpp>

void Serialization::serializeVectorOfPointClouds(QVector<VectorOfPoints> &data, const QString &path)
{
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::WRITE);
    storage << "data" << "[";

    foreach (const VectorOfPoints &pointcloud, data)
    {
        storage << "[";
        foreach (const cv::Point3d &p, pointcloud)
        {
            storage << p;
        }
        storage << "]";
    }
    storage << "]";
}
