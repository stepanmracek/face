#include "serialization.h"

#include <opencv2/core/core.hpp>

void Serialization::serializeVectorOfPointclouds(QVector<VectorOfPoints> &data, const QString &path)
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

QVector<VectorOfPoints> Serialization::readVectorOfPointclouds(const QString &path)
{
    QVector<VectorOfPoints> result;

    cv::FileStorage storage(path.toStdString(), cv::FileStorage::READ);
    cv::FileNode dataNode = storage["data"];

    for (cv::FileNodeIterator it = dataNode.begin(); it != dataNode.end(); ++it)
    {
        cv::FileNode pointCloudNode = *it;
        VectorOfPoints pointCloud;
        for (cv::FileNodeIterator pcIt = pointCloudNode.begin(); pcIt != pointCloudNode.end(); ++pcIt)
        {
            std::vector<double> tmp;
            (*pcIt) >> tmp;
            cv::Point3d p(tmp[0], tmp[1], tmp[2]);
            pointCloud << p;
        }
        result << pointCloud;
    }

    return result;
}
