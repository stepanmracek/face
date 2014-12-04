#include "faceCommon/linalg/serialization.h"

#include <opencv2/core/core.hpp>

using namespace Face::LinAlg;

void Serialization::serializeVectorOfPointclouds(std::vector<Face::FaceData::VectorOfPoints> &data, const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "data" << "[";

    for (const Face::FaceData::VectorOfPoints &pointcloud : data)
    {
        storage << "[";
        for (const cv::Point3d &p : pointcloud)
        {
            storage << p;
        }
        storage << "]";
    }
    storage << "]";
}

std::vector<Face::FaceData::VectorOfPoints> Serialization::readVectorOfPointclouds(const std::string &path)
{
    std::vector<Face::FaceData::VectorOfPoints> result;

    cv::FileStorage storage(path, cv::FileStorage::READ);
    cv::FileNode dataNode = storage["data"];

    for (cv::FileNodeIterator it = dataNode.begin(); it != dataNode.end(); ++it)
    {
        cv::FileNode pointCloudNode = *it;
        FaceData::VectorOfPoints pointCloud;
        for (cv::FileNodeIterator pcIt = pointCloudNode.begin(); pcIt != pointCloudNode.end(); ++pcIt)
        {
            std::vector<double> tmp;
            (*pcIt) >> tmp;
            cv::Point3d p(tmp[0], tmp[1], tmp[2]);
            pointCloud.push_back(p);
        }
        result.push_back(pointCloud);
    }

    return result;
}
