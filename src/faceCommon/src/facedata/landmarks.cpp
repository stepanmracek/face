#include "faceCommon/facedata/landmarks.h"
#include "faceCommon/linalg/procrustes.h"

using namespace Face::FaceData;

Landmarks::Landmarks(const std::string &path)
{
    cv::FileStorage readStorage(path, cv::FileStorage::READ);
    if (!readStorage.isOpened())
    {
        throw FACELIB_EXCEPTION("Can't read landmarks from " + path);
    }

    cv::FileNode landmarksNode = readStorage["landmarks"];
    for (cv::FileNodeIterator it = landmarksNode.begin(); it != landmarksNode.end(); ++it)
    {
        std::vector<double> tmp;
        (*it) >> tmp;
        Point p(tmp[0], tmp[1], tmp[2]);
        points.push_back(p);
    }
}

void Landmarks::serialize(const std::string &path) const
{
    cv::FileStorage writeStorage(path, cv::FileStorage::WRITE);
    writeStorage << "landmarks" << "[";
    for (const cv::Point3d &p : points)
    {
        writeStorage << p;
    }
    writeStorage << "]";
    writeStorage.release();
}

void Landmarks::translate(cv::Point3d shift)
{
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
        cv::Point3d &p = points[i];
        if (p.x != 0 && p.y != 0 && p.z != 0)
        {
            points[i] += shift;
        }
    }
}

void Landmarks::transform(Matrix &m)
{
    int n = points.size();
    for (int i = 0; i < n; i++)
    {
        cv::Point3d &p = points[i];
        if (p.x != 0 && p.y != 0 && p.z != 0)
        {
            Face::LinAlg::Procrustes3D::transform(p, m);
        }
    }
}
