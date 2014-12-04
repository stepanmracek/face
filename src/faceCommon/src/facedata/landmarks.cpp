#include "faceCommon/facedata/landmarks.h"

using namespace Face::FaceData;

Landmarks::Landmarks(const std::string &path)
{
    cv::FileStorage readStorage(path, cv::FileStorage::READ);
    cv::FileNode landmarksNode = readStorage["landmarks"];
    for (cv::FileNodeIterator it = landmarksNode.begin(); it != landmarksNode.end(); ++it)
    {
        std::vector<double> tmp;
        (*it) >> tmp;
        cv::Point3d p(tmp[0], tmp[1], tmp[2]);
        points.push_back(p);
    }
}

void Landmarks::serialize(const std::string &path)
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

bool Landmarks::check()
{
    if (points.size() != 9)
    {
        std::cerr << "  didn't pass count check" << std::endl;
        return false;
    }

    // check x-coordinates of the eye-line
    for (int i = 0; i <= 3; i++)
    {
        if (points[i].x >= points[i+1].x)
        {
            std::cerr << "  didn't pass eye-line check" << std::endl;
            return false;
        }
    }

    // check x-coordinated of nose-line
    for (int i = 5; i <= 6; i++)
    {
        if (points[i].x >= points[i+1].x)
        {
            std::cerr << "  didn't pass nose-line check" << std::endl;
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
                std::cerr << "  didn't pass nose-line below eyes-line check" << std::endl;
                return false;
            }
        }
    }

    // check that below-nose is under nose-tip
    if (points[LowerNose].y >= points[Nosetip].y)
    {
        std::cerr << "  didn't pass nose-tip above lower-nose" << std::endl;
        return false;
    }

    // check that nose is in the front
    if ((points[Nosetip].z <= points[LeftOuterNose].z) || (points[Nosetip].z <= points[RightOuterNose].z))
    {
        std::cerr << "  didn't pass nose in front check" << std::endl;
        return false;
    }

    return true;
}
