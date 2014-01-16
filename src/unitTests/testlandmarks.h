#ifndef TESTLANDMARKS_H
#define TESTLANDMARKS_H

#include <QDir>
#include <opencv2/core/core.hpp>

#include "facelib/mesh.h"
#include "facelib/landmarks.h"

class TestLandmarks
{
public:
    static void testReadWrite()
    {
        Face::FaceData::VectorOfPoints points;
        points << cv::Point3d(1,2,3);
        points << cv::Point3d(4,5,6);
        points << cv::Point3d(7,8,9);

        cv::FileStorage writeStorage("../test/landmarksWrite.xml", cv::FileStorage::WRITE);
        writeStorage << "landmarks" << "[";
        foreach (const cv::Point3d &p, points)
        {
            writeStorage << p;
        }
        writeStorage << "]";
        writeStorage.release();

        cv::FileStorage readStorage("../test/landmarksWrite.xml", cv::FileStorage::READ);
        cv::FileNode landmarksNode = readStorage["landmarks"];
        for (cv::FileNodeIterator it = landmarksNode.begin(); it != landmarksNode.end(); ++it)
        {
            std::vector<double> tmp;
            (*it) >> tmp;
            qDebug() << tmp.size() <<  tmp[0] << tmp[1] << tmp[2];
        }
    }
};

#endif // TESTLANDMARKS_H
