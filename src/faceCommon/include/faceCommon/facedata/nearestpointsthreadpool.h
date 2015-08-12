#pragma once

#include <Poco/ThreadPool.h>
#include <opencv2/flann/flann.hpp>

#include "faceCommon/linalg/common.h"

namespace Face {
namespace FaceData {

class NearestPointsThreadPool : public Poco::ThreadPool
{
    class Thread : public Poco::Runnable
    {
        int startRow;
        int endRow;
        cv::flann::Index *index;
        const Matrix *input;
        const Matrix *pointsMat;
        Matrix *output;

    public:
        void setUp(int startRow, int endRow, const Matrix *pointsMat, const Matrix *input, cv::flann::Index *index, Matrix *output);

        void run();
    };

    std::vector<Thread> threads;

public:
    NearestPointsThreadPool();

    void getNearestPoints(const Matrix *pointsMat, const Matrix *input, cv::flann::Index *index, Matrix *output);
};

}
}
