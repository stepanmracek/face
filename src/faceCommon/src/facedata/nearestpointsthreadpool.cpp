#include "faceCommon/facedata/nearestpointsthreadpool.h"

#include <Poco/Environment.h>

using namespace Face::FaceData;

void NearestPointsThreadPool::Thread::setUp(int startRow, int endRow, const Matrix *pointsMat,
                                            const Matrix *input, cv::flann::Index *index, Matrix *output)
{
    this->startRow = startRow;
    this->endRow = endRow;
    this->index = index;
    this->input = input;
    this->pointsMat = pointsMat;
    this->output = output;
}

void NearestPointsThreadPool::Thread::run()
{
    //std::cout << "in thread " << startRow << ".." << endRow << "; " << pointsMat->rows << " " << output->rows << " " << input->rows << std::endl;
    for (int r = startRow; r < endRow; r++)
    {
        cv::Mat query;
        input->row(r).convertTo(query, CV_32F);

        std::vector<int> resultIndicies;
        std::vector<float> resultDistances;
        index->knnSearch(query, resultIndicies, resultDistances, 1);

        int pIndex = resultIndicies[0];
        (*output)(r, 0) = (*pointsMat)(pIndex, 0);
        (*output)(r, 1) = (*pointsMat)(pIndex, 1);
        (*output)(r, 2) = (*pointsMat)(pIndex, 2);
    }
}

NearestPointsThreadPool::NearestPointsThreadPool()
{
    threads = std::vector<NearestPointsThreadPool::Thread>(Poco::Environment::processorCount());
}

void NearestPointsThreadPool::getNearestPoints(const Matrix *pointsMat, const Matrix *input, cv::flann::Index *index, Matrix *output)
{
    for (unsigned int i = 0; i < threads.size(); i++)
    {
        int startRow = i * input->rows / threads.size();
        int endRow = (i+1) * input->rows / threads.size();

        //std::cout << i << " " << pointsMat->rows << " startRow: " << startRow << " endRow: " << endRow << std::endl;
        threads[i].setUp(startRow, endRow, pointsMat, input, index, output);
        start(threads[i], "align-"+std::to_string(i));
    }
    joinAll();
}
