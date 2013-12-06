#include "kernelgenerator.h"

#include <QDebug>

Matrix KernelGenerator::gaussianKernel(int size)
{
    assert(size % 2 == 1);
    assert(size >= 1);
    Matrix kernel(size, size);

    double sigma = 0.3*sqrt(size);
    double a = 1 / (sigma * sqrt(2*M_PI));
    double b = 2 * sigma * sigma;
    //double max = 0;
    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            double d = euclideanDistance(cv::Point(x,y), cv::Point(size/2,size/2));
            double v = a * exp(-d/b);

            kernel(y, x) = v;
        }
    }

    return kernel;
}

