#include "faceCommon/linalg/differenceofgaussians.h"

#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::LinAlg;

Matrix DifferenceOfGaussians::dog(const Matrix &input, int kernel1Size, int kernel2Size, bool equalize)
{
    Matrix blurred1;
    cv::GaussianBlur(input, blurred1, cv::Size(kernel1Size, kernel1Size), 0);

    Matrix blurred2;
    cv::GaussianBlur(input, blurred2, cv::Size(kernel2Size, kernel2Size), 0);

    Matrix diff = blurred2 - blurred1;
    double min, max;
    cv::minMaxIdx(diff, &min, &max);
    diff = (diff-min)/(max-min);

    if (equalize)
    {
        Matrix equalized = MatrixConverter::equalize(diff);
        return equalized;
    }
    else
    {
        return diff;
    }
}
