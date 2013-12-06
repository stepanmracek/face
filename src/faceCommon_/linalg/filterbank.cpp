#include "filterbank.h"

Matrix FilterBank::absResponse(const Matrix &image, const Matrix &realKernel, const Matrix &imagKernel)
{
    Matrix re;
    cv::filter2D(image, re, CV_64F, realKernel);

    Matrix im;
    cv::filter2D(image, im, CV_64F, imagKernel);

    Matrix re2;
    cv::multiply(re, re, re2);
    Matrix im2;
    cv::multiply(im, im, im2);
    Matrix ab2 = re2 + im2;
    Matrix ab;
    cv::sqrt(ab2, ab);
    return ab;
}
