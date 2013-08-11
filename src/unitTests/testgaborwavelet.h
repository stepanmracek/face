#ifndef TESTGABORWAVELET_H
#define TESTGABORWAVELET_H

#include <QDebug>
#include <opencv2/opencv.hpp>
#include <cmath>

#include "linalg/common.h"
#include "linalg/gabor.h"
#include "linalg/matrixconverter.h"

struct GaborParams
{
    GaborParams(int size)
    {
        this->size = size;
        frequency = 1;
        orientation = 1;
    }

    int size;
    int frequency;
    int orientation;
};

Matrix inputImage;
Matrix responseReal;
Matrix responseImag;
Matrix responseAbs;

void redraw(GaborParams *gParams)
{
    qDebug() << "redraw, size:" << gParams->size
             << "frequency:" << gParams->frequency
             << "orientation:" << gParams->orientation;
    Matrix r(gParams->size, gParams->size);
    Matrix i(gParams->size, gParams->size);
    Gabor::createWavelet(r, i, gParams->frequency, gParams->orientation);

    qDebug() << "applying filters";
    cv::filter2D(inputImage, responseReal, -1, r);
    cv::filter2D(inputImage, responseImag, -1, i);

    Matrix re2;
    cv::multiply(responseReal, responseReal, re2);
    Matrix im2;
    cv::multiply(responseImag, responseImag, im2);
    Matrix ab2 = re2 + im2;
    cv::sqrt(ab2, responseAbs);

    qDebug() << "showing kernels";
    double min,max;
    cv::minMaxLoc(r, &min, &max);
    cv::imshow("real kernel", (r-min)/(max-min));
    cv::minMaxLoc(i, &min, &max);
    cv::imshow("imag kernel", (i-min)/(max-min));

    qDebug() << "showing result";
    cv::minMaxLoc(responseAbs, &min, &max);
    cv::imshow("response real", (responseReal-min)/(max-min));
    cv::minMaxLoc(responseAbs, &min, &max);
    cv::imshow("response imag", (responseImag-min)/(max-min));
    cv::minMaxLoc(responseAbs, &min, &max);
    cv::imshow("response abs", (responseAbs-min)/(max-min));
}

void onSizeChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->size = newVal+1;
    redraw(gparams);
}

void onFrequencyChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->frequency = newVal;
    redraw(gparams);
}

void onOrientationChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->orientation = newVal;
    redraw(gparams);
}

class TestGaborWavelet
{
public:
    static void test()
    {
        GaborParams gParams(100);
        cv::namedWindow("input image");
        cv::createTrackbar("size", "input image", &gParams.size, 200, onSizeChange, &gParams);
        cv::createTrackbar("frequency", "input image", &gParams.frequency, 10, onFrequencyChange, &gParams);
        cv::createTrackbar("orientation", "input image", &gParams.orientation, 8, onOrientationChange, &gParams);

        inputImage = MatrixConverter::imageToMatrix("/mnt/data/frgc/spring2004/zbin-aligned/index2/02463d652.png");
        cv::resize(inputImage, inputImage, cv::Size(inputImage.cols/2, inputImage.rows/2));
        cv::imshow("input image", inputImage);
        redraw(&gParams);

        cv::waitKey(0);
    }
};

#endif // TESTGABORWAVELET_H
