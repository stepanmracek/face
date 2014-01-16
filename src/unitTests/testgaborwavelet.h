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
Matrix responseSin;
Matrix responseQbits;
Matrix realKernel;
Matrix imagKernel;

void gaborRedraw(GaborParams *gParams)
{
    realKernel = Matrix(gParams->size, gParams->size);
    imagKernel = Matrix(gParams->size, gParams->size);
    Face::LinAlg::Gabor::createWavelet(realKernel, imagKernel, gParams->frequency, gParams->orientation);

    cv::filter2D(inputImage, responseReal, -1, realKernel);
    cv::filter2D(inputImage, responseImag, -1, imagKernel);

    Matrix re2;
    cv::multiply(responseReal, responseReal, re2);
    Matrix im2;
    cv::multiply(responseImag, responseImag, im2);
    Matrix ab2 = re2 + im2;
    cv::sqrt(ab2, responseAbs);

    responseSin = Matrix(responseReal.rows, responseReal.cols);
    for (int c = 0; c < responseReal.cols; c++)
    {
        for (int r = 0; r < responseReal.rows; r++)
        {
            responseSin(r,c) = sin(responseImag(r,c)/responseAbs(r,c));
        }
    }

    responseQbits = Matrix(responseReal.rows, responseReal.cols);
    for (int c = 0; c < responseReal.cols; c++)
    {
        for (int r = 0; r < responseReal.rows; r++)
        {
            double real = responseReal(r,c);
            double imag = responseImag(r,c);
            double val = 0.0;
            if (real > 0.0 && imag > 0.0) val = 0.0;
            if (real < 0.0 && imag > 0.0) val = 0.25;
            if (real < 0.0 && imag < 0.0) val = 0.75;
            if (real > 0.0 && imag < 0.0) val = 1.0;
            responseQbits(r,c) = val;
        }
    }

    double min,max;
    cv::minMaxLoc(realKernel, &min, &max);
    realKernel = (realKernel-min)/(max-min);
    cv::imshow("real kernel", realKernel);

    cv::minMaxLoc(imagKernel, &min, &max);
    imagKernel = (imagKernel-min)/(max-min);
    cv::imshow("imag kernel", imagKernel);

    cv::minMaxLoc(responseReal, &min, &max);
    responseReal = (responseReal-min)/(max-min);
    cv::imshow("response real", responseReal);

    cv::minMaxLoc(responseImag, &min, &max);
    responseImag = (responseImag-min)/(max-min);
    cv::imshow("response imag", responseImag);

    cv::minMaxLoc(responseAbs, &min, &max);
    responseAbs = (responseAbs-min)/(max-min);
    cv::imshow("response abs", responseAbs);

    cv::minMaxLoc(responseSin, &min, &max);
    responseSin = (responseSin-min)/(max-min);
    cv::imshow("response sin", responseSin);

    cv::imshow("response qbits", responseQbits);
}

void gaborOnSizeChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->size = newVal+1;
    gaborRedraw(gparams);
}

void gaborOnFrequencyChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->frequency = newVal;
    gaborRedraw(gparams);
}

void gaborOnOrientationChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->orientation = newVal;
    gaborRedraw(gparams);
}

class TestGaborWavelet
{
public:
    static void test()
    {
        GaborParams gParams(100);
        cv::namedWindow("input image");
        cv::createTrackbar("size", "input image", &gParams.size, 200, gaborOnSizeChange, &gParams);
        cv::createTrackbar("frequency", "input image", &gParams.frequency, 10, gaborOnFrequencyChange, &gParams);
        cv::createTrackbar("orientation", "input image", &gParams.orientation, 8, gaborOnOrientationChange, &gParams);

        inputImage = Face::LinAlg::MatrixConverter::imageToMatrix("/mnt/data/frgc/spring2004/zbin-aligned/index/02463d652.png");
        cv::imshow("input image", inputImage);
        gaborRedraw(&gParams);

        char key;
        while ((key = cv::waitKey(0)) != 27)
        {
            if (key == 's')
            {
                qDebug() << "Saving";
                cv::imwrite("gabor-input.png", inputImage*255);
                cv::imwrite("gabor-realKernel.png", realKernel*255);
                cv::imwrite("gabor-imagKernel.png", imagKernel*255);
                cv::imwrite("gabor-realResponse.png", responseReal*255);
                cv::imwrite("gabor-imagResponse.png", responseImag*255);
                cv::imwrite("gabor-absResponse.png", responseAbs*255);
            }
        }
    }
};

#endif // TESTGABORWAVELET_H
