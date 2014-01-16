#ifndef TESTLAGUERREWAVELET_H
#define TESTLAGUERREWAVELET_H

#include <QDebug>
#include <opencv2/opencv.hpp>
#include <cmath>

#include "linalg/common.h"
#include "linalg/gausslaguerre.h"
#include "linalg/matrixconverter.h"

struct LaguerreParams
{
    LaguerreParams(int size)
    {
        this->size = size;
        n = 1;
        k = 0;
    }

    int size;
    int n;
    int k;
};

Matrix lagInputImage;
Matrix lagResponseReal;
Matrix lagResponseImag;
Matrix lagResponseAbs;
Matrix lagKernelReal;
Matrix lagKernelImag;

void laguerreRedraw(LaguerreParams *params)
{
    lagKernelReal = Matrix();
    lagKernelImag = Matrix();
    Face::LinAlg::GaussLaguerre::createWavelet(lagKernelReal, lagKernelImag, params->size, params->n, params->k);

    cv::filter2D(lagInputImage, lagResponseReal, -1, lagKernelReal);
    cv::filter2D(lagInputImage, lagResponseImag, -1, lagKernelImag);

    Matrix re2;
    cv::multiply(lagResponseReal, lagResponseReal, re2);
    Matrix im2;
    cv::multiply(lagResponseImag, lagResponseImag, im2);
    Matrix ab2 = re2 + im2;
    cv::sqrt(ab2, lagResponseAbs);

    double min,max;
    cv::minMaxLoc(lagKernelReal, &min, &max);
    lagKernelReal = (lagKernelReal-min)/(max-min);
    cv::imshow("real kernel", lagKernelReal);

    cv::minMaxLoc(lagKernelImag, &min, &max);
    lagKernelImag = (lagKernelImag-min)/(max-min);
    cv::imshow("imag kernel", lagKernelImag);

    cv::minMaxLoc(lagResponseReal, &min, &max);
    lagResponseReal = (lagResponseReal-min)/(max-min);
    cv::imshow("response real", lagResponseReal);

    cv::minMaxLoc(lagResponseImag, &min, &max);
    lagResponseImag = (lagResponseImag-min)/(max-min);
    cv::imshow("response imag", lagResponseImag);

    cv::minMaxLoc(lagResponseAbs, &min, &max);
    lagResponseAbs = (lagResponseAbs-min)/(max-min);
    cv::imshow("response abs", lagResponseAbs);
}

void laguerreOnSizeChange(int newVal, void *p)
{
    LaguerreParams *gparams = (LaguerreParams*)p;
    gparams->size = newVal+1;
    laguerreRedraw(gparams);
}

void laguerreOnNChange(int newVal, void *p)
{
    LaguerreParams *gparams = (LaguerreParams*)p;
    gparams->n = newVal;
    laguerreRedraw(gparams);
}

void laguerreOnKChange(int newVal, void *p)
{
    LaguerreParams *gparams = (LaguerreParams*)p;
    gparams->k = newVal;
    laguerreRedraw(gparams);
}

class TestLaguerreWavelet
{
public:
    static void test()
    {
        LaguerreParams params(100);
        cv::namedWindow("input image");
        cv::createTrackbar("size", "input image", &params.size, 200, laguerreOnSizeChange, &params);
        cv::createTrackbar("n", "input image", &params.n, 5, laguerreOnNChange, &params);
        cv::createTrackbar("k", "input image", &params.k, 5, laguerreOnKChange, &params);

        lagInputImage = Face::LinAlg::MatrixConverter::imageToMatrix("/mnt/data/frgc/spring2004/zbin-aligned/textureE/02463d652.png");
        cv::imshow("input image", lagInputImage);
        laguerreRedraw(&params);

        char key;
        while ((key = cv::waitKey(0)) != 27)
        {
            if (key == 's')
            {
                qDebug() << "Saving";
                cv::imwrite("gl-input.png", lagInputImage*255);
                cv::imwrite("gl-realKernel.png", lagKernelReal*255);
                cv::imwrite("gl-imagKernel.png", lagKernelImag*255);
                cv::imwrite("gl-realResponse.png", lagResponseReal*255);
                cv::imwrite("gl-imagResponse.png", lagResponseImag*255);
                cv::imwrite("gl-absResponse.png", lagResponseAbs*255);
            }
        }
    }
};

#endif // TESTLAGUERREWAVELET_H
