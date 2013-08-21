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
        j = 0;
    }

    int size;
    int n;
    int k;
    int j;
};

Matrix lagInputImage;
Matrix lagResponseReal;
Matrix lagResponseImag;
Matrix lagResponseAbs;

void laguerreRedraw(LaguerreParams *params)
{
    qDebug() << "redraw";
    Matrix r(params->size, params->size);
    Matrix i(params->size, params->size);
    GaussLaguerre::createWavelet(r, i, params->n, params->k, params->j);

    qDebug() << "applying filters";
    cv::filter2D(lagInputImage, lagResponseReal, -1, r);
    cv::filter2D(lagInputImage, lagResponseImag, -1, i);

    Matrix re2;
    cv::multiply(lagResponseReal, lagResponseReal, re2);
    Matrix im2;
    cv::multiply(lagResponseImag, lagResponseImag, im2);
    Matrix ab2 = re2 + im2;
    cv::sqrt(ab2, lagResponseAbs);

    qDebug() << "showing kernels";
    double min,max;
    cv::minMaxLoc(r, &min, &max);
    cv::imshow("real kernel", (r-min)/(max-min));
    cv::minMaxLoc(i, &min, &max);
    cv::imshow("imag kernel", (i-min)/(max-min));

    qDebug() << "showing result";
    cv::minMaxLoc(lagResponseAbs, &min, &max);
    cv::imshow("response real", (lagResponseReal-min)/(max-min));
    cv::minMaxLoc(lagResponseAbs, &min, &max);
    cv::imshow("response imag", (lagResponseImag-min)/(max-min));
    cv::minMaxLoc(lagResponseAbs, &min, &max);
    cv::imshow("response abs", (lagResponseAbs-min)/(max-min));
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
        cv::createTrackbar("j", "input image", &params.j, 5, laguerreOnKChange, &params);

        lagInputImage = MatrixConverter::imageToMatrix("/mnt/data/frgc/spring2004/zbin-aligned/textureE/02463d652.png");
        cv::imshow("input image", lagInputImage);
        laguerreRedraw(&params);

        cv::waitKey(0);
    }
};

#endif // TESTLAGUERREWAVELET_H
