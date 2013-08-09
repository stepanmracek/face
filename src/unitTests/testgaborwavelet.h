#ifndef TESTGABORWAVELET_H
#define TESTGABORWAVELET_H

#include <QDebug>

#include <opencv2/opencv.hpp>
#include <cmath>

struct GaborParams
{
    GaborParams(int size)
    {
        this->size = size;
        waveSize = 0;
        orientation = 0;
    }

    int size;
    int waveSize;
    int orientation;
};

cv::Mat gabor(int size, int waveSize, int orientation)
{
    //CvGabor cvGabor(waveSize, orientation);
    //return cvGabor.Real.clone();

    int matSize = (size*2 + 1);
    Matrix result = Matrix::zeros(matSize, matSize);
    int centralIndex = matSize/2;

    double kmax = M_PI/2.0;
    double f = 2;
    double sigma = 2*M_PI;///size;

    double phi = orientation * M_PI / 8.0;
    double k = kmax/pow(f,(waveSize+2.0)/2.0);
    double k1 = k * cos(phi);
    double k2 = k * sin(phi);

    double kMag = k1*k1+k2*k2;
    double coef = kMag/(sigma*sigma);
    double coef2 = exp( - (sigma*sigma)/2 );

    for (int r = 0; r < matSize; r++)
    {
        int y = r - centralIndex;
        for (int c = 0; c < matSize; c++)
        {
            int x = c-centralIndex;

            double expPart = coef * exp(- (kMag*(x*x+y*y))/(2*sigma*sigma) );
            double cosPart = cos(k1*x + k2*y) - coef2;
            double val = expPart*cosPart;
            result(r, c) = val;
        }
    }
    return result;
}

cv::Mat inputImage;
cv::Mat outputImage;

void redraw(GaborParams *gParams)
{
    cv::Mat gaborWavelet = gabor(gParams->size, gParams->waveSize, gParams->orientation);

    cv::filter2D(inputImage, outputImage, -1, gaborWavelet);

    double min,max;
    cv::minMaxLoc(gaborWavelet, &min, &max);
    cv::imshow("wavelet generator", (gaborWavelet-min)/(max-min));

    cv::minMaxLoc(outputImage, &min, &max);
    cv::imshow("convolution output", outputImage);// (outputImage-min)/(max-min));
}

void onSizeChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->size = newVal;
    redraw(gparams);
}

void onWaveSizeChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->waveSize = newVal;
    redraw(gparams);
}

void onOrientationChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->orientation = newVal;
    redraw(gparams);
}

/*void onLambdaChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->lambda = newVal/100.0+0.5;
    redraw(gparams);
}
void onThetaChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->theta = (newVal-5.0)/10.0;
    redraw(gparams);
}
void onPsiChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->psi = (newVal-5.0)/10.0;
    redraw(gparams);
}
void onSigmaChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->sigma = newVal/100.0;
    redraw(gparams);
}
void onGammaChange(int newVal, void *p)
{
    GaborParams *gparams = (GaborParams*)p;
    gparams->gamma = (newVal-5.0)/10.0;
    redraw(gparams);
}*/

class TestGaborWavelet
{
public:
    static void test()
    {
        GaborParams gParams(20);
        cv::namedWindow("wavelet generator");
        cv::createTrackbar("size", "wavelet generator", &gParams.size, 100, onSizeChange, &gParams);
        cv::createTrackbar("wave size", "wavelet generator", &gParams.waveSize, 5, onWaveSizeChange, &gParams);
        cv::createTrackbar("orientation", "wavelet generator", &gParams.orientation, 7, onOrientationChange, &gParams);
        /*cv::createTrackbar("lambda", "wavelet generator", &gParams.lamdaSlider, 99, onLambdaChange, &gParams);
        cv::createTrackbar("theta", "wavelet generator", &gParams.thetaSlider, 10, onThetaChange, &gParams);
        cv::createTrackbar("psi", "wavelet generator", &gParams.psiSlider, 10, onPsiChange, &gParams);
        cv::createTrackbar("sigma", "wavelet generator", &gParams.sigmaSlider, 99, onSigmaChange, &gParams);
        cv::createTrackbar("gamma", "wavelet generator", &gParams.gammaSlider, 10, onGammaChange, &gParams);*/

        cv::namedWindow("input image");
        cv::namedWindow("convolution output");
        inputImage = cv::imread("/mnt/data/frgc/spring2004/zbin-aligned/index2/02463d652.png");
        cv::imshow("input image", inputImage);
        redraw(&gParams);

        cv::waitKey(0);
    }
};

#endif // TESTGABORWAVELET_H
