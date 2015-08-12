#pragma once

#include <opencv2/opencv.hpp>

#include "common.h"
#include "vector.h"
#include "imagefilter.h"

namespace Face {
namespace LinAlg {

typedef cv::Mat_<unsigned char> SpatialHistogramMat;

class FACECOMMON_EXPORTS SpatialHistogram : public ImageFilter
{
public:
    int histLen;
    int gridSizeX;
    int gridSizeY;

    SpatialHistogram(int histLen, int gridSizeX = 8, int gridSizeY = 8);

    virtual void op(const ImageGrayscale &inputImg, SpatialHistogramMat &resultHistogram) const = 0;

    SpatialHistogramMat calculate(const ImageGrayscale &inputImg) const;
    Face::LinAlg::Vector calculateVector(const ImageGrayscale &inputImg) const;
    Matrix calculateMatrix(const ImageGrayscale &inputImg) const;
    inline Matrix process(const Matrix &input) const { return calculateMatrix(input); }
};

class FACECOMMON_EXPORTS SpatialHistogramWLD : public SpatialHistogram
{
public:

    // [32 bins for zeta][2*64 bins for theta][16 bins for center intensity]
    // since the patches are pretty small(12x12), i can even get away using uchar for the historam bins
    // all those are heuristic/empirical, i.e, i found it works better with only the 1st 2 orientations

    // configurable, yet hardcoded values
    enum {
        size_center  = 4,   // num bits from the center
        size_theta_n = 2,   // orientation channels used
        size_theta_w = 8,   // each theta orientation channel is 8*w
        size_zeta    = 32,  // bins for zeta

        size_theta = 8*size_theta_w,
        size_all = (1<<size_center) + size_zeta + size_theta_n * size_theta

        // 176 bytes per patch, * 8 * 8 = 11264 bytes per image.
    };

    void op(const ImageGrayscale &inputImg, SpatialHistogramMat &resultHistogram) const;

    SpatialHistogramWLD(int gridSizeX = 8, int gridSizeY = 8);

    static std::string name() { return "wld"; }
    std::string writeParams() const;
};

}
}
