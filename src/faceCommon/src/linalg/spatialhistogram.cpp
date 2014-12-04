#include "faceCommon/linalg/spatialhistogram.h"

#include <Poco/String.h>

using namespace Face::LinAlg;

SpatialHistogram::SpatialHistogram(int histLen, int gridSizeX, int gridSizeY) :
    histLen(histLen), gridSizeX(gridSizeX), gridSizeY(gridSizeY)
{

}

SpatialHistogramMat SpatialHistogram::calculate(const ImageGrayscale &inputImg) const
{
    int width = inputImg.cols/gridSizeX;
    int height = inputImg.rows/gridSizeY;

    SpatialHistogramMat result = SpatialHistogramMat::zeros(0, 0);

    for(int i = 0; i < gridSizeY; i++) {
        for(int j = 0; j < gridSizeX; j++) {
            ImageGrayscale cell(inputImg, cv::Range(i*height,(i+1)*height), cv::Range(j*width,(j+1)*width));
            SpatialHistogramMat hist = SpatialHistogramMat::zeros(1, histLen);

            op(cell, hist);
            result.push_back(hist);
        }
    }

    return result;
}

Vector SpatialHistogram::calculateVector(const ImageGrayscale &inputImg) const
{
    SpatialHistogramMat mat = calculate(inputImg);

    Vector result(mat.rows*mat.cols);
    int i = 0;
    for (int r = 0; r < mat.rows; r++)
    {
        for (int c = 0; c < mat.cols; c++)
        {
            result(i++) = mat(r, c);
        }
    }

    return result;
}

Matrix SpatialHistogram::calculateMatrix(const ImageGrayscale &inputImg) const
{
    SpatialHistogramMat mat = calculate(inputImg);

    Matrix result(mat.rows, mat.cols);
    for (int r = 0; r < mat.rows; r++)
    {
        for (int c = 0; c < mat.cols; c++)
        {
            result(r, c) = mat(r, c);
        }
    }

    return result;
}

SpatialHistogramWLD::SpatialHistogramWLD(int gridSizeX, int gridSizeY) :
    SpatialHistogram(size_all, gridSizeX, gridSizeY)
{

}

void SpatialHistogramWLD::op(const ImageGrayscale &inputImg, SpatialHistogramMat &resultHistogram) const
{
    int radius = 1;
    for(int i = radius; i < inputImg.rows-radius; i++) {
        for(int j = radius ; j < inputImg.cols-radius; j++) {
            // 7 0 1
            // 6 c 2
            // 5 4 3
            uchar c   = inputImg(i, j);
            uchar n[8]= {
                inputImg(i-1, j  ),
                inputImg(i-1, j+1),
                inputImg(i  , j+1),
                inputImg(i+1, j+1),
                inputImg(i+1, j  ),
                inputImg(i+1, j-1),
                inputImg(i,   j-1),
                inputImg(i-1, j-1)
            };

            int p = n[0]+n[1]+n[2]+n[3]+n[4]+n[5]+n[6]+n[7];
            p -= c*8;
            // (7), projected from [-pi/2,pi/2] to [0,size_zeta]
            double zeta = 0;
            if (p!=0) zeta = double(size_zeta) * (atan(double(p)/c) + M_PI*0.5) / M_PI;
            resultHistogram(int(round(zeta))) += 1;

            // (11), projected from [-pi/2,pi/2] to [0,size_theta]
            for ( int i=0; i<size_theta_n; i++ ) {
                double a = atan2(double(n[i]-n[(i+4)%8]), double(n[(i+2)%8]-n[(i+6)%8]));
                double theta = M_PI_4 * fmod( (a+M_PI)/M_PI_4+0.5f, 8) * size_theta_w; // (11)
                resultHistogram(int(round(theta))+size_zeta+size_theta * i) += 1;
            }

            // additionally, add some bits of the actual center value (MSB).
            int cen = c >> (8 - size_center);
            resultHistogram(cen + size_zeta + size_theta * size_theta_n) += 1;
        }
    }
}

std::string SpatialHistogramWLD::writeParams() const
{
    return name() + "-" + std::to_string(gridSizeX) + "-" + std::to_string(gridSizeY);
}
