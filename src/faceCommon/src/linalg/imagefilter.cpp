#include "faceCommon/linalg/imagefilter.h"

#include <Poco/Format.h>

#include "faceCommon/linalg/gabor.h"
#include "faceCommon/linalg/gausslaguerre.h"
#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::LinAlg;

Matrix ImageFilter::batchProcess(const Matrix &input, const std::vector<ImageFilter::Ptr> &filters)
{
    // kdyz tady volani clone() nebylo, v urcitych pripadech program padal
    // docasne, hnusne, vidlacke, ale fungujici reseni
    Matrix result = input.clone();
    for (const ImageFilter::Ptr &filter : filters)
    {
        Matrix processed = filter->process(result);
        result = processed.clone();
    }
    return result;
}

// ---------- Gabor filters ----------

GaborFilter::GaborFilter(int frequency, int orientation) : frequency(frequency), orientation(orientation)
{
    Gabor::createWavelet(realKernel, imagKernel, frequency, orientation);
}

GaborFilterReal::GaborFilterReal(int frequency, int orientation) : GaborFilter(frequency, orientation)
{

}

Matrix GaborFilterReal::process(const Matrix &input) const
{
    Matrix result;
    cv::filter2D(input, result, CV_64F, realKernel);
    return result;
}

std::string GaborFilterReal::writeParams() const
{
    return Poco::format("%s-%d-%d", name(), frequency, orientation);
}

GaborFilterImag::GaborFilterImag(int frequency, int orientation) : GaborFilter(frequency, orientation)
{

}

Matrix GaborFilterImag::process(const Matrix &input) const
{
    Matrix result;
    cv::filter2D(input, result, CV_64F, imagKernel);
    return result;
}

std::string GaborFilterImag::writeParams() const
{
    return Poco::format("%s-%d-%d", name(), frequency, orientation);
}

GaborFilterAbs::GaborFilterAbs(int frequency, int orientation) : GaborFilter(frequency, orientation)
{

}

Matrix GaborFilterAbs::process(const Matrix &input) const
{
    return Gabor::absResponse(input, realKernel, imagKernel);
}

std::string GaborFilterAbs::writeParams() const
{
    return Poco::format("%s-%d-%d", name(), frequency, orientation);
}

// ---------- G-L filters ----------

GaussLaguerreFilter::GaussLaguerreFilter(int kernelSize, int n, int k) :
    kernelSize(kernelSize), n(n), k(k)
{
    GaussLaguerre::createWavelet(realKernel, imagKernel, kernelSize, n, k);
}

GaussLaguerreFilterReal::GaussLaguerreFilterReal(int kernelSize, int n, int k) : GaussLaguerreFilter(kernelSize, n, k)
{

}

Matrix GaussLaguerreFilterReal::process(const Matrix &input) const
{
    Matrix result;
    cv::filter2D(input, result, CV_64F, realKernel);
    return result;
}

std::string GaussLaguerreFilterReal::writeParams() const
{
    return Poco::format("%s-%d-%d-%d", name(), kernelSize, n, k);
}

GaussLaguerreFilterImag::GaussLaguerreFilterImag(int kernelSize, int n, int k) : GaussLaguerreFilter(kernelSize, n, k)
{

}

Matrix GaussLaguerreFilterImag::process(const Matrix &input) const
{
    Matrix result;
    cv::filter2D(input, result, CV_64F, imagKernel);
    return result;
}

std::string GaussLaguerreFilterImag::writeParams() const
{
    return Poco::format("%s-%d-%d-%d", name(), kernelSize, n, k);
}

GaussLaguerreFilterAbs::GaussLaguerreFilterAbs(int kernelSize, int n, int k) : GaussLaguerreFilter(kernelSize, n, k)
{

}

Matrix GaussLaguerreFilterAbs::process(const Matrix &input) const
{
    return GaussLaguerre::absResponse(input, realKernel, imagKernel);
}

std::string GaussLaguerreFilterAbs::writeParams() const
{
    return Poco::format("%s-%d-%d-%d", name(), kernelSize, n, k);
}

// ---------- others filters ----------

DifferenceOfGaussiansFilter::DifferenceOfGaussiansFilter(int kernel1size, int kernel2size) :
    kernel1size(kernel1size), kernel2size(kernel2size)
{

}

Matrix DifferenceOfGaussiansFilter::process(const Matrix &input) const
{
    Matrix blurred1;
    cv::GaussianBlur(input, blurred1, cv::Size(kernel1size, kernel1size), 0);

    Matrix blurred2;
    cv::GaussianBlur(input, blurred2, cv::Size(kernel2size, kernel2size), 0);

    return (blurred2 - blurred1);
}

std::string DifferenceOfGaussiansFilter::writeParams() const
{
    return Poco::format("%s-%d-%d", name(), kernel1size, kernel2size);
}

Matrix EqualizeFilter::process(const Matrix &input) const
{
    double min, max;
    cv::minMaxIdx(input, &min, &max);
    Matrix normalized = (input-min)/(max-min);
    return MatrixConverter::equalize(normalized);
}

std::string EqualizeFilter::writeParams() const
{
    return name();
}

GaussianBlurFilter::GaussianBlurFilter(int kernelSize) : kernelSize(kernelSize)
{

}

Matrix GaussianBlurFilter::process(const Matrix &input) const
{
    Matrix result;
    cv::GaussianBlur(input, result, cv::Size(kernelSize, kernelSize), -1);
    return result;
}

std::string GaussianBlurFilter::writeParams() const
{
    return Poco::format("%s-%d", name(), kernelSize);
}

ScaleFilter::ScaleFilter(double scale) : scale(scale)
{

}

Matrix ScaleFilter::process(const Matrix &input) const
{
    return MatrixConverter::scale(input, scale);
}

std::string ScaleFilter::writeParams() const
{
    return Poco::format("%s-%f", name(), scale);
}

Matrix LBPFilter::process(const Matrix &input) const
{
    Matrix result(input.rows-2, input.cols-2);
    for (int r = 1; r < input.rows-1; r++)
    {
        for (int c = 1; c < input.cols; c++)
        {
            result (r-1,c-1) = getValue(input, r, c);
        }
    }
    return result;
}

inline double LBPFilter::getValue(const Matrix &in, int row, int col) const
{
    // 0 1 2
    // 7 c 3
    // 6 5 4
    double center = in(row, col);
    double values[] = {
        in(row-1, col-1),
        in(row-1, col  ),
        in(row-1, col+1),
        in(row  , col+1),
        in(row+1, col+1),
        in(row+1, col  ),
        in(row+1, col-1),
        in(row  , col-1)
    };

    unsigned char result = 0;
    for (int i = 0; i < 8; i++)
    {
        result |= ((values[i] >= center) << i);
    }

    return result/255.0;
}

std::string LBPFilter::writeParams() const
{
    return name();
}

GammaFilter::GammaFilter(double gamma) : gamma(gamma)
{

}

std::string GammaFilter::writeParams() const
{
    return Poco::format("%s-%d", name(), gamma);
}

Matrix GammaFilter::process(const Matrix &input) const
{
    Matrix result;
    cv::pow(input, gamma, result);
    return result;
}

HistogramFilter::HistogramFilter(int gridSizeX, int gridSizeY) :
    gridSizeX(gridSizeX), gridSizeY(gridSizeY)
{

}

std::string HistogramFilter::writeParams() const
{
    return Poco::format("%s-%d-%d", name(), gridSizeX, gridSizeY);
}

Matrix HistogramFilter::process(const Matrix &input) const
{
    int width = input.cols/gridSizeX;
    int height = input.rows/gridSizeY;

    std::vector<double> resultVec;
    for(int i = 0; i < gridSizeY; i++)
    {
        for(int j = 0; j < gridSizeX; j++)
        {
            std::vector<double> h(256);
            double increment = 1.0/(width*height);
            for (int r = i*height; r < (i+1)*height; r++)
            {
                for (int c = j*width; c < (j+1)*width; c++)
                {
                    //int bin = input(r,c)*255;
                    int bin = (int)round(input(r,c)*255.0);
                    h[bin] += increment;
                }
            }
            resultVec.insert(resultVec.end(), h.begin(), h.end());
        }
    }
    return Vector(resultVec);
}

std::vector<double> HistogramFilter::histogram(const Matrix &cell) const
{
    std::vector<double> h(256);
    double increment = 1.0/(cell.rows*cell.cols);
    for (int r = 0; r < cell.rows; r++)
    {
        for (int c = 0; c < cell.cols; c++)
        {
            //int bin = cell(r,c)*255;
            int bin = (int)round(cell(r,c)*255.0);
            h[bin] += increment;
        }
    }

    return h;
}

Matrix ContrastEqualizationFilter::process(const Matrix &input) const
{
    Matrix tmp; input.copyTo(tmp);

    Matrix tmp1;
    cv::pow(cv::abs(tmp), alpha, tmp1);
    double meanI = cv::mean(tmp1)[0];
    tmp = tmp / cv::pow(meanI, 1.0 / alpha);

    cv::pow(cv::min(cv::abs(tmp1), tau), alpha, tmp1);
    meanI = cv::mean(tmp1)[0];
    tmp = tmp / cv::pow(meanI, 1.0 / alpha);

    for (int r = 0; r < tmp.rows; r++)
    {
        for (int c = 0; c < tmp.cols; c++)
        {
            tmp(r, c) = tanh(tmp(r,c) / tau);
        }
    }
    tmp = (tau * tmp)+tau;
    cv::normalize(tmp, tmp, 0.0, 1.0, CV_MINMAX);
    return tmp;
}
