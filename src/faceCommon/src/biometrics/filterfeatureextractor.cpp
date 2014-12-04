#include "faceCommon/biometrics/filterfeatureextractor.h"

using namespace Face::Biometrics;

FilterFeatureExtractor::FilterFeatureExtractor() :
    realKernel(Matrix::ones(1,1)),
    imagKernel(Matrix::ones(1,1)),
    imageScale(1.0)
{

}

FilterFeatureExtractor::FilterFeatureExtractor(const Matrix &realKernel, const Matrix &imagKernel, double imageScale) :
    realKernel(realKernel),
    imagKernel(imagKernel),
    imageScale(imageScale)
{

}

Face::LinAlg::Vector FilterFeatureExtractor::extractPhaseBinaryCode(const Matrix &inputImage) const
{
    Matrix realResponse;
    Matrix imagResponse;
    cv::filter2D(inputImage, realResponse, -1, realKernel);
    cv::filter2D(inputImage, imagResponse, -1, imagKernel);

    cv::Size newSize(inputImage.cols*imageScale, inputImage.rows*imageScale);
    cv::resize(realResponse, realResponse, newSize);
    cv::resize(imagResponse, imagResponse, newSize);

    Face::LinAlg::Vector result(newSize.width*newSize.height*2);
    int i = 0;
    for (int r = 0; r < realResponse.rows; r++)
    {
        for (int c = 0; c < realResponse.cols; c++)
        {
            result(i++) = realResponse(r, c) > 0;
            result(i++) = imagResponse(r, c) > 0;
        }
    }

    return result;
}

std::vector<Face::LinAlg::Vector> FilterFeatureExtractor::extractPhaseBinaryCode(const std::vector<Matrix> &inputImages) const
{
    std::vector<Face::LinAlg::Vector> result;
    for (const Matrix &img : inputImages)
    {
        result.push_back(extractPhaseBinaryCode(img));
    }
    return result;
}


Face::LinAlg::Vector FilterFeatureExtractor::extractAbsoluteResponse(const Matrix &inputImage) const
{
    Matrix realResponse;
    Matrix imagResponse;
    cv::filter2D(inputImage, realResponse, -1, realKernel);
    cv::filter2D(inputImage, imagResponse, -1, imagKernel);

    cv::Size newSize(inputImage.cols*imageScale, inputImage.rows*imageScale);
    cv::resize(realResponse, realResponse, newSize);
    cv::resize(imagResponse, imagResponse, newSize);

    Face::LinAlg::Vector result(newSize.width*newSize.height);
    int i = 0;
    for (int r = 0; r < realResponse.rows; r++)
    {
        for (int c = 0; c < realResponse.cols; c++)
        {
            result(i++) = sqrt(realResponse(r, c)*realResponse(r, c) + imagResponse(r, c)*imagResponse(r, c));
        }
    }

    return result;
}

std::vector<Face::LinAlg::Vector> FilterFeatureExtractor::extractAbsoluteResponse(const std::vector<Matrix> &inputImages) const
{
    std::vector<Face::LinAlg::Vector> result;
    for (const Matrix &img : inputImages)
    {
        result.push_back(extractAbsoluteResponse(img));
    }
    return result;
}
