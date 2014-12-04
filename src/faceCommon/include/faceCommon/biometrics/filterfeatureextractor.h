#ifndef FILTERBANKFEATUREEXTRACTOR_H
#define FILTERBANKFEATUREEXTRACTOR_H

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/vector.h"

namespace Face {
namespace Biometrics {

class FilterFeatureExtractor
{
public:
    Matrix realKernel;
    Matrix imagKernel;
    double imageScale;

    FilterFeatureExtractor();

    FilterFeatureExtractor(const Matrix &realKernel, const Matrix &imagKernel, double imageScale);

    Face::LinAlg::Vector extractPhaseBinaryCode(const Matrix &inputImage) const;

    std::vector<Face::LinAlg::Vector> extractPhaseBinaryCode(const std::vector<Matrix> &inputImages) const;

    Face::LinAlg::Vector extractAbsoluteResponse(const Matrix &inputImage) const;

    std::vector<Face::LinAlg::Vector> extractAbsoluteResponse(const std::vector<Matrix> &inputImages) const;
};

}
}

#endif // FILTERBANKFEATUREEXTRACTOR_H
