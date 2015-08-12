#pragma once

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/vector.h"

namespace Face {
namespace FaceData {

class FACECOMMON_EXPORTS FaceNormalization
{
    //int width;
    //int height;
    //Face::LinAlg::Vector referenceVector;
    //std::vector<cv::Point2d> referencePoints;

public:
    FaceNormalization();

    static void learn(const ImageGrayscale &input, const std::vector<cv::Point2d> &landmarks, const cv::Rect &roi, const std::string &path);

    ImageGrayscale normalize(const ImageGrayscale &input, const std::vector<cv::Point2d> &landmarks);
};

}
}
