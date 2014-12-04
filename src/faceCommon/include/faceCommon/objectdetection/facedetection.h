#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include <opencv2/objdetect/objdetect.hpp>

#include "faceCommon/faceCommon.h"
#include "faceCommon/linalg/common.h"

namespace Face {
namespace ObjectDetection {

class FACECOMMON_EXPORTS FaceDetection
{
public:
    struct DetectionResult
    {
        std::vector<cv::Rect> faceRegions;
        std::vector<cv::Rect> leftEyes;
        std::vector<cv::Rect> rightEyes;
        std::vector<ImageGrayscale> normalizedFaceImages;
    };

protected:
    cv::CascadeClassifier faceClassifier;
    cv::CascadeClassifier leftEyeClassifier;
    cv::CascadeClassifier rightEyeClassifier;
    int inputImageScaleFactor;

public:
    FaceDetection(const std::string &faceClassifierPath,
                  const std::string &leftEyeClassifierPath,
                  const std::string &rightEyeClassifierPath,
                  int inputImageScaleFactor = 4);

    DetectionResult detect(const ImageGrayscale &inputImg, bool normalizeFaces);
};

}
}

#endif // FACEDETECTION_H
