#ifndef REALTIMETRACK_H
#define REALTIMETRACK_H

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <vector>

#include "faceCommon/linalg/common.h"
#include "faceCommon/faceCommon.h"

namespace Face {
namespace ObjectDetection {

class FACECOMMON_EXPORTS Tracker
{
    cv::CascadeClassifier classifier;
    int consecutiveDetects;
    int consecutiveNonDetects;
    bool lastDetect;
    cv::Rect initialRegion;
    cv::Rect lastRegion;
    int moveDisplacementThreshold;
    double areaDisplacementThreshold;
    bool bigDisplacement;

    cv::Point rectCenter(const cv::Rect &rect);
    cv::Rect getBiggestArea(const std::vector<cv::Rect> &regions);

public:
    Tracker(const std::string &path);
    Tracker() {}
    void init();
    cv::Rect detect(const ImageGrayscale &img);

    int getConsecutiveDetects() { return consecutiveDetects; }
    int getConsecutiveNonDetects() { return consecutiveNonDetects; }
    bool isBigDisplacement() { return bigDisplacement; }
    bool isLastDetect() { return lastDetect; }
    cv::Rect getLastRegion() { return lastRegion; }
    cv::Rect getInitialRegion() { return initialRegion; }
};

}
}

#endif // REALTIMETRACK_H
