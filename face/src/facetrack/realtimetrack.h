#ifndef REALTIMETRACK_H
#define REALTIMETRACK_H

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <vector>

#include "linalg/common.h"

class RealTimeTrack
{
    cv::CascadeClassifier faceDetect;
    cv::CascadeClassifier eyeDetect;
    cv::CascadeClassifier noseDetect;

public:
    RealTimeTrack();

    std::vector<cv::Rect>  trackFace(ImageGrayscale &img);

    int trackTest();
};

#endif // REALTIMETRACK_H
