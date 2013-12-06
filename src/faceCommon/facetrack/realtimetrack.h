#ifndef REALTIMETRACK_H
#define REALTIMETRACK_H

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <vector>

#include "linalg/common.h"

class RealTimeTracker
{
    cv::CascadeClassifier classifier;
public:
    RealTimeTracker(const QString &path);

    std::vector<cv::Rect>  detect(ImageGrayscale &img);

    //int trackTest();
};

#endif // REALTIMETRACK_H
