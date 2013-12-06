#ifndef HAARDETECT_H
#define HAARDETECT_H

#include <opencv/cv.h>

#include "common.h"

class HaarDetect
{
private:
    cv::CascadeClassifier cascadeClassifier;

public:
    HaarDetect()
    {
        cascadeClassifier.load("foo.xml");
    }

    void detect(Matrix &img);
};

#endif // HAARDETECT_H
