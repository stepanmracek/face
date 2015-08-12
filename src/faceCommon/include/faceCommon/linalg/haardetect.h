#pragma once

#include <opencv/cv.h>

#include "common.h"

namespace Face {
namespace LinAlg {

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

}
}
