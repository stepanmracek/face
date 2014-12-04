#ifndef DELAUNAY_H
#define DELAUNAY_H

#include <opencv2/opencv.hpp>

#include "common.h"

namespace Face {
namespace LinAlg {

class Delaunay
{
public:
    static std::vector<cv::Vec3i> process(std::vector<cv::Point2d> &points);
};

}
}

#endif // DELAUNAY_H
