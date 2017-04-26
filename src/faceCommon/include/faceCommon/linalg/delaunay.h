#pragma once

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
