#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <opencv2/core/core.hpp>

#include "faceCommon/linalg/common.h"

namespace Face {
namespace FaceData {

class Landmarks
{
public:
    std::vector<cv::Point3d> points;

    Landmarks() : points(std::vector<cv::Point3d>(9, cv::Point3d())) { } // 15
    Landmarks(std::vector<cv::Point3d> points) : points(points) { }
    Landmarks(const std::string &path);

    enum LandmarkNames
    {
        LeftOuterEye = 0,
        LeftInnerEye = 1,
        NasalBridge = 2,
        RightInnerEye = 3,
        RightOuterEye = 4,
        LeftOuterNose = 5,
        Nosetip = 6,
        RightOuterNose = 7,
        LowerNose = 8/*,
        LeftMouseCorner = 9,
        RighMouseCorner = 10,
        LeftOuterEyebrow = 11,
        LeftInnerEyebrow = 12,
        RightInnerEyebrow = 13,
        RightOuterEyeBrow = 14*/
    };

    void serialize(const std::string &path);

    cv::Point3d get(LandmarkNames name) { return points[name]; }

    bool is(LandmarkNames name) { return points[name] != cv::Point3d(); }

    void set(LandmarkNames name, cv::Point3d value) { points[name] = value; }

    bool check();
};

}
}

#endif // LANDMARKS_H
