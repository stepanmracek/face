#pragma once

#include <opencv2/core/core.hpp>

#include "faceCommon/linalg/common.h"

namespace Face {
namespace FaceData {

class FACECOMMON_EXPORTS Landmarks
{
public:
	typedef cv::Point3d Point;
	typedef std::vector<Point> Points;
    Points points;

	Landmarks() : points(Points(21, Point())) { }
    Landmarks(std::vector<Point> points) : points(points) { }
    Landmarks(const std::string &path);
	
    void serialize(const std::string &path) const;
    void translate(cv::Point3d shift);
    void transform(Matrix &m);
};

}
}
