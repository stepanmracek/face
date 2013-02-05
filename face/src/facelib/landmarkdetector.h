#ifndef LANDMARKDETECTOR_H
#define LANDMARKDETECTOR_H

#include "mesh.h"
#include "landmarks.h"
#include "surfaceprocessor.h"
#include "linalg/vector3.h"

class LandmarkDetector
{
public:
    static cv::Point3d Nosetip(Mesh &f);
    static Landmarks Detect(Mesh &f);
};

#endif // LANDMARKDETECTOR_H
