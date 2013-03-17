#ifndef LANDMARKDETECTOR_H
#define LANDMARKDETECTOR_H

#include "mesh.h"
#include "landmarks.h"
#include "surfaceprocessor.h"

class LandmarkDetector
{
private:
    Mesh &mesh;
    MapConverter converter;
    Map depth;
    Map croppedDepth;
    int cropStartX;
    int cropStartY;
    CurvatureStruct curvature;
    Map peakDensity;
    Map pitDensity;

    // detector params
    int stripeWidth;
    int depthSmoothIterations;
    double depthSmoothAlpha;
    double depthScale;
    int depthErode;
    int depthLevelSelect;
    int peakDensityWindowsSize;
    int pitDensityWindowsSize;
    int pitsStripeSmoothKernel;
    int minYDistanceFromNosetipToEyes;
    int maxYDistanceFromNosetipToEyes;
    int minXDistanceFromNosetipToEyes;
    int maxXDistanceFromNosetipToEyes;

    void Nosetip(Landmarks &l);
    void InnerEyeCorners(Landmarks &l);

public:
    LandmarkDetector(Mesh &mesh);
    Landmarks Detect();
};

#endif // LANDMARKDETECTOR_H
