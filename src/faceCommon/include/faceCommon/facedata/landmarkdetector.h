#pragma once

//#include "mesh.h"
//#include "landmarks.h"
//#include "surfaceprocessor.h"
#include "faceCommon/linalg/common.h"
#include "faceCommon/settings/settings.h"

namespace dlib {
class shape_predictor;
}

namespace Face {
namespace FaceData {

class FACECOMMON_EXPORTS LandmarkDetector
{
private:
    cv::Ptr<dlib::shape_predictor> shapePredictor;

public:
	typedef std::vector<cv::Point2d> Landmarks2D;

    LandmarkDetector(const std::string &shapePredictorPath = Face::Settings::instance().settingsMap[Face::Settings::ShapePredictorPathKey]);
    Landmarks2D get(ImageGrayscale &img, cv::Rect &roi);
};

/*class FACECOMMON_EXPORTS LandmarkDetector
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
    //Map pitDensity;

    // detector params
    int stripeWidth;
    int depthGaussSize;
    int depthGaussIterations;
    double depthScale;
    int depthErode;
    int depthLevelSelect;
    int peakDensityWindowsSize;
    int pitsStripeSmoothKernel;
    int minYDistanceFromNosetipToEyes;
    int maxYDistanceFromNosetipToEyes;
    int minXDistanceFromNosetipToEyes;
    int maxXDistanceFromNosetipToEyes;

    void nosetip(Landmarks &l);
    void innerEyeCorners(Landmarks &l);

public:
    LandmarkDetector(Mesh &mesh);
    Landmarks detect();
};*/

}
}
