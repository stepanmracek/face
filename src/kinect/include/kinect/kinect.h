#ifndef KINECT_H_
#define KINECT_H_

#include "faceCommon/linalg/common.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/map.h"

namespace Face {
namespace Sensors {
namespace Kinect {

class Kinect
{
private:
    static const int n = 640*480;

    //static const double KINECT_alpha = 0.00307110156374;
    //static const double KINECT_beta = 3.33094951605675;
    static constexpr double mindistance = -10;
    static constexpr double scaleFactor = 0.0021;

public:
    static bool getDepth(double *depth, bool *mask = NULL, double minDistance = 200, double maxDistance = 2500);

    static bool getDepth(double *depth, int scansCount, bool *mask = NULL, double minDistance = 200, double maxDistance = 2500);

    static bool getRGB(unsigned char *rgb);

    static bool getRGBIter(unsigned char *rgb, int scansCount);

    static Face::FaceData::VectorOfPoints depthToVectorOfPoints(double *depth);

    static Matrix depthToMatrix(double *depth, double nullValue = 0.0);
    static void depthToMatrix(double *depth, Matrix &out, double nullValue = 0.0);

    static Face::FaceData::Mesh *createMesh(double *depth, unsigned char *rgb);

    static ImageBGR RGBToColorMatrix(unsigned char *rgb);

    static ImageGrayscale RGBToGrayscale(unsigned char *rgb);
    static void RGBToGrayscale(unsigned char *rgb, ImageGrayscale &out);

    static Face::FaceData::Mesh *scanFace(int scanIterations, const std::string &faceHaarPath);

    static Face::FaceData::Mesh *scanAndAlignFace(int scanIterations, int icpIterations, const std::string &alignReferenceOBJPath, const std::string &faceHaarPath);

    static bool isKinectPluggedIn(double *depthBuffer);

    static void stop();

    enum LEDState {
        LED_OFF              = 0, /**< Turn LED off */
        LED_GREEN            = 1, /**< Turn LED to Green */
        LED_RED              = 2, /**< Turn LED to Red */
        LED_YELLOW           = 3, /**< Turn LED to Yellow */
        LED_BLINK_GREEN      = 4, /**< Make LED blink Green */

        LED_BLINK_RED_YELLOW = 6, /**< Make LED blink Red/Yellow */
    };

    static void setLEDs(LEDState);
};

}
}
}

#endif /* KINECT_H_ */
