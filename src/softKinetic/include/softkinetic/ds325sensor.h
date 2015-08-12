#ifndef DS325SENSOR_H
#define DS325SENSOR_H

#include "softkinetic/ds325sensorbase.h"
#include "faceCommon/objectdetection/tracker.h"
#include "faceCommon/settings/settings.h"
#include "softKinetic.h"

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class SOFTKINETIC_EXPORTS DS325Sensor : public DS325SensorBase
{
public:
    DS325Sensor(const std::string &haarFaceDetectionPath, const std::string &landmarkDetectorPath, const Settings &settings,  DepthSense::Context & c);
    DS325Sensor(const std::string &haarFaceDetectionPath = Face::Settings::instance().settingsMap[Face::Settings::CascadeFaceDetectorPathKey],
                const std::string &landmarkDetectorPath = Face::Settings::instance().settingsMap[Face::Settings::ShapePredictorPathKey],
                const Settings &settings = Settings());

    virtual ~DS325Sensor();
private:
    void init(const std::string &haarFaceDetectionPath);

    DepthSense::Device device;
    DepthSense::ColorNode colorNode;
    DepthSense::DepthNode depthNode;
    Face::ObjectDetection::Tracker tracker;

    cv::Mat_<cv::Vec3b> colorFrame;
    cv::Mat_<uchar> grayscaleFrame;
    cv::Mat_<uchar> smallFrame;
    cv::Mat_<uchar> roiFrame;

    void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

    void setState(State newState);
    void doWaiting();

    Face::ObjectDetection::Tracker &getTracker() { return tracker; }
    ImageBGR &getCurrentColorFrame() { return colorFrame; }
    ImageGrayscale &getCurrentGrayscaleFrame() { return grayscaleFrame; }

    void convertColorData(const DepthSense::ColorNode::NewSampleReceivedData &data);
    void detectFace();
};

}
}
}

#endif // DS325SENSOR_H
