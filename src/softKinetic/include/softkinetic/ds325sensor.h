#ifndef DS325SENSOR_H
#define DS325SENSOR_H

#include "softkinetic/ds325sensorbase.h"
#include "faceCommon/objectdetection/tracker.h"

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class DS325Sensor : public DS325SensorBase
{
public:
    DS325Sensor(const std::string &haarFaceDetectionPath, const Settings &settings /*= Settings()*/,  DepthSense::Context & c);
    virtual ~DS325Sensor();
private:
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
