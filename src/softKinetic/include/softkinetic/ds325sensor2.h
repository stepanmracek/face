#ifndef DS325SENSOR2_H
#define DS325SENSOR2_H

#include <DepthSense.hxx>
#include <Poco/Runnable.h>
#include <Poco/Thread.h>
#include <map>
#include <string>
#include <vector>

#include "faceSensors/isensor.h"
#include "faceCommon/objectdetection/tracker.h"
#include "softkinetic/settings.h"
#include "softkinetic/ds325sensorbase.h"
#include "softKinetic.h"

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class SOFTKINETIC_EXPORTS DS325Sensor2 : public DS325SensorBase
{
public:
    DS325Sensor2(const std::string &haarFaceDetectionPath, const std::string &landmarkDetectorPath, const Settings &settings /*= Settings()*/,  DepthSense::Context & c);
    virtual ~DS325Sensor2();

    virtual void start();

private:
    std::string currentSensor;
    std::vector<std::string> serials;
    std::map<std::string, DepthSense::Device> devices;
    std::map<std::string, DepthSense::ColorNode> colorNodes;
    std::map<std::string, DepthSense::DepthNode> depthNodes;
    std::map<std::string, Face::ObjectDetection::Tracker> trackers;

    std::map<std::string, cv::Mat_<cv::Vec3b> > colorFrames;
    std::map<std::string, cv::Mat_<uchar> > grayscaleFrames;
    std::map<std::string, cv::Mat_<uchar> > smallFrames;
    std::map<std::string, cv::Mat_<uchar> > roiFrames;

    void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

    void convertColorData(const std::string &serial, const DepthSense::ColorNode::NewSampleReceivedData &data);

    void setState(State newState);
    void doWaiting();

    Face::ObjectDetection::Tracker &getTracker() { return trackers.at(currentSensor); }
    ImageBGR &getCurrentColorFrame() { return colorFrames.at(currentSensor); }
    ImageGrayscale &getCurrentGrayscaleFrame() { return grayscaleFrames.at(currentSensor); }

    void detectFace();
    void detectFace(const std::string &sensorSerialNumber);

    void registerIfNotExists(DepthSense::Node node);
    void unregisterIfExists(DepthSense::Node node);
};

}
}
}

#endif // DS325SENSOR2_H
