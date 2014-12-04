#ifndef DS325SENSORBASE_H
#define DS325SENSORBASE_H

#include <DepthSense.hxx>

#include "softkinetic/ds325sensorfactory.h"
#include "softkinetic/settings.h"
#include "faceCommon/objectdetection/tracker.h"
#include <Poco/Timestamp.h>
#include <Poco/Nullable.h>

namespace Face {
namespace Sensors {
namespace SoftKinetic {

class DS325SensorBase : public IDS325Sensor, protected Poco::Runnable
{
public:
    DS325SensorBase(const Settings &settings, DepthSense::Context & c);

    void doLoop();

    virtual ~DS325SensorBase();

    void scan();

protected:
    const static int ColorWidth = 640;
    const static int ColorHeight = 480;
    const static int DepthWidth = 320;
    const static int DepthHeight = 240;
    const static int DesiredConfidence = 200;

    Settings settings;
    DepthSense::Context & context;
    cv::Rect faceDetectionRoi;

    std::vector<float> depthMap;
    std::vector<DepthSense::FPVertex> pointCloud;
    std::vector<DepthSense::UV> uvMap;
    std::vector<int> vertexCounter;
    std::vector<DepthSense::FPVertex> pointCloudAccumulator;

    int optimalDistanceCounter;
    int capturingCounter;

    void configureDepthNode(DepthSense::DepthNode node);
    void configureColorNode(DepthSense::ColorNode node);
    void unregisterAllNodes();
    void reconnectAllNodes();
    double getDistance();

    virtual void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data) = 0;
    void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

    virtual void setState(State newState) = 0;
    virtual void doWaiting() = 0;
    void doPositioning();
    void doCapturing();
    void doCapturingDone();

    void start();
    virtual void stop();

    virtual Face::ObjectDetection::Tracker &getTracker() = 0;
    virtual void detectFace() = 0;
    virtual ImageGrayscale &getCurrentGrayscaleFrame() = 0;
    virtual ImageBGR &getCurrentColorFrame() = 0;

    cv::Rect getFullFaceRegion();

    void run();

    void updateLastDepth();
    void resetLastDepth();
protected:
    Poco::Nullable<Poco::Timestamp> lastDepth;
    Poco::Thread contextThread;
    std::vector < DepthSense::Device> usedDevices;
};

}
}
}

#endif // DS325SENSORBASE_H
