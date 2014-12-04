#include "softkinetic/ds325sensor.h"

using namespace Face::Sensors::SoftKinetic;

DS325Sensor::DS325Sensor(const std::string &haarFaceDetectionPath, const Settings &settings, DepthSense::Context & c) :
    DS325SensorBase(settings, c)
{
    ////qDebug() << "Getting devices...";
    std::vector<DepthSense::Device> devices = context.getDevices();
    if (devices.size() < 1) throw FACELIB_EXCEPTION ("no sensor was detected");


    device = devices.at(0);
    this->usedDevices.push_back(device);
    ////qDebug() << "Found device: " << device.getSerialNumber().c_str();

    ////qDebug() << "Requesting control on device...";
    context.requestControl(device);

    std::vector<DepthSense::Node> nodes = device.getNodes();
    for (DepthSense::Node &node : nodes)
    {
        if (node.is<DepthSense::ColorNode>())
        {
            colorNode = node.as<DepthSense::ColorNode>();
            configureColorNode(colorNode);
        }
        if (node.is<DepthSense::DepthNode>())
        {
            depthNode = node.as<DepthSense::DepthNode>();
            configureDepthNode(depthNode);
        }
    }

    colorFrame = cv::Mat_<cv::Vec3b>(ColorHeight, ColorWidth);
    grayscaleFrame = cv::Mat_<uchar>(ColorHeight, ColorWidth);
    smallFrame = cv::Mat_<uchar>(ColorHeight/4, ColorWidth/4);
    roiFrame = cv::Mat_<uchar>(faceDetectionRoi.height, faceDetectionRoi.width);

    tracker = Face::ObjectDetection::Tracker(haarFaceDetectionPath);
    context.startNodes();

    colorNode.setEnableColorMap(true);
    context.registerNode(colorNode);

    depthNode.setEnableDepthMapFloatingPoint(true);
    depthNode.setEnableVerticesFloatingPoint(true);
    depthNode.setEnableUvMap(true);
    context.registerNode(depthNode);
}

DS325Sensor::~DS325Sensor(){


}

void DS325Sensor::setState(IDS325Sensor::State newState)
{
    if (output.state == newState) return;
    output.state = newState;

    switch (newState)
    {
    case Off:
    	////qDebug() << "Entering Off state";
        stop();
        break;

    case Waiting:
    	////qDebug() << "Entering Waiting state";

        tracker.init();

        ////qDebug() << "registered";
        break;

    case Positioning:
    	////qDebug() << "Entering Positioning state";
        optimalDistanceCounter = 0;
        output.positioningProgress = 0;
        break;

    case Capturing:
    	////qDebug() << "Entering Capturing state";
        capturingCounter = 0;
        output.capturingProgress = 0;

        std::fill(vertexCounter.begin(), vertexCounter.end(), 0);
        std::fill(pointCloudAccumulator.begin(), pointCloudAccumulator.end(), DepthSense::FPVertex(0,0,0));

        depthNode.setEnableEparSmootherFilter(true);
        break;

    case CapturingDone:
    	////qDebug() << "Entering CapturingDone state";
        depthNode.setEnableEparSmootherFilter(false);
    }
}

void DS325Sensor::onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
    convertColorData(data);
}

void DS325Sensor::convertColorData(const DepthSense::ColorNode::NewSampleReceivedData &data)
{
    int i = 0;
    for (int y = 0; y < ColorHeight; y++) {
        for (int x = 0; x < ColorWidth; x++) {
            colorFrame(y, x)[0] = data.colorMap[3 * i];
            colorFrame(y, x)[1] = data.colorMap[3 * i + 1];
            colorFrame(y, x)[2] = data.colorMap[3 * i + 2];
            i++;
        }
    }

    cv::cvtColor(colorFrame, grayscaleFrame, CV_BGR2GRAY);
}

void DS325Sensor::doWaiting()
{
    detectFace();
    if ((tracker.getConsecutiveDetects() >= settings.consecutiveDetectsToStartPositioning) &&
        (tracker.getLastRegion().area() >= settings.minimalFaceAreaToStartPositioning))
    {
        setState(Positioning);
    }
}

void DS325Sensor::detectFace()
{
    cv::resize(grayscaleFrame, smallFrame, cv::Size(ColorWidth / 4, ColorHeight / 4));
    roiFrame = smallFrame(faceDetectionRoi);
    tracker.detect(roiFrame);
}
