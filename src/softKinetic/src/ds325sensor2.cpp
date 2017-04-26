#include "softkinetic/ds325sensor2.h"

#include <Poco/Thread.h>
#include "softkinetic/logger.h"

using namespace Face::Sensors::SoftKinetic;

DS325Sensor2::DS325Sensor2(const std::string &haarFaceDetectionPath, const std::string &landmarkDetectorPath, const Settings &settings,  DepthSense::Context & c) :
    DS325SensorBase(settings, landmarkDetectorPath, c)
{
	////qDebug() << "Getting devices...";
    std::vector<DepthSense::Device> devices = context.getDevices();
    if (devices.size() != 2) throw FACELIB_EXCEPTION ("2 sensors were not detected");

    for (DepthSense::Device &dev : devices)
    {
        std::string serial = dev.getSerialNumber();
        serials.push_back(serial);
        this->devices[serial] = dev;
        this->usedDevices.push_back(dev);
        ////qDebug() << "Found device: " << serial.c_str();

        ////qDebug() << "Requesting control on device...";
        context.requestControl(dev);

        std::vector<DepthSense::Node> nodes = dev.getNodes();
        for (DepthSense::Node &node : nodes)
        {
            if (node.is<DepthSense::ColorNode>())
            {
                configureColorNode(node.as<DepthSense::ColorNode>());
                colorNodes[node.getSerialNumber()] = node.as<DepthSense::ColorNode>();
            }
            if (node.is<DepthSense::DepthNode>())
            {
                configureDepthNode(node.as<DepthSense::DepthNode>());
                depthNodes[node.getSerialNumber()] = node.as<DepthSense::DepthNode>();
            }
        }

        colorFrames[serial] = cv::Mat_<cv::Vec3b>(ColorHeight, ColorWidth);
        grayscaleFrames[serial] = cv::Mat_<uchar>(ColorHeight, ColorWidth);
        smallFrames[serial] = cv::Mat_<uchar>(ColorHeight/4, ColorWidth/4);
        roiFrames[serial] = cv::Mat_<uchar>(faceDetectionRoi.height, faceDetectionRoi.width);

        trackers[serial] = Face::ObjectDetection::Tracker(new Face::ObjectDetection::OpenCVDetector(haarFaceDetectionPath));
    }

    Poco::Thread::sleep(250);

    context.startNodes();

	Poco::Thread::sleep(250);
}

DS325Sensor2::~DS325Sensor2(){

}

void DS325Sensor2::start() {
	DS325SensorBase::start();

	Poco::Thread::sleep(500);

	for (const std::string &serial : serials)
	{
		colorNodes[serial].setEnableColorMap(true);
		registerIfNotExists(colorNodes[serial]);
	}
}

void DS325Sensor2::convertColorData(const std::string &serial, const DepthSense::ColorNode::NewSampleReceivedData &data)
{
    auto &color = colorFrames[serial];
    int i = 0;
    for (int y = 0; y < ColorHeight; y++) {
        for (int x = 0; x < ColorWidth; x++) {
            color(y, x)[0] = data.colorMap[3 * i];
            color(y, x)[1] = data.colorMap[3 * i + 1];
            color(y, x)[2] = data.colorMap[3 * i + 2];
            i++;
        }
    }

    cv::cvtColor(colorFrames[serial], grayscaleFrames[serial], CV_BGR2GRAY);
}

void DS325Sensor2::onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
    std::string serial = node.getSerialNumber();
    convertColorData(serial, data);
}

void DS325Sensor2::doWaiting()
{
    bool startPositioning = false;

    // detect faces in both color frames
    for (const auto &sn : serials)
    {
        detectFace(sn);
        Face::ObjectDetection::Tracker &tracker = trackers.at(sn);

        if ((tracker.getConsecutiveDetects() >= settings.consecutiveDetectsToStartPositioning) &&
            (tracker.getLastRegion().area() >= settings.minimalFaceAreaToStartPositioning))
        {
        	////qDebug() << "Positioning may start";
            startPositioning = true;
        }
    }

    // decide which sensor use
    if (startPositioning)
    {
        if (trackers[serials[0]].getConsecutiveDetects() > settings.consecutiveDetectsToStartPositioning/2 &&
            trackers[serials[1]].getConsecutiveDetects() > settings.consecutiveDetectsToStartPositioning/2)
        {
            // choose the one with face closer to the vertical center of the frame
            const cv::Rect &firstReg = trackers.at(serials[0]).getLastRegion();
            const cv::Rect &secondReg = trackers.at(serials[1]).getLastRegion();
            int firstDistance = abs(faceDetectionRoi.height/2 - (firstReg.y + firstReg.height/2));
            int secondDistance = abs(faceDetectionRoi.height/2 - (secondReg.y + secondReg.height/2));

            if (firstDistance < secondDistance)
            {
            	////qDebug() << "choosing first sensor";
                currentSensor = serials[0];
            }
            else
            {
            	////qDebug() << "choosing second sensor";
                currentSensor = serials[1];
            }
        }
        else if (trackers[serials[0]].getConsecutiveDetects() >= settings.consecutiveDetectsToStartPositioning)
        {
            // choose first sensor
            currentSensor = serials[0];
        }
        else
        {
            // choose second sensor
            currentSensor = serials[1];
        }

        setState(STATE_POSITIONING);
    }
}

void DS325Sensor2::detectFace()
{
    detectFace(currentSensor);
}

void DS325Sensor2::detectFace(const std::string &sensorSerialNumber)
{
    cv::resize(grayscaleFrames[sensorSerialNumber], smallFrames[sensorSerialNumber], cv::Size(ColorWidth / 4, ColorHeight / 4));
    roiFrames[sensorSerialNumber] = smallFrames[sensorSerialNumber](faceDetectionRoi);
    trackers.at(sensorSerialNumber).detect(roiFrames.at(sensorSerialNumber));
}

void DS325Sensor2::registerIfNotExists(DepthSense::Node node) {
	auto regNodes = context.getRegisteredNodes();
	auto found = std::find(regNodes.begin(), regNodes.end(), node);
	if (found == regNodes.end()) {
		context.registerNode(node);
	}
}

void DS325Sensor2::unregisterIfExists(DepthSense::Node node) {
	try {
		context.unregisterNode(node);
	} catch (DepthSense::ArgumentException&) {
	}
}

void DS325Sensor2::setState(DS325Sensor2::State newState)
{
    if (output.state == newState) return;
    output.state = newState;

    switch (newState)
    {
    case STATE_OFF:
    	LOG_DEBUG("Entering Off state");

        stop();
        break;

    case STATE_IDLE:
    	LOG_DEBUG("Entering Waiting state");

        trackers.at(serials[0]).init();
        trackers.at(serials[1]).init();
        //unregisterAllNodes();

        LOG_DEBUG("registering nodes");

        resetLastDepth();

        {
            std::string firstNode = serials[0];
            depthNodes.at(firstNode).setEnableDepthMapFloatingPoint(true);
            depthNodes.at(firstNode).setEnableVerticesFloatingPoint(true);
            depthNodes.at(firstNode).setEnableUvMap(true);

        	auto regNodes = context.getRegisteredNodes();
            for(auto& dNode : depthNodes) {
            	if (dNode.first != firstNode) {
            		unregisterIfExists(dNode.second);
            	}
            }

            registerIfNotExists(depthNodes[firstNode]);
        }

        updateLastDepth();

        context.startNodes();
        {
			std::stringstream sstr;
			sstr << "registered & started, waiting: " << context.getRegisteredNodes().size();
			LOG_DEBUG(sstr.str().c_str());
        }

        break;

    case STATE_POSITIONING:
    	LOG_DEBUG("Entering Positioning state");
        optimalDistanceCounter = 0;
        output.positioningProgress = 0;

        resetLastDepth();

        {
        	auto regNodes = context.getRegisteredNodes();
            for(auto& regNode : regNodes) {
            	if (regNode.is<DepthSense::DepthNode>() && regNode.getSerialNumber() != currentSensor) {
            		context.unregisterNode(regNode);
            	}
            }
        }

        LOG_DEBUG("registering nodes: " << currentSensor.c_str());
        colorNodes.at(currentSensor).setEnableColorMap(true);
        depthNodes.at(currentSensor).setEnableDepthMapFloatingPoint(true);
        depthNodes.at(currentSensor).setEnableVerticesFloatingPoint(true);
        depthNodes.at(currentSensor).setEnableUvMap(true);

        registerIfNotExists(depthNodes[currentSensor]);

        updateLastDepth();

        context.startNodes();
        {
        	std::stringstream sstr;
            sstr << "registered & started, positioning: " << context.getRegisteredNodes().size();
            LOG_DEBUG(sstr.str().c_str());
        }

        break;

    case STATE_CAPTURING:
    	LOG_DEBUG("Entering Capturing state");
        capturingCounter = 0;
        output.capturingProgress = 0;

        std::fill(vertexCounter.begin(), vertexCounter.end(), 0);
        std::fill(pointCloudAccumulator.begin(), pointCloudAccumulator.end(), DepthSense::FPVertex(0,0,0));

        setEnableSmoothFilter(depthNodes.at(currentSensor), true);
        break;

    case STATE_CAPTURING_DONE:
    	LOG_DEBUG("Entering CapturingDone state");
        setEnableSmoothFilter(depthNodes.at(currentSensor), false);
    }
}

