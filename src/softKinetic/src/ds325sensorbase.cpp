#include "softkinetic/ds325sensorbase.h"

#include "softkinetic/guidrawer.h"
#include "softkinetic/logger.h"
#include <Poco/Exception.h>

using namespace Face::Sensors::SoftKinetic;

DS325SensorBase::DS325SensorBase(const Settings &settings, DepthSense::Context & c) :
    faceDetectionRoi(25, 0, 110, 120),
    settings(settings),
    context(c),
    contextThread("ds325.bg")
{
    depthMap.resize(DepthWidth*DepthHeight);
    pointCloud.resize(DepthWidth*DepthHeight);
    uvMap.resize(DepthWidth*DepthHeight);
    vertexCounter.resize(DepthWidth*DepthHeight);
    pointCloudAccumulator.resize(DepthWidth*DepthHeight);

    output.currentFrame = cv::Mat_<uchar>::zeros(ColorHeight, ColorWidth);

    LOG_DEBUG("Creating context...");
    //context = c;
}

void DS325SensorBase::doLoop()
{
	if (output.state == Waiting || output.state == Positioning || output.state == Capturing){
        if (!lastDepth.isNull() && lastDepth.value().elapsed() / 1000 > 500){
			 LOG_ERROR("No depth buffer data in 500ms");
			 throw Poco::Exception("No depth data");
			 //this->start();
		}
	}
	output.previousState = output.state;
    switch (output.state)
    {
    case Waiting:
        doWaiting();
        break;
    case Positioning:
        doPositioning();
        break;
    case Capturing:
        doCapturing();
        break;
    case CapturingDone:
        doCapturingDone();
    }
}

void DS325SensorBase::run()
{
    try
    {
        LOG_DEBUG("running context loop...");
		output.stopGesture = false;
		context.run();
		LOG_DEBUG("running context finished...");
    }
    catch (std::exception & e)
    {
    	LOG_ERROR("run std exception: " << e.what());
	}
}

void DS325SensorBase::updateLastDepth() {
    if (!lastDepth.isNull()) {
        lastDepth = Poco::Timestamp();
	}
    lastDepth = Poco::Timestamp();
}

void DS325SensorBase::resetLastDepth() {
    lastDepth = Poco::Timestamp();
}


DS325SensorBase::~DS325SensorBase()
{
	try {
		unregisterAllNodes();
		this->stop();
	} catch (...){

	}
	try {
		context.stopNodes();
	} catch (...){

	}

	for (auto &dev : usedDevices)
	{
		try {

			std::vector<DepthSense::Node> nodes = dev.getNodes();
			for (DepthSense::Node &node : nodes)
			{
				if (node.is<DepthSense::ColorNode>())
				{
					node.as<DepthSense::ColorNode>().newSampleReceivedEvent().disconnect(this, &DS325SensorBase::onNewColorSample);

				}
				if (node.is<DepthSense::DepthNode>())
				{
					node.as<DepthSense::DepthNode>().newSampleReceivedEvent().disconnect(this, &DS325SensorBase::onNewDepthSample);
				}
			}

			context.releaseControl(dev);
		} catch (...){

		}
	}

}

void DS325SensorBase::start()
{
    if (!lastDepth.isNull()) {
        lastDepth = Poco::Timestamp();
	}
	LOG_DEBUG("start called");
	if (contextThread.isRunning()) return;
	LOG_DEBUG("start called real");
	contextThread.start(*this);
    setState(Waiting);

    LOG_DEBUG("start called done");
}

void DS325SensorBase::stop()
{
	updateLastDepth();
	LOG_DEBUG("stop called");
	if (!contextThread.isRunning()) return;
	// TODO commented out because of one-camera sensor
    //unregisterAllNodes();
    LOG_DEBUG("stop called real");
    context.quit();
    contextThread.join();
    output.state = Off;

    LOG_DEBUG("stop called done");
}

void DS325SensorBase::scan()
{
    LOG_DEBUG("DS325Sensor2::scan()");
    start();

    while (output.state != Off)
    {
        if (cv::waitKey(output.state == Waiting ? 100 : 30) > 0 || output.stopGesture)
        {
            stop();
        }

        doLoop();
        GUIDrawer::draw(getOutput());
    }
    LOG_DEBUG("DS325Sensor2::scan() ended");
}

void DS325SensorBase::configureColorNode(DepthSense::ColorNode node)
{
    node.newSampleReceivedEvent().connect(this, &DS325SensorBase::onNewColorSample);

    DepthSense::ColorNode::Configuration config = node.getConfiguration();
    config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
    config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
    config.framerate = 25;

    node.setConfiguration(config);
}

void DS325SensorBase::configureDepthNode(DepthSense::DepthNode node)
{
    node.newSampleReceivedEvent().connect(this, &DS325SensorBase::onNewDepthSample);

    DepthSense::DepthNode::Configuration config = node.getConfiguration();
    config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
    config.framerate = 25;
    config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    node.setEnableEparSmootherFilter(false);
    node.setConfidenceThreshold(DesiredConfidence);
    node.setConfiguration(config);
}

void DS325SensorBase::onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
	updateLastDepth();
	LOG_TRACE("new depth sample");

	if (data.droppedSampleCount > 0) {
		std::stringstream sstr;
		sstr << "Samples dropped: " << data.droppedSampleCount;
		LOG_DEBUG(sstr.str().c_str());
	}

    int saturatedCounter = 0;
    int n = DepthWidth*DepthHeight;
    for (int i = 0; i < n; i++)
    {
        depthMap[i] = data.depthMapFloatingPoint[i];
        pointCloud[i] = data.verticesFloatingPoint[i];
        uvMap[i] = data.uvMap[i];

        if (depthMap[i] == -2) saturatedCounter++;
    }

    int saturatedPercent = saturatedCounter*100/n;
    output.stopGesture = (saturatedPercent > 50);
}

void DS325SensorBase::unregisterAllNodes()
{
    auto registeredNodes = context.getRegisteredNodes();
    std::stringstream sstr;
    sstr << "unregistering all nodes: " << registeredNodes.size();
    LOG_DEBUG(sstr.str().c_str());
    for (auto &n : registeredNodes)
    {
    	LOG_DEBUG((std::string("unregistering ") + n.getType().name()).c_str());
        context.unregisterNode(n);
    }
}

void DS325SensorBase::reconnectAllNodes() {
	for (auto &dev : usedDevices)
	{
		std::vector<DepthSense::Node> nodes = dev.getNodes();
		for (DepthSense::Node &node : nodes)
		{
			if (node.is<DepthSense::ColorNode>())
			{
				node.as<DepthSense::ColorNode>().newSampleReceivedEvent().disconnect(this, &DS325SensorBase::onNewColorSample);
				node.as<DepthSense::ColorNode>().newSampleReceivedEvent().connect(this, &DS325SensorBase::onNewColorSample);
			}
			if (node.is<DepthSense::DepthNode>())
			{
				node.as<DepthSense::DepthNode>().newSampleReceivedEvent().disconnect(this, &DS325SensorBase::onNewDepthSample);
				node.as<DepthSense::DepthNode>().newSampleReceivedEvent().connect(this, &DS325SensorBase::onNewDepthSample);
			}
		}
	}
}

cv::Rect DS325SensorBase::getFullFaceRegion()
{
    Face::ObjectDetection::Tracker &tracker = getTracker();
    if (!tracker.isLastDetect())
    {
        return cv::Rect();
    }
    else
    {
        const cv::Rect &r = tracker.getLastRegion();
        return cv::Rect((faceDetectionRoi.x + r.x) * 4, (faceDetectionRoi.y + r.y) * 4, r.width * 4, r.height * 4);
    }
}

double DS325SensorBase::getDistance()
{
    float sum = 0.0;
    int count = 0;
    int depthBufferSize = DepthWidth * DepthHeight;
    cv::Rect faceRegion = getFullFaceRegion();
    cv::Point faceCenter = cv::Point(faceRegion.x + faceRegion.width/2, faceRegion.y + faceRegion.height/2);

    for (int i = 0; i < depthBufferSize; i++)
    {
        const DepthSense::UV & uv = uvMap[i];
        if (uv.u == -FLT_MAX || uv.v == -FLT_MAX || depthMap[i] == -2.0)
        {
            continue;
        }
        int x = uv.u * ColorWidth;
        int y = uv.v * ColorHeight;

        if (!faceRegion.contains(cv::Point(x, y))) continue;

        float depthValue = depthMap[i];
        float dx = faceCenter.x - x;
        float dy = faceCenter.y - y;
        double distanceFromCenter = sqrt(dx*dx + dy*dy);

        if (distanceFromCenter < 0.4f*faceRegion.width) {
            sum += depthValue;
            count++;
            //test.at<unsigned char>(y, x) = 127;
        }
    }

    return sum / count;
}

void DS325SensorBase::doPositioning()
{
	output.capturingProgress = 0;
	output.positioningProgress = 0;

    detectFace();
    Face::ObjectDetection::Tracker &tracker = getTracker();

    if (tracker.getConsecutiveNonDetects() == settings.consecutiveNonDetectsToLeavePositioning)
    {
        setState(Waiting);
        return;
    }

    if (tracker.isBigDisplacement())
    {
        optimalDistanceCounter = 0;
    }

    double distance = getDistance();

    //std::cout << "distance: " << distance << std::endl;

    output.faceRegion = cv::Rect();
    if (distance != distance) output.positioningOutput = NoData;
    else if (distance > settings.maximalDistance) output.positioningOutput = MoveCloser;
    else if (distance < settings.minimalDistance) output.positioningOutput = MoveFar;
    // else if (...) TODO: tilt;
    else
    {
        output.positioningOutput = DontMove;
        optimalDistanceCounter++;
        output.positioningProgress = optimalDistanceCounter*100/settings.consecutiveOptimalDistanceToStartCapturing;
        output.faceRegion = getFullFaceRegion();
    }

    if (optimalDistanceCounter == settings.consecutiveOptimalDistanceToStartCapturing)
    {
        setState(Capturing);
    }

    getCurrentGrayscaleFrame().copyTo(output.currentFrame);
}

void DS325SensorBase::doCapturing()
{
    int depthBufferSize = DepthWidth * DepthHeight;
    for (int i = 0; i < depthBufferSize; i++)
    {
        const DepthSense::FPVertex &vertex = pointCloud[i];
        if (vertex.z < 1 && vertex.z > 0) {
            vertexCounter[i]++;
            pointCloudAccumulator[i].x += vertex.x;
            pointCloudAccumulator[i].y += vertex.y;
            pointCloudAccumulator[i].z += vertex.z;
        }
    }
    capturingCounter++;
    output.capturingProgress = capturingCounter*100/settings.captureFrames;

    if (capturingCounter == settings.captureFrames)
    {
        LOG_DEBUG("creating model");

        cv::Rect faceRegion = getFullFaceRegion();
        Face::FaceData::VectorOfPoints points;
        Face::FaceData::Mesh::Colors colorsVec;
        ImageBGR &colorFrame = getCurrentColorFrame();
        for (int i = 0; i < depthBufferSize; i++)
        {
            int count = vertexCounter[i];
            if (count == 0) continue;

            const DepthSense::UV & uv = uvMap[i];
            if (uv.u == -FLT_MAX || uv.v == -FLT_MAX || depthMap[i] == -2.0)
            {
                continue;
            }

            int x = uv.u * ColorWidth;
            int y = uv.v * ColorHeight;
            if (!faceRegion.contains(cv::Point(x, y)))
            {
                continue;
            }

            const DepthSense::FPVertex &p = pointCloudAccumulator[i];
            points.push_back(cv::Point3d((1000.0 * p.x) / count, (1000.0 * p.y) / count, (-1000.0 * p.z) / count));
            //qDebug() << points.last().x << points.last().y << points.last().z;
            colorsVec.push_back(colorFrame(y, x));
        }

        //qDebug() << "points:" << points.count();
        //qDebug() << "colors:" << colorsVec.count();

        faceMesh = Face::FaceData::Mesh::fromPointcloud(points, true, true);
        faceMesh.colors = colorsVec;
        faceMesh.printStats();

        setState(CapturingDone);
    }

    getCurrentGrayscaleFrame().copyTo(output.currentFrame);
}

void DS325SensorBase::doCapturingDone()
{
    setState(Off);
}
