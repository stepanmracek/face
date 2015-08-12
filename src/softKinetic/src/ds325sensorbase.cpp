#include "softkinetic/ds325sensorbase.h"

#include <Poco/Exception.h>

#include "faceCommon/facedata/landmarks.h"
#include "softkinetic/logger.h"

using namespace Face::Sensors::SoftKinetic;

DS325SensorBase::DS325SensorBase(const Settings &settings, const std::string &landmarkDetectorPath) :
    context(DepthSense::Context::createStandalone()),
    contextThread("ds325.bg"),
    lmDetector(landmarkDetectorPath)
{
    init(settings);
}

DS325SensorBase::DS325SensorBase(const Settings &settings, const std::string &landmarkDetectorPath, DepthSense::Context & c) :
    context(c),
    contextThread("ds325.bg"),
    lmDetector(landmarkDetectorPath)
{
    init(settings);
}

void DS325SensorBase::init(const Settings &settings)
{
    faceDetectionRoi = cv::Rect(25, 0, 110, 120);
    this->settings = settings;

    depthMap.resize(DepthWidth*DepthHeight);
    pointCloud.resize(DepthWidth*DepthHeight);
    uvMap.resize(DepthWidth*DepthHeight);
    vertexCounter.resize(DepthWidth*DepthHeight);
    pointCloudAccumulator.resize(DepthWidth*DepthHeight);

    output.currentFrame = cv::Mat_<uchar>::zeros(ColorHeight, ColorWidth);
}

void DS325SensorBase::doLoop()
{
    if (output.state == STATE_IDLE || output.state == STATE_POSITIONING || output.state == STATE_CAPTURING){
        if (!lastDepth.isNull() && lastDepth.value().elapsed() / 1000 > 500){
			 LOG_ERROR("No depth buffer data in 500ms");
			 throw Poco::Exception("No depth data");
			 //this->start();
		}
	}
	output.previousState = output.state;
    switch (output.state)
    {
    case STATE_IDLE:
        doWaiting();
        break;
    case STATE_POSITIONING:
        doPositioning();
        break;
    case STATE_CAPTURING:
        doCapturing();
        break;
    case STATE_CAPTURING_DONE:
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

void DS325SensorBase::setEnableSmoothFilter(DepthSense::DepthNode& depthNode, bool enable) {
#if DEPTHSENSE_VERSION_BUILD > 1477
        depthNode.setEnableFilter3(enable);
#else
        depthNode.setEnableEparSmootherFilter(enable);
#endif
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
    setState(STATE_IDLE);

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
    output.state = STATE_OFF;

    LOG_DEBUG("stop called done");
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

    setEnableSmoothFilter(node, false);
    node.setConfidenceThreshold(DesiredConfidence);
    node.setConfiguration(config);
}

void DS325SensorBase::onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
	updateLastDepth();
    //LOG_TRACE("new depth sample");

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
        setState(STATE_IDLE);
        return;
    }

    if (tracker.isBigDisplacement())
    {
        optimalDistanceCounter = 0;
    }

    double distance = getDistance();

    //std::cout << "distance: " << distance << std::endl;

    output.faceRegion = cv::Rect();
    if (distance != distance) output.positioningOutput = POS_NONE;
    else if (distance > settings.maximalDistance) output.positioningOutput = POS_MOVE_CLOSER;
    else if (distance < settings.minimalDistance) output.positioningOutput = POS_MOVE_FAR;
    // else if (...) TODO: tilt;
    else
    {
        output.positioningOutput = POS_DONTMOVE;
        optimalDistanceCounter++;
        output.positioningProgress = optimalDistanceCounter*100/settings.consecutiveOptimalDistanceToStartCapturing;
        output.faceRegion = getFullFaceRegion();
    }

    if (optimalDistanceCounter == settings.consecutiveOptimalDistanceToStartCapturing)
    {
        setState(STATE_CAPTURING);
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
        setState(STATE_CAPTURING_DONE);
    }

    getCurrentGrayscaleFrame().copyTo(output.currentFrame);
}

void DS325SensorBase::doCapturingDone()
{
    LOG_DEBUG("creating model");

    int depthBufferSize = DepthWidth * DepthHeight;
    cv::Rect faceRegion = getFullFaceRegion();
    Face::FaceData::VectorOfPoints points;
    Face::FaceData::Mesh::Colors colorsVec;
    ImageBGR &colorFrame = getCurrentColorFrame();

    std::map<int, int> invUvMap;
    int depthCount = 0;
    for (int depthIndex = 0; depthIndex < depthBufferSize; depthIndex++)
    {
        int count = vertexCounter[depthIndex];
        if (count == 0) continue;

        const DepthSense::UV & uv = uvMap[depthIndex];
        if (uv.u == -FLT_MAX || uv.v == -FLT_MAX || depthMap[depthIndex] == -2.0)
        {
            continue;
        }

        int x = uv.u * ColorWidth;
        int y = uv.v * ColorHeight;
        int colorIndex = y*ColorWidth + x;
        invUvMap[colorIndex] = depthIndex;
        depthCount++;

        if (!faceRegion.contains(cv::Point(x, y)))
        {
            continue;
        }

        const DepthSense::FPVertex &p = pointCloudAccumulator[depthIndex];
        points.push_back(cv::Point3d((1000.0 * p.x) / count, (1000.0 * p.y) / count, (-1000.0 * p.z) / count));
        //qDebug() << points.last().x << points.last().y << points.last().z;
        colorsVec.push_back(colorFrame(y, x));
    }

    /*ImageGrayscale grayFrame = getCurrentGrayscaleFrame();
    std::vector<cv::Point2d> landmarks2d = lmDetector.get(grayFrame, faceRegion);
    if (landmarks2d.size() > 0)
    {
        LOG_TRACE("Detected " + std::to_string(landmarks2d.size()) + " landmarks");
        LOG_TRACE("Inverse UV map has " + std::to_string(invUvMap.size()) + " values; depth has " + std::to_string(depthCount) + " values");
        landmarks.points.clear();
        for (const cv::Point2d &p2d : landmarks2d)
        {
            LOG_TRACE("Landmark " + std::to_string(p2d.x) + " " + std::to_string(p2d.y));
            int colorIndex = p2d.y*640 + p2d.x;
            if (invUvMap.count(colorIndex) == 0)
            {
                LOG_TRACE("  not corresponding depth in inverse UV map");
                landmarks.points.push_back(cv::Point3d());
                continue;
            }
            int depthIndex = invUvMap[colorIndex];
            int count = vertexCounter[depthIndex];
            if (depthIndex >= depthBufferSize || depthIndex < 0 || count == 0)
            {
                landmarks.points.push_back(cv::Point3d());
                continue;
            }

            const DepthSense::FPVertex &p3d = pointCloudAccumulator[depthIndex];
            landmarks.points.push_back(cv::Point3d((1000.0 * p3d.x) / count, (1000.0 * p3d.y) / count, (-1000.0 * p3d.z) / count));
            cv::circle(grayFrame, p2d, 3, 255);
        }
        cv::imshow("lm preview", grayFrame);
        cv::Mat_<float> depth(DepthHeight, DepthWidth, depthMap.data());
        cv::imshow("depth preview", depth);
        cv::waitKey();
    }
    else
    {
        landmarks = Face::FaceData::Landmarks();
    }*/
    landmarks.points.clear();

    faceMesh = Face::FaceData::Mesh::fromPointcloud(points, false, true);
    faceMesh.colors = colorsVec;
    cv::Point3d centralizeShift = faceMesh.centralize();
    landmarks.translate(centralizeShift);
    faceMesh.printStats();

    setState(STATE_OFF);
}
