#include "mergerimpl.h"

#include <iostream>
#include <memory>
#include <limits>
#include <cmath>
#include <sstream>
#include <Poco/Timestamp.h>

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/procrustes.h"
#include "faceCommon/settings/settings.h"

using namespace Face::Sensors::RealSense;

MergerImpl::MergerImpl(const Merger::Settings &settings, Face::ObjectDetection::Detector::Ptr faceDetector) :
	settings(settings),
	senseManager(0),
	vertices(0),
	uvmap(0),
	invUvmap(0),
	device(0),
	projection(0),
	depthUnit(31.25f),
	depthLowConfidenceValue(0),
	tracker(faceDetector)
{

	senseManager = PXCSenseManager::CreateInstance();
	onDataDescription = {};
	onDataDescription.streams.depth.sizeMin.width = onDataDescription.streams.depth.sizeMax.width = 640;
	onDataDescription.streams.depth.sizeMin.height = onDataDescription.streams.depth.sizeMax.height = 480;
	onDataDescription.streams.depth.frameRate.min = 30;
	onDataDescription.streams.depth.frameRate.max = 30;

	onDataDescription.streams.color.sizeMin.width = onDataDescription.streams.color.sizeMax.width = 640;
	onDataDescription.streams.color.sizeMin.height = onDataDescription.streams.color.sizeMax.height = 480;
	onDataDescription.streams.color.frameRate.min = 30;
	onDataDescription.streams.color.frameRate.max = 30;

	sts = senseManager->EnableStreams(&onDataDescription);
	if (sts != PXC_STATUS_NO_ERROR) {
		std::string msg("Can't enable streams on RealSense camera (" + std::to_string(sts) + ")");
		std::cerr << msg << std::endl;
		throw FACELIB_EXCEPTION(msg);
	}

	sts = senseManager->Init();
	if (sts != PXC_STATUS_NO_ERROR) {
		std::string msg("Can't init RealSense camera (" + std::to_string(sts) + ")");
		std::cerr << msg << std::endl;
		throw FACELIB_EXCEPTION(msg);
	}

	vertices = new PXCPoint3DF32[640 * 480];
	uvmap = new PXCPointF32[640 * 480];
	invUvmap = new PXCPointF32[640 * 480];

	colorPreview = ImageBGR::zeros(480, 640);
	grayscalePreview = ImageGrayscale::zeros(480, 640);
	detectRoi = cv::Rect(40, 0, 240, 240);

	std::cout << "Setting RealSense device..." << std::endl;
	device = senseManager->QueryCaptureManager()->QueryDevice();
	device->SetMirrorMode(PXCCapture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL);
	
	device->SetIVCAMLaserPower(settings.ivcamLaserPower);
	device->SetIVCAMAccuracy((PXCCapture::Device::IVCAMAccuracy)settings.ivcamAccuracy);
	device->SetIVCAMMotionRangeTradeOff(settings.ivcamMotionRangeTradeOff);
	device->SetIVCAMFilterOption(settings.ivcamFilterOption);
	device->SetDepthConfidenceThreshold(settings.depthConfidenceThreshold);

	std::cout << "Laser power: " << device->QueryIVCAMLaserPower() << std::endl;
	std::cout << "Accuracy: " << device->QueryIVCAMAccuracy() << std::endl;
	std::cout << "Motion vs. range: " << device->QueryIVCAMMotionRangeTradeOff() << std::endl;
	std::cout << "Filter option: " << device->QueryIVCAMFilterOption() << std::endl;
	std::cout << "confidence threshold: " << device->QueryDepthConfidenceThreshold() << std::endl;
	depthUnit = device->QueryDepthUnit();
	std::cout << "Depth unit: " << depthUnit << std::endl;
	std::cout << "Low confidence value: " << device->QueryDepthLowConfidenceValue() << std::endl;

	projection = device->CreateProjection();
	realSenseProjection = new RealSenseProjection(senseManager, projection);

	depthMatrix = Matrix::zeros(480, 640);
}

MergerImpl::~MergerImpl()
{
	std::cout << "MergerImpl destructor" << std::endl;
	try {
		projection->Release();
		if (senseManager && senseManager->IsConnected()) {
			senseManager->Release();
			std::cout << "senseManager released." << std::endl;
		}
		if (uvmap) delete[] uvmap;
		if (vertices) delete[] vertices;
		if (invUvmap) delete[] invUvmap;
	} catch (std::exception& e) {
		std::cout << "Std Exception when destructing MergerImpl: " << e.what() << std::endl;
	} catch (...) {
		std::cout << "Unknown Exception when destructing MergerImpl." << std::endl;
	}
}

Face::FaceData::Mesh MergerImpl::getSingleMesh()
{
	int count = 0;
	Face::FaceData::VectorOfPoints points;
	Face::FaceData::Mesh::Colors colors;
	Face::FaceData::Mesh::UVMap uvMap;

	auto reg = toDepthRoi(toRealSizeRoi(tracker.getLastRegion()));
	int depthCount = 0;
	double zThreshold = nearestZpoint + settings.captureDepth;
	for (int r = 0; r < 480; r++) {
		for (int c = 0; c < 640; c++) {
			// skip points outside ROI
			if (r < reg.y || r >= reg.y + reg.height || c < reg.x || c >= reg.x + reg.width) continue;
			int i = r * 640 + c;

			//skip points to far from the nearest point
			if (depthMatrix(r, c) > zThreshold) continue;
			depthCount++;
			
			// skip not-valid points
			if (vertices[i].x != 0 && vertices[i].y != 0 && vertices[i].z != 0) {
				count++;

				cv::Point3d p(-vertices[i].x, vertices[i].y, -vertices[i].z);
				points.push_back(p);

				const PXCPointF32 &uv = uvmap[i];
				uvMap.push_back(cv::Vec2d(uv.x, uv.y));

				int colorCol = uv.x * 640;
				int colorRow = uv.y * 480;
				if (colorCol >= 640 || colorCol < 0 || colorRow >= 480 || colorRow < 0)
					colors.push_back(cv::Vec3b(0, 0, 0));
				else
					colors.push_back(colorPreview(colorRow, colorCol));
			}
		}
	}

	auto mesh = Face::FaceData::Mesh::fromPointcloud(points);
	mesh.colors = colors;
	return mesh;
}

Merger::State MergerImpl::go()
{
	switch (state)
	{
	case Merger::STATE_OFF:
		// do nothing
		break;

	case Merger::STATE_IDLE:
		// Check for face presence every 5th frame
		// Start positioning if face is present long enough
		doIdle();
		break;

	case Merger::STATE_POSITIONING:
		// Navigate user to proper position
		doPositioning();
		break;

	case Merger::STATE_CAPTURING:
		// Just convert current depth and color buffer to mesh
		doCapturing();
		break;

	case Merger::STATE_CAPTURING_DONE:
		// do nothing
		doCapturingDone();
		break;

	default:
		break;
	}

	return getState();
}

void MergerImpl::setState(Merger::State newState)
{
	if (state == newState) return;

	switch (newState)
	{
	case Merger::STATE_OFF:
		break;

	case Merger::STATE_IDLE:
		_isCoverGesture = false;
		_mesh.clear();
		tracker.init();
		idleFrameCounter = 0;
		break;

	case Merger::STATE_POSITIONING:
		tracker.init();
		break;

	case Merger::STATE_CAPTURING:
		depthAcc = Face::FaceData::Map(400, 400);
		depthAcc.setAll(0);
		capturingProgress = 0;
		break;

	case Merger::STATE_CAPTURING_DONE:
		break;

	default:
		break;
	}

	state = newState;
}

void MergerImpl::doIdle()
{
	singleCapture(false, false);
	idleFrameCounter = (idleFrameCounter + 1) % settings.faceDetectionModulo;
	if (idleFrameCounter == 0) {
		faceDetect();
		std::cout << "IDLE: face detects: " << tracker.getConsecutiveDetects() << "/" << settings.consecutiveDetectsToStartPositioning << std::endl;
	}

	if (tracker.getConsecutiveDetects() == settings.consecutiveDetectsToStartPositioning) {
		setState(Merger::STATE_POSITIONING);
	}
}

void MergerImpl::doPositioning()
{
	Merger::PositioningOutput prevPosOutput = positioningOutput;
	positioningOutput = Merger::POS_DONTMOVE;

	singleCapture(false, false);
	faceDetect();
	calcMinZ();
	
	if (tracker.getConsecutiveNonDetects() == settings.consecutiveNonDetectsToStopPositioning) {
		setState(Merger::STATE_IDLE);
		senseManager->ReleaseFrame();
		return;
	}

	cv::Rect reg = toRealSizeRoi(tracker.getLastRegion());
	double distance = calcDistance(reg);

	// add hysteresis for distance
	double targetMaxDistance = settings.maximalDistance;
	if (prevPosOutput == Merger::POS_MOVE_CLOSER) {
		targetMaxDistance *= 0.96;
	}
	double targetMinDistance = settings.minimalDistance;
	if (prevPosOutput == Merger::POS_MOVE_FAR) {
		targetMinDistance *= 1.04;
	}

	if (distance > targetMaxDistance) {
		positioningOutput = Merger::POS_MOVE_CLOSER;
	}
	else if (distance < targetMinDistance) {
		positioningOutput = Merger::POS_MOVE_FAR;
	}

	if (tracker.isLastDetect() && (tracker.isBigDisplacement() || positioningOutput != Merger::POS_DONTMOVE)) {
		tracker.init();
	}

	if (tracker.getConsecutiveDetects() == settings.consecutiveDetectsToStartCapturing) {
		setState(Merger::STATE_CAPTURING);
	}
}

void MergerImpl::doCapturing()
{
	if (capturingProgress == 0) {
		std::cout << "Capturing started; nearest z point: " << nearestZpoint << std::endl;
	}

	singleCapture(true, true);
	capturingProgress++;

	if (landmarks2d.size() > 5)
	{
		_mesh = getSingleMesh();
		aligner.align(_mesh, _landmarks);
		depthAcc.add(Face::FaceData::SurfaceProcessor::depthmap(_mesh, mapConverter, cv::Point2d(-100, -100), cv::Point2d(100, 100), 2.0, Face::FaceData::SurfaceProcessor::ZCoord));
		std::cout << depthAcc.w << " " << depthAcc.h << std::endl;

		cv::imshow("depthAcc", depthAcc.toMatrix());
	}

	if (capturingProgress == settings.depthAverageCount) {
		setState(Merger::STATE_CAPTURING_DONE);
	}
}

void MergerImpl::doCapturingDone()
{
	std::cout << "Capturing done" << std::endl;
	depthAcc.linearTransform(1.0 / settings.depthAverageCount, 0.0);

	Face::FaceData::Mesh::Colors colors;
	auto redMap = Face::FaceData::SurfaceProcessor::depthmap(_mesh, mapConverter, cv::Point2d(-100, -100), cv::Point2d(100, 100), 2.0, Face::FaceData::SurfaceProcessor::Texture_R);
	auto greenMap = Face::FaceData::SurfaceProcessor::depthmap(_mesh, mapConverter, cv::Point2d(-100, -100), cv::Point2d(100, 100), 2.0, Face::FaceData::SurfaceProcessor::Texture_G);
	auto blueMap = Face::FaceData::SurfaceProcessor::depthmap(_mesh, mapConverter, cv::Point2d(-100, -100), cv::Point2d(100, 100), 2.0, Face::FaceData::SurfaceProcessor::Texture_B);

	for (int y = 0; y < depthAcc.h; y++) {
		for (int x = 0; x < depthAcc.w; x++) {
			if (depthAcc.isSet(x, y)) {
				colors.push_back(Face::FaceData::Mesh::Color(blueMap.get(x, y), greenMap.get(x, y),  redMap.get(x, y)));
			}
		}
	}
	
	_mesh = Face::FaceData::Mesh::fromMap(depthAcc, mapConverter);
	_mesh.colors = colors;

	_mesh.printStats();


	setState(Merger::STATE_OFF);
}

void MergerImpl::faceDetect()
{
	cv::resize(grayscalePreview, resizedPreview, cv::Size(320, 240));
	roiPreview = resizedPreview(detectRoi);
	tracker.detect(roiPreview);
}

void MergerImpl::calcMinZ()
{
	nearestZpoint = DBL_MAX;
	if (!tracker.isLastDetect()) return;

	auto roi = toDepthRoi(toRealSizeRoi(tracker.getLastRegion()));
	auto depth = depthMatrix(roi);
	
	for (int r = 0; r < depth.rows; r++) {
		for (int c = 0; c < depth.cols; c++) {
			double value = depth(r, c);
			if (value > 0 && value < nearestZpoint) {
				nearestZpoint = value;
			}
		}
	}
}

cv::Rect MergerImpl::toRealSizeRoi(const cv::Rect &trackerRoi) const
{
	return cv::Rect(
		2 * (trackerRoi.x + detectRoi.x),
		2 * (trackerRoi.y + detectRoi.y),
		trackerRoi.width * 2,
		trackerRoi.height * 2);
}

cv::Rect MergerImpl::toDepthRoi(const cv::Rect &realSizeRoi) const
{
	return cv::Rect(
		realSizeRoi.x + 0.1*realSizeRoi.width,
		realSizeRoi.y + 0.07*realSizeRoi.height,
		0.9 * realSizeRoi.width,
		0.9 *realSizeRoi.height);
}

double MergerImpl::calcDistance(const cv::Rect &realSizeRoi)
{
	double sum = 0;
	int count = 0;

	cv::Rect depthRoi = toDepthRoi(realSizeRoi);
	
	for (int r = depthRoi.y; r < depthRoi.y + depthRoi.height; r++) {
		for (int c = depthRoi.x; c < depthRoi.x + depthRoi.width; c++) {
			if (r < 0 || r >= 640 || c < 0 || c >= 480) continue;
			double v = depthMatrix(r, c);
			if (v <= 0 || v > (nearestZpoint + settings.captureDepth)) continue;

			sum += v;
			count++;
		}
	}

	double d = count > 0 ? sum / count : 0;
	return d;
}

void MergerImpl::getBuffers(PXCImage *depth, PXCImage *color, bool calcUvMap, bool calcLandmarks)
{
	PXCImage::ImageData colorData;
	color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &colorData);
	unsigned char *colorDataStart = colorData.planes[0];

	PXCImage::ImageData depthData;
	depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_RAW, &depthData);
	unsigned short* dataStart = reinterpret_cast<unsigned short*>(depthData.planes[0]);

	int coveredCount = 0;
	for (int r = 0; r < 480; r++) {
		for (int c = 0; c < 640; c++) {
			int index = r * 640 + c;
			int colorIndex = 3 * index;
			unsigned char red = colorDataStart[colorIndex];
			unsigned char green = colorDataStart[colorIndex + 1];
			unsigned char blue = colorDataStart[colorIndex + 2];
			grayscalePreview(r, c) = 0.299*red + 0.587*green + 0.114*blue;
			colorPreview(r, c) = cv::Vec3b(red, green, blue);

			double value = dataStart[index] * depthUnit / 1000.0;
			depthMatrix(r, c) = value;

			if (value > 0 && value < settings.coverGestureDistance) {
				coveredCount++;
			}
		}
	}
	int coverPercent = coveredCount * 100 / (640 * 480);
	_isCoverGesture = coverPercent > settings.coverGesturePercentThreshold;

	if (calcUvMap) {
		std::cout << "UV map" << std::endl;
		projection->QueryVertices(depth, vertices);
		projection->QueryUVMap(depth, uvmap);
		projection->QueryInvUVMap(depth, invUvmap);

		if (calcLandmarks) {
			auto reg = toRealSizeRoi(tracker.getLastRegion());
			landmarks2d = lmDetector.get(grayscalePreview, reg);

			int n = landmarks2d.size();
			_landmarks.points = realSenseProjection->colorToWorld(depth, landmarks2d, Face::Sensors::RealSense::RealSenseProjection::Transformation::OpenGL);
		}
	}

	color->ReleaseAccess(&colorData);
	depth->ReleaseAccess(&depthData);
}

void MergerImpl::singleCapture(bool calcUvMap, bool calcLandmarks)
{
	sts = senseManager->AcquireFrame(true);

	if (sts != PXC_STATUS_NO_ERROR) {
		std::string msg("Can't acquire frame from camera (" + std::to_string(sts) + ")");
		std::cerr << msg << std::endl;
		throw FACELIB_EXCEPTION(msg);
	}

	PXCCapture::Sample *sample = senseManager->QuerySample();
	getBuffers(sample->depth, sample->color, calcUvMap, calcLandmarks);

	senseManager->ReleaseFrame();
}
