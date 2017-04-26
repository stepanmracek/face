#include "realsensesensorimpl.h"

#include <iostream>
#include <memory>
#include <limits>
#include <cmath>
#include <sstream>
#include <Poco/Timestamp.h>

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/procrustes.h"
#include "faceCommon/settings/settings.h"
#include "realsenseprocessor.h"
#include "realsensedetector.h"

using namespace Face::Sensors::RealSense;

const int MOTION_RANGE_POSITIONING_TOFF = 30;

void printTimeIfBig(Poco::Int64 t, const std::string& name) {
	if (t > 50 * 1000) {
		std::cout << " !!! " << name << " (ms): " << t / 1000 << std::endl;
	}
}

void check(pxcStatus status, const std::string& errMsg) {
	if (status != PXC_STATUS_NO_ERROR) {
		std::string msg(errMsg + "(" + std::to_string(status) + ")");
		std::cerr << msg << std::endl;
		throw FACELIB_EXCEPTION(msg);
	}
}

DepthAverage::DepthAverage(std::size_t size, std::size_t avgCount, Type type, float depthUnitMicro, unsigned short lowConfidenceValue) :
	size(size), avgCount(avgCount), type(type), depthUnitMicro(depthUnitMicro), lowConfidenceValue(lowConfidenceValue) {
}

DepthAverage::~DepthAverage() {
}

void DepthAverage::append(PXCImage* depth) {
	PXCImage::ImageData depthData;
	check(depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_RAW, &depthData), "Can't acquire depth image access");
	InnerType* depthDataPtr = reinterpret_cast<InnerType*>(depthData.planes[0]);

	std::size_t size = depth->QueryInfo().width * depth->QueryInfo().height;
	std::vector<InnerType> vec(depthDataPtr, depthDataPtr + size);
	check(depth->ReleaseAccess(&depthData), "Can't release depth image access");

	depthAcc.push_back(vec);

	std::vector<std::size_t> cnt(this->size, 0);
	for (auto it = vec.begin(); it != vec.end(); ++it) {
		if (*it == 0 || *it == this->lowConfidenceValue) {
			continue;
		}
		cnt[it - vec.begin()] = 1;
	}
	counterPlanes.push_back(cnt);

	while (depthAcc.size() > avgCount) {
		depthAcc.pop_front();
	}
	while (counterPlanes.size() > avgCount) {
		counterPlanes.pop_front();
	}
}

void DepthAverage::getAverageOmitMinMax(std::vector<std::size_t>& avg, std::vector<std::size_t>& counts) const {
	std::vector<std::pair<std::size_t, std::size_t> > minMaxIdxVec(this->size, std::pair<std::size_t, std::size_t>(0, 0));
	for (std::size_t iDepth = 0; iDepth < this->size; ++iDepth) {
		std::size_t nonzeros = 0;
		for (std::size_t iPlane = 1; iPlane < depthAcc.size(); ++iPlane) {
			auto val = depthAcc.at(iPlane).at(iDepth);
			if (val < depthAcc.at(minMaxIdxVec.at(iDepth).first).at(iDepth)) {
				minMaxIdxVec.at(iDepth).first = iPlane;
			}
			if (val > depthAcc.at(minMaxIdxVec.at(iDepth).second).at(iDepth)) {
				minMaxIdxVec.at(iDepth).second = iPlane;
			}
			nonzeros += val == 0 ? 0 : 1;
		}
		if (nonzeros > 2) {
			minMaxIdxVec.at(iDepth).first = 0;
			minMaxIdxVec.at(iDepth).second = 0;
		}
		for (std::size_t iPlane = 0; iPlane < depthAcc.size(); ++iPlane) {
			auto minIdx = minMaxIdxVec.at(iDepth).first;
			auto maxIdx = minMaxIdxVec.at(iDepth).second;
			if (minIdx != maxIdx) {
				if (iPlane != minIdx && iPlane != maxIdx) {
					avg.at(iDepth) += depthAcc.at(iPlane).at(iDepth);
				}
			} else {
				avg.at(iDepth) += depthAcc.at(iPlane).at(iDepth);
			}
		}
	}

	for (std::size_t iPlane = 0; iPlane < counterPlanes.size(); ++iPlane) {
		for (std::size_t i = 0; i < this->size; ++i) {
			auto minIdx = minMaxIdxVec.at(i).first;
			auto maxIdx = minMaxIdxVec.at(i).second;
			if (minIdx != maxIdx) {
				if (iPlane != minIdx && iPlane != maxIdx) {
					counts.at(i) += counterPlanes.at(iPlane).at(i);
				}
			} else {
				counts.at(i) += counterPlanes.at(iPlane).at(i);
			}
		}
	}
}

void DepthAverage::getAverage(PXCImage* depth) const {
	std::vector<std::size_t> avg(this->size, 0);
	std::vector<std::size_t> counts(this->size, 0);

	if (type == Average) {
		for (auto itPlane = depthAcc.begin(); itPlane != depthAcc.end(); ++itPlane) {
			for (auto itDepth = itPlane->begin(); itDepth != itPlane->end(); ++itDepth) {
				avg.at(itDepth - itPlane->begin()) += *itDepth;
			}
		}

		for (auto itCounterPlane = counterPlanes.begin(); itCounterPlane != counterPlanes.end(); ++itCounterPlane) {
			for (std::size_t i = 0; i < this->size; ++i) {
				counts.at(i) += itCounterPlane->at(i);
			}
		}
	}
	if (type == OmitMinMax) {
		getAverageOmitMinMax(avg, counts);
	}

	std::size_t maxCount = 0;
	for (std::size_t i = 0; i < this->size; ++i) {
		maxCount = std::max(maxCount, counts.at(i));
		if (counts.at(i) == 0) {
			avg.at(i) = 0;
			continue;
		}
		avg.at(i) /= 1.0 * counts.at(i);
	}
	std::cout << "Average depth max count: " << maxCount << std::endl;

	PXCImage::ImageData depthData;
	check(depth->AcquireAccess(PXCImage::ACCESS_WRITE, PXCImage::PIXEL_FORMAT_DEPTH_RAW, &depthData), "Can't acquire depth image access");
	InnerType* depthDataPtr = reinterpret_cast<InnerType*>(depthData.planes[0]);
	std::copy(avg.begin(), avg.end(), depthDataPtr);
	check(depth->ReleaseAccess(&depthData), "Can't release depth image access");
}

void RealSenseSensorImpl::release() {
	if (projection) {
		projection->Release();
	}
	if (senseManager && senseManager->IsConnected()) {
		senseManager->Release();
		std::cout << "senseManager released." << std::endl;
	}
}

void RealSenseSensorImpl::init(const RealSenseSensor::Settings &settings) {
	try {
		senseManager = 0;
		device = 0;
		projection = 0;

		this->settings = settings;

		depthUnit = 31.25f;
		depthLowConfidenceValue = 0;
		facePoseEstimator = new FacePoseEstimator(
			FacePoseEstimator::Settings(
			FacePoseEstimator::DefaultAlignTolerance,
			FacePoseEstimator::DefaultRollZeroAngle,
			FacePoseEstimator::DefaultPitchZeroAngle,
			FacePoseEstimator::DefaultYawZeroAngle)
		);

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

		offDataDescription = {};

		check(senseManager->EnableStreams(&onDataDescription), "Can't enable streams on RealSense camera");

		check(senseManager->Init(), "Can't init RealSense camera");

		vertices.resize(640 * 480);
		averagedVertices.resize(640 * 480);

		uvmap.resize(640 * 480);
		averagedUvmap.resize(640 * 480);

		invUvmap.resize(640 * 480);
		invAveragedUvmap.resize(640 * 480);

		colorPreview = ImageBGR::zeros(480, 640);
		grayscalePreview = ImageGrayscale::zeros(480, 640);
		detectRoi = cv::Rect(40, 0, 240, 240);

		std::cout << "Setting RealSense device..." << std::endl;
		device = senseManager->QueryCaptureManager()->QueryDevice();

		check(device->SetMirrorMode(PXCCapture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL), "Can't set mirrorMode");

		PXCCapture::Device::PropertyInfo gainInfo = device->QueryColorGainInfo();
		std::cout << "ColorGainInfo default value: " << gainInfo.defaultValue <<
			"; range: <" << gainInfo.range.min << "," << gainInfo.range.max << ">, step: " << gainInfo.step << std::endl;
		PXCCapture::Device::PropertyInfo brightnessInfo = device->QueryColorBrightnessInfo();
		std::cout << "ColorBrightnessInfo default value: " << brightnessInfo.defaultValue <<
			"; range: <" << brightnessInfo.range.min << "," << brightnessInfo.range.max << ">, step: " << brightnessInfo.step << std::endl;

		check(device->SetIVCAMLaserPower(settings.ivcamLaserPower), "Can't set laser power");
		check(device->SetIVCAMAccuracy((PXCCapture::Device::IVCAMAccuracy)settings.ivcamAccuracy), "Can't set accuracy");
		check(device->SetIVCAMMotionRangeTradeOff(MOTION_RANGE_POSITIONING_TOFF), "Can't set motion range tradeoff");
		check(device->SetIVCAMFilterOption(settings.ivcamFilterOption), "Can't set filter option");
		check(device->SetDepthConfidenceThreshold(settings.depthConfidenceThreshold), "Can't set confidence threshold");
		//check(device->SetColorBackLightCompensation(0), "Can't set backlight compensation");
		check(device->SetColorGain(gainInfo.defaultValue), "Can't set color gain");
		check(device->SetColorBrightness(brightnessInfo.defaultValue), "Can't set color brightness");

		std::cout << "Laser power: " << device->QueryIVCAMLaserPower() << std::endl;
		std::cout << "Accuracy: " << device->QueryIVCAMAccuracy() << std::endl;
		std::cout << "Motion vs. range: " << device->QueryIVCAMMotionRangeTradeOff() << std::endl;
		std::cout << "Filter option: " << device->QueryIVCAMFilterOption() << std::endl;
		std::cout << "confidence threshold: " << device->QueryDepthConfidenceThreshold() << std::endl;
		depthUnit = device->QueryDepthUnit();
		std::cout << "Depth unit: " << depthUnit << std::endl;
		std::cout << "Low confidence value: " << device->QueryDepthLowConfidenceValue() << std::endl;
		std::cout << "ColorBackLightCompensation value: " << device->QueryColorBackLightCompensation() << std::endl;
		std::cout << "ColorGain: " << device->QueryColorGain() << std::endl;
		std::cout << "ColorBrightness: " << device->QueryColorBrightness() << std::endl;
		
		PXCCapture::DeviceInfo deviceInfo;
		device->QueryDeviceInfo(&deviceInfo);
		std::cout << "Device firmware: " << deviceInfo.firmware[0] << "." << deviceInfo.firmware[1] << "." << deviceInfo.firmware[2] << "." << deviceInfo.firmware[3] << std::endl;

		int versionSum = deviceInfo.firmware[0] * 100 + deviceInfo.firmware[1];
		if (versionSum < 250) {
			std::stringstream sstr;
			sstr << "Old camera FW version: ";
			sstr << deviceInfo.firmware[0] << "." << deviceInfo.firmware[1] << "." << deviceInfo.firmware[2] << "." << deviceInfo.firmware[3];
			sstr << " (needs to be at least 2.50.X.X)";
			throw FACELIB_EXCEPTION(sstr.str());
		}


		projection = device->CreateProjection();

		realsenseProjection = new RealSenseProjection(senseManager, projection);
		realsenseDepthFaceTracker = new RealSenseDepthFaceTracker(realsenseProjection);

		depthMatrix = Matrix::zeros(480, 640);
		targetColorMean = 128;
		diff = 0;
		brightnessToSet = 0;
	} catch (std::exception&) {
		release();
		throw;
	}

}

RealSenseSensorImpl::RealSenseSensorImpl(const RealSenseSensor::Settings &settings, Face::ObjectDetection::Detector::Ptr faceDetector) :
		positioningFrameStats(3) {
	init(settings);

	tracker = new Face::ObjectDetection::Tracker(settings.trackerSetup, faceDetector);
}

RealSenseSensorImpl::RealSenseSensorImpl(const RealSenseSensor::Settings &settings, RealSenseSensor::FaceDetector faceDetector, const std::string& fdetectPath) :
		positioningFrameStats(3) {
	init(settings);

	Face::ObjectDetection::Detector::Ptr fd;
	switch (faceDetector) {
		case RealSenseSensor::FaceDetector::OpenCV_Cascade:
			fd = new Face::ObjectDetection::OpenCVDetector(fdetectPath);
			break;
		case RealSenseSensor::FaceDetector::OpenCV_Cascade_Depth: {
			Face::Sensors::RealSense::RealSenseDetector::Setup setup;
			setup.cascadeClassifierPath = fdetectPath;
			setup.scale = 0.5;
			setup.detectRoi = this->detectRoi;
			fd = new Face::Sensors::RealSense::RealSenseDetector(setup, realsenseDepthFaceTracker);
			break;
		}
		case RealSenseSensor::FaceDetector::DLIB:
			fd = new Face::ObjectDetection::DlibDetector();
			break;
	}

	tracker = new Face::ObjectDetection::Tracker(settings.trackerSetup, fd);
}

RealSenseSensorImpl::~RealSenseSensorImpl()
{
	std::cout << "RealSenseSensorImpl destructor" << std::endl;
	try {
		release();
	} catch (std::exception& e) {
		std::cout << "Std Exception when destructing RealSenseSensorImpl: " << e.what() << std::endl;
	} catch (...) {
		std::cout << "Unknown Exception when destructing RealSenseSensorImpl." << std::endl;
	}
}

RealSenseSensor::State RealSenseSensorImpl::go()
{
	try {
		if (brightnessToSet != 0) {
			Poco::Timestamp now;
			check(device->SetColorBrightness(brightnessToSet), "Set color brightness failed");
			printTimeIfBig(now.elapsed(), "setColorBrightnessTime");
			brightnessToSet = 0;
		}

		switch (state) {
			case RealSenseSensor::STATE_OFF:
				// do nothing
				break;

			case RealSenseSensor::STATE_IDLE:
				// Check for face presence every 5th frame
				// Start positioning if face is present long enough
				doIdle();
				break;

			case RealSenseSensor::STATE_POSITIONING:
				// Navigate user to proper position
				doPositioning();
				break;

			case RealSenseSensor::STATE_CAPTURING:
				// Just convert current depth and color buffer to mesh
				doCapturing();
				break;

			case RealSenseSensor::STATE_CAPTURING_DONE:
				// do nothing
				doCapturingDone();
				break;

			default:
				break;
		}
	} catch (std::exception& e) {
		std::cerr << "go() exception in state " << getState() << ": " << e.what() << std::endl;
	}

	return getState();
}

void RealSenseSensorImpl::setState(RealSenseSensor::State newState)
{
	if (state == newState) return;

	switch (newState)
	{
	case RealSenseSensor::STATE_OFF:
		break;

	case RealSenseSensor::STATE_IDLE:
		_isCoverGesture = 0;
		_rawMesh.clear();
		_processedMesh.clear();
		tracker->init();
		idleFrameCounter = 0;
		capturingProgress = 0;
		break;

	case RealSenseSensor::STATE_POSITIONING:
		tracker->init();
		facePoseEstimator->restart();
		positioningFrameStats.reset();
		check(device->SetIVCAMMotionRangeTradeOff(MOTION_RANGE_POSITIONING_TOFF), "Can't set motion range tradeoff");
		break;

	case RealSenseSensor::STATE_CAPTURING:
		landmarks.clear();
		check(device->SetIVCAMMotionRangeTradeOff(settings.ivcamMotionRangeTradeOff), "Can't set motion range tradeoff");
		this->depthAverage = std::make_shared<DepthAverage>(
			640 * 480,
			settings.depthAverageCount,
			DepthAverage::OmitMinMax,
			depthUnit,
			depthLowConfidenceValue
			);
		capturingProgress = 0;
		break;

	case RealSenseSensor::STATE_CAPTURING_DONE:
		this->depthAverage.reset();
		//_rawLandmarks.clear();
		//_processedLandmarks.clear();
		break;

	default:
		break;
	}

	state = newState;
}

void RealSenseSensorImpl::doIdle()
{
	singleCapture(false, false, true);
	idleFrameCounter = (idleFrameCounter + 1) % settings.faceDetectionModulo;
	if (idleFrameCounter == 0) {
		// scale color preview, use ROI
		faceDetect(false);
		std::cout << "IDLE: face detects: " << tracker->getConsecutiveDetects() << "/" << settings.consecutiveDetectsToStartPositioning << std::endl;
	}

	calcMinZ();
	if (nearestZpoint < settings.maximalPositioningDistance && tracker->getConsecutiveDetects() >= settings.consecutiveDetectsToStartPositioning) {
		setState(RealSenseSensor::STATE_POSITIONING);
	}

	if (Face::Settings::instance().debug()) {
		cv::Rect colorRoi = toRealSizeRoi(tracker->getLastRegion());
		cv::Rect depthRoi = toDepthRoi(colorRoi);
		cv::rectangle(grayscalePreview, colorRoi, 255);
		cv::rectangle(depthMatrix, depthRoi, 255);
	}
}

void shadowText(ImageGrayscale& im, const std::string& text, const cv::Point& pos, double scale) {
	cv::Scalar white(255, 255, 255);
	cv::Scalar black(0, 0, 0);

	cv::putText(im, text, cv::Point(pos.x + 2, pos.y + 2), CV_FONT_HERSHEY_SIMPLEX, scale, black, 1, CV_AA);
	cv::putText(im, text, pos, CV_FONT_HERSHEY_SIMPLEX, scale, white, 1, CV_AA);
};


void RealSenseSensorImpl::doPositioning()
{
	Poco::Int64 singleCaptureTime = 0;
	Poco::Int64 faceDetectTime = 0;
	Poco::Int64 pose1Time = 0;
	Poco::Int64 pose2Time = 0;
	Poco::Int64 frameReleaseTime = 0;
	Poco::Int64 frameTime = 0;

	static Poco::Timestamp frameTimer;

	RealSenseSensor::PositioningOutput prevPosOutput = positioningOutput;
	positioningOutput = RealSenseSensor::POS_NONE;

	Poco::Timestamp now;
	singleCapture(false, false, false);
	singleCaptureTime = now.elapsed();
	
	now.update();
	faceDetect(settings.hasPosFeature(RealSenseSensor::PositioningFeature::DepthMask));
	faceDetectTime = now.elapsed();

	frameTime = frameTimer.elapsed();
	frameTimer.update();

	calcMinZ();

	if (tracker->getConsecutiveNonDetects() >= settings.consecutiveNonDetectsToStopPositioning ||
			(tracker->isLastDetect() && nearestZpoint > settings.maximalPositioningDistance)) {
		setState(RealSenseSensor::STATE_IDLE);
		senseManager->ReleaseFrame();
		return;
	}

	if (tracker->getConsecutiveNonDetects() > 10) {
		positioningOutput = RealSenseSensor::POS_NONE;
		senseManager->ReleaseFrame();
		return;
	}

	if (!tracker->isLastDetect()) {
		senseManager->ReleaseFrame();
		return;
	}

	cv::Rect reg = toRealSizeRoi(tracker->getLastRegion());
	double distance = calcDistance(reg);

	now.update();

	FacePoseEstimator::Pose pose = FacePoseEstimator::Pose::UnknownPose();

	if (settings.hasPosFeature(RealSenseSensor::PositioningFeature::PoseEstimation) && tracker->isLastDetect()) {
		landmarks = lmDetector.getLandmarks(grayscalePreview, toRealSizeRoi4LMDetector(tracker->getLastRegion()), false);
		std::map<std::string, std::vector<cv::Point2d> > selectedLMPoints = lmDetector.lmGroupPoints(landmarks);
		pose1Time = now.elapsed();
		
		now.update();
		pose = facePoseEstimator->facePose(selectedLMPoints, realsenseProjection);
		pose2Time = now.elapsed();

		if (Face::Settings::instance().debug() && pose != FacePoseEstimator::Pose::UnknownPose()) {
			cv::Scalar red(0, 0, 255);
			cv::Scalar green(0, 255, 0);
			cv::Scalar white(255, 255, 255);
			cv::Scalar black(0, 0, 0);

			{
				positioningFrameStats.update();
				std::stringstream sstr;
				sstr.precision(1);
				sstr.setf(std::ios::fixed, std::ios::floatfield);
				sstr << "Pitch: " << pose.pitch - pose.relativePitch << "/" << facePoseEstimator->getSettings().pitchZeroAngle <<
					", Yaw: " << pose.yaw - pose.relativeYaw << "/" << facePoseEstimator->getSettings().yawZeroAngle <<
					", FPS: " << positioningFrameStats.fps();
				sstr.flush();
				shadowText(grayscalePreview, sstr.str(), cv::Point(150, 15), 0.5);

				{
					std::stringstream sstrDebug;
					sstrDebug << std::dec << "Capture time: " << singleCaptureTime / 1000 << "; FaceDetect time: " << faceDetectTime / 1000;
					shadowText(grayscalePreview, sstrDebug.str(), cv::Point(150, 45), 0.5);
				}

				{
					std::stringstream sstrDebug;
					sstrDebug << std::dec << "pose time: " << pose1Time / 1000 << "/" << pose2Time / 1000 << "; frame time: " << frameTime / 1000;
					shadowText(grayscalePreview, sstrDebug.str(), cv::Point(150, 60), 0.5);
				}

				auto mirror = [](const cv::Mat& img, const cv::Point& p) -> cv::Point {
					return cv::Point(img.cols - p.x, p.y);
				};

				cv::line(grayscalePreview, mirror(grayscalePreview, pose.imageAxisX.from), mirror(grayscalePreview, pose.imageAxisX.to), black);
				cv::line(grayscalePreview, mirror(grayscalePreview, pose.imageAxisY.from), mirror(grayscalePreview, pose.imageAxisY.to), black);
			}

			cv::rectangle(grayscalePreview, toRealSizeRoi4LMDetector(tracker->getLastRegion()), 255);

			for (const auto& lm : landmarks) {
				cv::circle(grayscalePreview, lm, 2, black);
			}
		}

		landmarks.clear();
	}

	// add hysteresis for distance
	double targetMaxDistance = settings.maximalDistance;
	if (prevPosOutput == RealSenseSensor::POS_MOVE_CLOSER) {
		targetMaxDistance *= 0.96;
	}
	double targetMinDistance = settings.minimalDistance;
	if (prevPosOutput == RealSenseSensor::POS_MOVE_FAR) {
		targetMinDistance *= 1.04;
	}

	if (distance > targetMaxDistance) {
		positioningOutput = RealSenseSensor::POS_MOVE_CLOSER;
	}
	else if (distance < targetMinDistance) {
		positioningOutput = RealSenseSensor::POS_MOVE_FAR;
	}
	else {
		FacePoseEstimator::AlignInstruction instruction = facePoseEstimator->giveFaceAlignInstruction(pose);
		convert(instruction, positioningOutput);
	}

	if (tracker->positionUnsteady() || positioningOutput != RealSenseSensor::POS_DONTMOVE) {
		tracker->init();
	}

	if (positioningOutput == RealSenseSensor::POS_DONTMOVE && tracker->getConsecutiveDetects() >= settings.consecutiveDetectsToStartCapturing) {
		setState(RealSenseSensor::STATE_CAPTURING);
	}

	senseManager->ReleaseFrame();
//	std::cout << std::dec << "SingleCapture time: " << singleCaptureTime << "; FaceDetect time: " << faceDetectTime;
//	std::cout << std::dec << "; pose time: " << pose1Time << "/" << pose2Time << "; frame time: " << frameTime << std::endl;
}

void RealSenseSensorImpl::doCapturing()
{
	if (capturingProgress == 0) {
		std::cout << "Capturing started; nearest z point: " << nearestZpoint << std::endl;
	}

	if (capturingProgress < (settings.depthAverageCount - 1)) {
		singleCapture(false, false, true);
		capturingProgress++;
	}
	else {
		singleCapture(true, true, true);

		capturingProgress++;
		setState(RealSenseSensor::STATE_CAPTURING_DONE);
	}
}

void RealSenseSensorImpl::doCapturingDone()
{
	std::cout << "Capturing done" << std::endl;

	int count = 0;
	Face::FaceData::VectorOfPoints points;
	Face::FaceData::Mesh::Colors colors;
	Face::FaceData::Mesh::UVMap uvMap;

	int averagedCount = 0;
	Face::FaceData::VectorOfPoints averagedPoints;
	Face::FaceData::Mesh::Colors averagedColors;
	Face::FaceData::Mesh::UVMap averagedUvMap;
	
	auto colorRoi = toRealSizeRoi(tracker->getLastRegion());
	int depthCount = 0;
	double zThreshold = nearestZpoint + settings.captureDepth;
	for (int r = 0; r < 480; r++) {
		for (int c = 0; c < 640; c++) {
			int i = r * 640 + c;

			//skip points to far from the nearest point
			if (depthMatrix(r, c) > zThreshold) continue;

			// skip not-valid points
			if (vertices[i].x != 0 && vertices[i].y != 0 && vertices[i].z != 0) {
				const PXCPointF32 &uv = uvmap[i];
				int colorCol = uv.x * 640;
				int colorRow = uv.y * 480;

				// skip points outside ROI
				if (!cv::Point2i(colorCol, colorRow).inside(colorRoi)) {
					continue;
				}

				count++;

				cv::Point3d p(-vertices[i].x, vertices[i].y, -vertices[i].z);
				points.push_back(p);

				uvMap.push_back(cv::Vec2d(uv.x, uv.y));

				if (colorCol >= 640 || colorCol < 0 || colorRow >= 480 || colorRow < 0)
					colors.push_back(cv::Vec3b(0, 0, 0));
				else
					colors.push_back(colorPreview(colorRow, colorCol));
			}

			// skip not-valid points
			if (averagedVertices[i].x != 0 && averagedVertices[i].y != 0 && averagedVertices[i].z != 0 && averagedVertices[i].z <= zThreshold) {
				const PXCPointF32 &uv = averagedUvmap[i];
				int colorCol = uv.x * 640;
				int colorRow = uv.y * 480;

				// skip points outside ROI
				if (!cv::Point2i(colorCol, colorRow).inside(colorRoi)) {
					continue;
				}

				averagedCount++;

				cv::Point3d p(-averagedVertices[i].x, averagedVertices[i].y, -averagedVertices[i].z);
				averagedPoints.push_back(p);

				averagedUvMap.push_back(cv::Vec2d(uv.x, uv.y));

				if (colorCol >= 640 || colorCol < 0 || colorRow >= 480 || colorRow < 0)
					averagedColors.push_back(cv::Vec3b(0, 0, 0));
				else
					averagedColors.push_back(colorPreview(colorRow, colorCol));
			}

			depthCount++;
		}
	}

	std::cout << "Depth points: " << depthCount << std::endl;
	std::cout << "Added " << count << " points" << std::endl;
	std::cout << "Added " << averagedCount << " averaged points" << std::endl;

	_rawMesh = Face::FaceData::Mesh::fromPointcloud(points, false, true);
	_rawMesh.colors = colors;
	_rawMesh.uvmap = uvMap;
	cv::Point3d shift = _rawMesh.centralize();
	_rawLandmarks.translate(shift);
	
	_processedMesh = Face::FaceData::Mesh::fromPointcloud(averagedPoints, false, true);
	_processedMesh.colors = averagedColors;
	_processedMesh.uvmap = averagedUvMap;
	shift = _processedMesh.centralize();
	_processedLandmarks.translate(shift);

	setState(RealSenseSensor::STATE_OFF);
}

ImageGrayscale RealSenseSensorImpl::depth() const {
	ImageGrayscale grey(depthMatrix.size(), CV_8UC1);
	depthMatrix.convertTo(grey, CV_8UC1);

	if (getState() == RealSenseSensor::STATE_IDLE) {
		shadowText(grey, std::string("Covered: ") + Poco::NumberFormatter::format(coverPercent), cv::Point(320, 50), 1.0);
	}

	return grey;
}

void RealSenseSensorImpl::faceDetect(bool depthMask)
{
	Poco::Int64 maskTime = 0;
	Poco::Int64 detectTime = 0;

	Poco::Timestamp now;
	ImageGrayscale mask(grayscalePreview.clone());
	if (depthMask) {
		PXCCapture::Sample *sample = senseManager->QuerySample();
		mask = RealSenseProcessor::depthMask(sample->depth, sample->color, projection, grayscalePreview);
		//cv::imshow("Preview mask", mask);
	}
	maskTime = now.elapsed();

	now.update();
	cv::resize(mask, resizedPreview, cv::Size(320, 240), 0, 0, CV_INTER_NN);
	roiPreview = resizedPreview(detectRoi);
	tracker->detect(roiPreview);
	detectTime = now.elapsed();

	if (depthMask && tracker->isLastDetect()) {
		cv::Rect faceRoi = toRealSizeRoi(tracker->getLastRegion());
		mask = RealSenseProcessor::fillHoles(faceRoi, mask, grayscalePreview, 0.0);
		//cv::imshow("grayscalePreview mask 2", mask);
	}

	//std::cout << std::dec << "Mask time: " << maskTime << "; Face detect time: " << detectTime << std::endl;

	/*
	if (Face::Settings::instance().debug()) {
		if (tracker->isLastDetect()) {
			cv::rectangle(colorPreview, toRealSizeRoi(tracker->getLastRegion()), 255);
		}
		//cv::imshow("detection", roiPreview);
	}*/
}

void RealSenseSensorImpl::calcMinZ()
{
	nearestZpoint = DBL_MAX;
	if (!tracker->isLastDetect()) return;

	auto roi = toDepthRoi(toRealSizeRoi(tracker->getLastRegion()));
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

cv::Rect RealSenseSensorImpl::toRealSizeRoi(const cv::Rect &trackerRoi) const
{
	return cv::Rect(
		2 * (trackerRoi.x + detectRoi.x),
		2 * (trackerRoi.y + detectRoi.y),
		trackerRoi.width * 2,
		trackerRoi.height * 2);
}

cv::Rect RealSenseSensorImpl::toRealSizeRoi4LMDetector(const cv::Rect &trackerRoi) const {
	cv::Rect rect(toRealSizeRoi(trackerRoi));

	rect.height *= 1.2;
	if (rect.y + rect.height > grayscalePreview.rows) {
		rect.height = grayscalePreview.rows - rect.y - 1;
	}

	return rect;
}

cv::Rect RealSenseSensorImpl::toDepthRoi(const cv::Rect &realSizeRoi) const
{
	return cv::Rect(
		realSizeRoi.x + 0.1*realSizeRoi.width,
		realSizeRoi.y + 0.07*realSizeRoi.height,
		0.9 * realSizeRoi.width,
		0.9 *realSizeRoi.height);
}

void RealSenseSensorImpl::convert(FacePoseEstimator::AlignInstruction poseInstruction, RealSenseSensor::PositioningOutput& sensorPosOutput) {
	switch (poseInstruction) {
		case FacePoseEstimator::AlignInstruction::UNKNOWN: 
			sensorPosOutput = RealSenseSensor::POS_NONE;
			break;
		case FacePoseEstimator::AlignInstruction::OK:
			sensorPosOutput = RealSenseSensor::POS_DONTMOVE;
			break;
		case FacePoseEstimator::AlignInstruction::LOOK_LEFT:
			sensorPosOutput = RealSenseSensor::POS_LOOK_LEFT;
			break;
		case FacePoseEstimator::AlignInstruction::LOOK_RIGHT:
			sensorPosOutput = RealSenseSensor::POS_LOOK_RIGHT;
			break;
		case FacePoseEstimator::AlignInstruction::LOOK_UP:
			sensorPosOutput = RealSenseSensor::POS_LOOK_UP;
			break;
		case FacePoseEstimator::AlignInstruction::LOOK_DOWN:
			sensorPosOutput = RealSenseSensor::POS_LOOK_DOWN;
			break;
		case FacePoseEstimator::AlignInstruction::ROTATE_LEFT:
			sensorPosOutput = RealSenseSensor::POS_ROTATE_LEFT;
			break;
		case FacePoseEstimator::AlignInstruction::ROTATE_RIGHT:
			sensorPosOutput = RealSenseSensor::POS_ROTATE_RIGHT;
			break;
	}
}

double RealSenseSensorImpl::calcDistance(const cv::Rect &realSizeRoi)
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

	/*double min, max;
	cv::minMaxIdx(depthMatrix, &min, &max);
	Matrix depthAndRoi = (depthMatrix - min) / (max - min);
	cv::rectangle(depthAndRoi, depthRoi, 1);
	cv::imshow("depth and roi", depthAndRoi);

	ImageGrayscale colorAndRoi = grayscalePreview.clone();
	cv::rectangle(colorAndRoi, realSizeRoi, 255);
	cv::imshow("color and roi", colorAndRoi);*/

	double d = count > 0 ? sum / count : 0;
	//std::cout << "calcDistance: " << d << std::endl;
	return d;
}

void RealSenseSensorImpl::getBuffers(PXCImage *depth, PXCImage *color, bool calcUvMap, bool calcLandmarks)
{
	PXCImage::ImageData colorData;
	check(color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &colorData), "Can't acquire access to color image");
	unsigned char *colorDataStart = colorData.planes[0];

	colorPreview = cv::Mat(480, 640, CV_8UC3, colorDataStart);
	check(color->ReleaseAccess(&colorData), "Can't release color image access");
	cv::cvtColor(colorPreview, grayscalePreview, CV_RGB2GRAY);

	PXCImage::ImageData depthData;
	check(depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_RAW, &depthData), "Can't acquire access to depth image");
	unsigned short* dataStart = reinterpret_cast<unsigned short*>(depthData.planes[0]);

	depthMatrix = cv::Mat(480, 640, CV_16U, dataStart).clone();
	check(depth->ReleaseAccess(&depthData), "Can't release depth image access");
	double multiplier = depthUnit / 1000.0;
	depthMatrix = multiplier * depthMatrix;

	{
		cv::Mat scaled(48, 64, CV_16U);
		cv::resize(depthMatrix, scaled, cv::Size(scaled.cols, scaled.rows), 0, 0, CV_INTER_NN);
		cv::Mat threshed(48, 64, CV_32FC1);
		depthMatrix.convertTo(threshed, CV_32FC1);
		cv::threshold(threshed, threshed, settings.coverGestureDistance, 1.0, CV_THRESH_TOZERO_INV);
		int coveredCount = cv::countNonZero(threshed);

		coverPercent = coveredCount * 100 / (threshed.cols * threshed.rows);
		_isCoverGesture = coverPercent > settings.coverGesturePercentThreshold ? _isCoverGesture + 1 : 0;
	}

	if (depthAverage.get()) {
		depthAverage->append(depth);
	}

	if (calcUvMap) {
		std::cout << "UV map" << std::endl;

		if (calcLandmarks) {
			auto reg = toRealSizeRoi4LMDetector(tracker->getLastRegion());
			landmarks = lmDetector.getLandmarks(grayscalePreview, reg, true);

			_rawLandmarks = realsenseProjection->colorToWorld(depth, landmarks, RealSenseProjection::Transformation::OpenGL);
		}

		projection->QueryVertices(depth, vertices.data());
		projection->QueryUVMap(depth, uvmap.data());
		projection->QueryInvUVMap(depth, invUvmap.data());
		
		if (depthAverage.get()) {
			depthAverage->getAverage(depth);

			if (calcLandmarks) {
				_processedLandmarks = realsenseProjection->colorToWorld(landmarks, RealSenseProjection::Transformation::OpenGL);
			}
			projection->QueryVertices(depth, averagedVertices.data());
			projection->QueryUVMap(depth, averagedUvmap.data());
			projection->QueryInvUVMap(depth, invAveragedUvmap.data());
		}
	}
}

void RealSenseSensorImpl::singleCapture(bool calcUvMap, bool calcLandmarks, bool releaseFrame)
{
	Poco::Int64 acquireTime = 0;
	Poco::Int64 getBuffersTime = 0;
	
	Poco::Timestamp now;
	sts = senseManager->AcquireFrame(true);

	if (sts != PXC_STATUS_NO_ERROR) {
		senseManager->ReleaseFrame();
		std::string msg("Can't acquire frame from camera (" + std::to_string(sts) + ")");
		std::cerr << msg << std::endl;
		throw FACELIB_EXCEPTION(msg);
	}
	PXCCapture::Sample *sample = senseManager->QuerySample();
	acquireTime = now.elapsed();
	
	now.update();
	getBuffers(sample->depth, sample->color, calcUvMap, calcLandmarks);
	getBuffersTime = now.elapsed();

	try {
		brightnessToSet = 0;
		Poco::Int64 detectFaceRoiTime = 0;
		Poco::Int64 meanCalcTime = 0;
		Poco::Int64 queryColorBrightnessTime = 0;
		Poco::Int64 queryColorBrightnessInfoTime = 0;
		Poco::Int64 setColorBrightnessTime = 0;

		now.update();
		cv::Rect croi = realsenseDepthFaceTracker->detectFaceRoi(depthMatrix, sample->depth, RealSenseDepthFaceTracker::Domain::Color, false /*debug*/);
		detectFaceRoiTime = now.elapsed();

		// brightness control
		if ((settings.brightnessControl == static_cast<int>(RealSenseSensor::BrightnessControlType::HeadRoiBased) && croi.area() && 
			(state == RealSenseSensor::STATE_POSITIONING || state == RealSenseSensor::STATE_IDLE))) {
			now.update();
			cv::Mat mask = cv::Mat::zeros(grayscalePreview.size(), CV_8U);
			cv::rectangle(mask, croi, cv::Scalar(255), CV_FILLED);
			int mean = cv::mean(grayscalePreview, mask)[0];
			meanCalcTime = now.elapsed();

//			if (Face::Settings::instance().debug()) {
//				cv::Mat maskDrawing = grayscalePreview.clone();
//				cv::rectangle(maskDrawing, croi, cv::Scalar(255));
//				cv::imshow("Depth roi for gain projected to color", maskDrawing);
//			}

			now.update();
			pxcI32 brightness = device->QueryColorBrightness();
			queryColorBrightnessTime = now.elapsed();
			diff = targetColorMean - mean;

			auto newValue = [=](const PXCCapture::Device::PropertyInfo& info, pxcI32 val, pxcI32 newVal) -> pxcI32 {
				pxcI32 newV = std::max(info.range.min, std::min(static_cast<pxcF32>(newVal), info.range.max));
				if (Face::Settings::instance().debug()) {
					std::stringstream sstr;
					sstr << "Val: " << val << "; adjusted val: " << newV << "; mean: " << mean << "/" << targetColorMean;
					shadowText(grayscalePreview, sstr.str(), cv::Point(150, 30), 0.5);
				}
				return newV;
			};

			int _diff = diff == 0 ? 0 : diff / abs(diff);
			pxcI32 newBrightness = newValue(device->QueryColorBrightnessInfo(), brightness, brightness + _diff);
			if (abs(diff) > 15) {
				brightnessToSet = newBrightness;
			}
		}
	} catch (std::exception& e) {
		std::cout << " --- Brightness control exception: " << e.what() << std::endl;
	}

	//std::cout << std::dec << "Acquire time: " << acquireTime << "; getBuffers time: " << getBuffersTime << std::endl;

	if (releaseFrame) {
		senseManager->ReleaseFrame();
	}
}


