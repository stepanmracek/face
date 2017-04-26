#pragma once

#include <pxcsensemanager.h>
#include <pxcprojection.h>
#include <memory>

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/objectdetection/tracker.h"
#include "realSense/realsensesensor.h"
#include "landmarkdetector.h"
#include "realsenseprojection.h"
#include "faceposeestimator.h"
#include "framestats.h"
#include "realsensedepthfacetracker.h"

namespace Face {
namespace Sensors {
namespace RealSense {

class DepthAverage {
		typedef unsigned short InnerType;
	public:
		typedef std::shared_ptr<DepthAverage> Ptr;
		enum Type {
			Average, OmitMinMax
		};

		DepthAverage(std::size_t size, std::size_t avgCount, Type type, float depthUnitMicro, unsigned short lowConfidenceValue);
		virtual ~DepthAverage();

		void append(PXCImage* depth);

		void getAverage(PXCImage* depth) const;

	private:
		void getAverageOmitMinMax(std::vector<std::size_t>& avg, std::vector<std::size_t>& counts) const;

	private:
		std::deque<std::vector<InnerType> > depthAcc;
		std::deque<std::vector<std::size_t> > counterPlanes;
		std::size_t size;
		std::size_t avgCount;
		Type type;
		float depthUnitMicro;
		unsigned short lowConfidenceValue;
};


class RealSenseSensorImpl
{
private:
	RealSenseSensor::Settings settings;
	RealSenseSensor::State state;
	RealSenseSensor::PositioningOutput positioningOutput;

	pxcStatus sts;
	PXCSenseManager* senseManager;
	PXCVideoModule::DataDesc onDataDescription;
	PXCVideoModule::DataDesc offDataDescription;
	std::vector<PXCPoint3DF32> vertices;
	std::vector<PXCPoint3DF32> averagedVertices;
	std::vector<PXCPointF32> uvmap;
	std::vector<PXCPointF32> averagedUvmap;
	std::vector<PXCPointF32> invUvmap;
	std::vector<PXCPointF32> invAveragedUvmap;
	PXCCapture::Device* device;
	PXCProjection* projection;
	float depthUnit;
	unsigned short depthLowConfidenceValue;

	LandmarkDetector lmDetector;
	std::vector<cv::Point2d> landmarks;
	Poco::SharedPtr<FacePoseEstimator> facePoseEstimator;

	RealSenseProjection::Ptr realsenseProjection;
	RealSenseDepthFaceTracker::Ptr realsenseDepthFaceTracker;

	Poco::SharedPtr<Face::ObjectDetection::Tracker> tracker;
	Face::FaceData::Mesh _rawMesh;
	Face::FaceData::Mesh _processedMesh;
	Face::FaceData::Landmarks _rawLandmarks;
	Face::FaceData::Landmarks _processedLandmarks;
	DepthAverage::Ptr depthAverage;

	ImageBGR colorPreview;
	ImageGrayscale grayscalePreview;
	ImageGrayscale resizedPreview;
	ImageGrayscale roiPreview;
	cv::Rect detectRoi;
	Matrix depthMatrix;

	int idleFrameCounter;
	int capturingProgress;
	double nearestZpoint;
	int _isCoverGesture;
	int coverPercent;
	int targetColorMean;
	int diff;
	int brightnessToSet;

	FrameStats positioningFrameStats;

	void singleCapture(bool calcUvMap, bool calcLandmarks, bool releaseFrame);
	void getBuffers(PXCImage *depth, PXCImage *color, bool calcUvMap, bool calcLandmarks);
	void faceDetect(bool depthFiler);
	void calcMinZ();
	double calcDistance(const cv::Rect &realSizeRoi);
	cv::Rect toRealSizeRoi(const cv::Rect &trackerRoi) const;
	cv::Rect toRealSizeRoi4LMDetector(const cv::Rect &trackerRoi) const;
	cv::Rect toDepthRoi(const cv::Rect &realSizeRoi) const;
	static void convert(FacePoseEstimator::AlignInstruction poseInstruction, RealSenseSensor::PositioningOutput& sensorPosOutput);

	void doIdle();
	void doPositioning();
	void doCapturing();
	void doCapturingDone();

	void init(const RealSenseSensor::Settings &settings);
	void release();

public:
	RealSenseSensorImpl(const RealSenseSensor::Settings &settings, Face::ObjectDetection::Detector::Ptr faceDetector);
	RealSenseSensorImpl(const RealSenseSensor::Settings &settings, RealSenseSensor::FaceDetector faceDetector, const std::string& fdetectPath);
	~RealSenseSensorImpl();
		
	const Face::FaceData::Mesh &rawMesh() const { return _rawMesh; }
	const decltype(_rawLandmarks) &rawLandmarks() { return _rawLandmarks; }
	const Face::FaceData::Mesh &averagedMesh() const { return _processedMesh; }
	const decltype(_processedLandmarks) &averagedLandmarks() { return _processedLandmarks; }

	ImageBGR getColorFrame() { return colorPreview; }
	ImageGrayscale getGrayscaleFrame() { return grayscalePreview; }
	ImageGrayscale depth() const;

	void setState(RealSenseSensor::State newState);
	RealSenseSensor::State getState() const { return state; }
	RealSenseSensor::State go();
	RealSenseSensor::PositioningOutput getPositioningOutput() const { return positioningOutput; }
	cv::Rect getInitialDetectPosition() const { return toRealSizeRoi(tracker->getInitialRegion()); }
	int getPositioningProgress() const { return tracker->getConsecutiveDetects() * 100 / settings.consecutiveDetectsToStartCapturing; }
	int getCapturingProgress() const { return capturingProgress * 100 / settings.depthAverageCount;  }
	bool isCoverGesture() const { return _isCoverGesture > 2; };
	const decltype(landmarks) &getLandmarks() { return landmarks; }
};

}
}
}
