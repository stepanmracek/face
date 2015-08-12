#pragma once

#include <pxcsensemanager.h>
#include <pxcprojection.h>
#include <memory>

#include "realSense/merger.h"
#include "realsenseprojection.h"
#include "faceCommon/facedata/landmarkdetector.h"
#include "faceCommon/facedata/facealigner.h"
#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/objectdetection/tracker.h"

namespace Face {
namespace Sensors {
namespace RealSense {

class MergerImpl
{
private:
	Merger::Settings settings;
	Merger::State state;
	Merger::PositioningOutput positioningOutput;

	pxcStatus sts;
	PXCSenseManager *senseManager;
	PXCVideoModule::DataDesc onDataDescription;
	PXCPoint3DF32* vertices;
	PXCPointF32* uvmap;
	PXCPointF32* invUvmap;
	PXCCapture::Device* device;
	PXCProjection* projection;
	float depthUnit;
	unsigned short depthLowConfidenceValue;

	RealSenseProjection::Ptr realSenseProjection;
	Face::FaceData::FaceAlignerLandmark aligner;
	Face::FaceData::LandmarkDetector::Landmarks2D landmarks2d;
	Face::FaceData::LandmarkDetector lmDetector;

	Face::FaceData::MapConverter mapConverter;
	Face::FaceData::Map depthAcc;

	Face::ObjectDetection::Tracker tracker;
	Face::FaceData::Mesh _mesh;
	Face::FaceData::Landmarks _landmarks;

	ImageBGR colorPreview;
	ImageGrayscale grayscalePreview;
	ImageGrayscale resizedPreview;
	ImageGrayscale roiPreview;
	cv::Rect detectRoi;
	Matrix depthMatrix;

	int idleFrameCounter;
	int capturingProgress;
	double nearestZpoint;
	bool _isCoverGesture;

	void singleCapture(bool calcUvMap, bool calcLandmarks);
	void getBuffers(PXCImage *depth, PXCImage *color, bool calcUvMap, bool calcLandmarks);
	void faceDetect();
	void calcMinZ();
	double calcDistance(const cv::Rect &realSizeRoi);
	cv::Rect toRealSizeRoi(const cv::Rect &trackerRoi) const;
	cv::Rect toRealSizeRoi4LMDetector(const cv::Rect &trackerRoi) const;
	cv::Rect toDepthRoi(const cv::Rect &realSizeRoi) const;

	Face::FaceData::Mesh getSingleMesh();

	void doIdle();
	void doPositioning();
	void doCapturing();
	void doCapturingDone();

public:
	MergerImpl(const Merger::Settings &settings, Face::ObjectDetection::Detector::Ptr faceDetector);
	~MergerImpl();
		
	const Face::FaceData::Mesh &mesh() const { return _mesh; }
	const Face::FaceData::Landmarks &landmarks() { return _landmarks; }

	ImageBGR getColorFrame() { return colorPreview; }
	ImageGrayscale getGrayscaleFrame() { return grayscalePreview; }

	void setState(Merger::State newState);
	Merger::State getState() const { return state; }
	Merger::State go();
	Merger::PositioningOutput getPositioningOutput() const { return positioningOutput; }
	cv::Rect getInitialDetectPosition() const { return toRealSizeRoi(tracker.getInitialRegion()); }
	int getPositioningProgress() const { return tracker.getConsecutiveDetects() * 100 / settings.consecutiveDetectsToStartCapturing; }
	int getCapturingProgress() const { return capturingProgress * 100 / settings.depthAverageCount;  }
	bool isCoverGesture() const { return _isCoverGesture; };
};

}
}
}