#pragma once

#include <string>

#include "realsense.h"
#include "faceSensors/isensor.h"
#include "faceCommon/objectdetection/detector.h"

namespace Face {
namespace Sensors {
namespace RealSense {

class MergerImpl;

class REALSENSE_EXPORTS Merger : public Face::Sensors::ISensor
{
public:
	static const int FaceDetectionModulo = 5;
	static const int ConsecutiveDetectsToStartPositioning = 3;
	static const int MinimalDistance = 400;
	static const int MaximalDistance = 500; // 600;
	static const int ConsecutiveNonDetectsToStopPositioning = 10;
	static const int ConsecutiveDetectsToStartCapturing = 10;
	static const int CaptureDepth = 75;

	static const int IVCAMLaserPower = 13;
	static const int IVCAMAccuracy = 1; //PXCCapture::Device::IVCAM_ACCURACY_FINEST;
	static const int IVCAMMotionRangeTradeOff = 75;
	static const int IVCAMFilterOption = 5;
	static const int DepthConfidenceThreshold = 13;
	static const int DepthAverageCount = 25;

	static const int CoverGesturePercentThreshold = 3;
	static const int CoverGestureDistance = 100;

private:
	MergerImpl *impl;

public:
	struct Settings {
		int faceDetectionModulo;
		int consecutiveDetectsToStartPositioning;
		int minimalDistance;
		int maximalDistance;
		int consecutiveNonDetectsToStopPositioning;
		int consecutiveDetectsToStartCapturing;
		int capturingFrames;
		int captureDepth;

		int ivcamLaserPower;
		int ivcamAccuracy;
		int ivcamMotionRangeTradeOff;
		int ivcamFilterOption;
		int depthConfidenceThreshold;
		int depthAverageCount;

		int coverGesturePercentThreshold;
		int coverGestureDistance;

		Settings() :
			faceDetectionModulo(FaceDetectionModulo),
			consecutiveDetectsToStartPositioning(ConsecutiveDetectsToStartPositioning),
			minimalDistance(MinimalDistance),
			maximalDistance(MaximalDistance),
			consecutiveNonDetectsToStopPositioning(ConsecutiveNonDetectsToStopPositioning),
			consecutiveDetectsToStartCapturing(ConsecutiveDetectsToStartCapturing),
			captureDepth(CaptureDepth),

			ivcamLaserPower(IVCAMLaserPower),
			ivcamAccuracy(IVCAMAccuracy),
			ivcamMotionRangeTradeOff(IVCAMMotionRangeTradeOff),
			ivcamFilterOption(IVCAMFilterOption),
			depthConfidenceThreshold(DepthConfidenceThreshold),
			depthAverageCount(DepthAverageCount),

			coverGesturePercentThreshold(CoverGesturePercentThreshold),
			coverGestureDistance(CoverGestureDistance)
		{

		}
	};

	Merger(const Settings &settings = Settings(), Face::ObjectDetection::Detector::Ptr faceDetector = new Face::ObjectDetection::OpenCVDetector());
	virtual ~Merger();

	SensorData sensorData();
	State getState();
	void start();
	void stop();
	void doLoop();
};

}
}
}
