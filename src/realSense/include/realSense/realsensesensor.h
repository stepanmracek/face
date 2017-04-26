#ifndef REALSENSESENSOR
#define REALSENSESENSOR

#include <string>

#include "realsense.h"
#include "faceSensors/isensor.h"
#include "faceCommon/objectdetection/detector.h"
#include "faceCommon/objectdetection/tracker.h"

namespace Face {
namespace Sensors {
namespace RealSense {

class RealSenseSensorImpl;

class REALSENSE_EXPORTS RealSenseSensor : public Face::Sensors::ISensor
{
public:
	static const int FaceDetectionModulo = 5;
	static const int ConsecutiveDetectsToStartPositioning = 2;
	static const int MinimalDistance = 350;
	static const int MaximalDistance = 500; // 600;
	static const int MaximalPositioningDistance = 1000;
	static const int ConsecutiveNonDetectsToStopPositioning = 30;
	static const int ConsecutiveDetectsToStartCapturing = 10;
	static const int CaptureDepth = 75;

	static const int IVCAMLaserPower = 13;
	static const int IVCAMAccuracy = 1; //PXCCapture::Device::IVCAM_ACCURACY_FINEST;
	static const int IVCAMMotionRangeTradeOff = 75;
	static const int IVCAMFilterOption = 5;
	static const int DepthConfidenceThreshold = 13;
	static const int DepthAverageCount = 7;

	static const int CoverGesturePercentThreshold = 3;
	static const int CoverGestureDistance = 100;

	enum class PositioningFeature {
		None = 0, DepthMask = 1, PoseEstimation = 2
	};
	static const int PositionFeatures = static_cast<int>(PositioningFeature::PoseEstimation);

	enum class BrightnessControlType {
		None = 0, HeadRoiBased = 1
	};
	static const int BrightnessControl = static_cast<int>(BrightnessControlType::None);

private:
	RealSenseSensorImpl *impl;

public:
	struct Settings {
		int faceDetectionModulo;
		int consecutiveDetectsToStartPositioning;
		int minimalDistance;
		int maximalDistance;
		int maximalPositioningDistance;
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

		int positioningFeatures;

		int brightnessControl;

		ObjectDetection::Tracker::Setup trackerSetup;

		Settings(const std::string &landmarkModelPath = Face::Settings::instance().settingsMap[Face::Settings::MeanFaceModelLandmarksPathKey]) :
			faceDetectionModulo(FaceDetectionModulo),
			consecutiveDetectsToStartPositioning(ConsecutiveDetectsToStartPositioning),
			minimalDistance(MinimalDistance),
			maximalDistance(MaximalDistance),
			maximalPositioningDistance(MaximalPositioningDistance),
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
			coverGestureDistance(CoverGestureDistance),

			positioningFeatures(PositionFeatures),
			brightnessControl(BrightnessControl)
		{

		}

		bool hasPosFeature(PositioningFeature feature) const {
			return this->positioningFeatures & static_cast<int>(feature);
		}
	};

	RealSenseSensor(const Settings &settings = Settings(),
                    Face::ObjectDetection::Detector::Ptr faceDetector = new Face::ObjectDetection::OpenCVDetector());

	enum class FaceDetector {
		OpenCV_Cascade, OpenCV_Cascade_Depth, DLIB
	};
	RealSenseSensor(const Settings &settings,
                    FaceDetector faceDetector, const std::string& fdetectPath = std::string());
	virtual ~RealSenseSensor();

	SensorData sensorData();
	void start();
	void stop();
	void doLoop();

	static std::vector<unsigned int> connectedSensors();
};

}
}
}

#endif //REALSENSESENSOR
