#ifndef REALTIMETRACK_H
#define REALTIMETRACK_H

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <vector>
#include <Poco/Timestamp.h>

#include "faceCommon/linalg/common.h"
#include "faceCommon/faceCommon.h"
#include "faceCommon/objectdetection/detector.h"

namespace Face {
namespace ObjectDetection {

class FACECOMMON_EXPORTS Tracker
{
public:
	struct Setup {
		struct MovementSpan {
			bool active;
			int timeFrameMs;
			int moveThresh;
			double areaThresh;

			MovementSpan() : active(false), timeFrameMs(500), moveThresh(10), areaThresh(0.3) {}
			MovementSpan(bool active) : MovementSpan() {
				this->active = active;
			}
		};
		MovementSpan movementSpan;
	};

private:
	Setup setup;
	Detector::Ptr detector;
    int consecutiveDetects;
    int consecutiveNonDetects;
    bool lastDetect;
    cv::Rect initialRegion;
    cv::Rect lastRegion;
    int moveDisplacementThreshold;
    double areaDisplacementThreshold;
    bool bigDisplacement;
    bool positionUnsteady_;

    struct DetectInfo {
    	cv::Rect region;
    	Poco::Timestamp time;

    	DetectInfo(const cv::Rect& region) : region(region) {}
    };
    std::vector<DetectInfo> detectHistory;

    cv::Point rectCenter(const cv::Rect &rect);
    cv::Rect getBiggestArea(const std::vector<cv::Rect> &regions);

public:
    Tracker() {}
    Tracker(Detector::Ptr detector);
    Tracker(const Setup& setup, Detector::Ptr detector);
    void init();
    cv::Rect detect(const ImageGrayscale &img);

    int getConsecutiveDetects() const { return consecutiveDetects; }
    int getConsecutiveNonDetects() const { return consecutiveNonDetects; }
    bool isBigDisplacement() const { return bigDisplacement; }
    bool positionUnsteady() const;
    bool isLastDetect() const { return lastDetect; }
    cv::Rect getLastRegion() const { return lastRegion; }
    cv::Rect getInitialRegion() const { return initialRegion; }
};

}
}

#endif // REALTIMETRACK_H
