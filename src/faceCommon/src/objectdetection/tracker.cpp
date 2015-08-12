#include "faceCommon/objectdetection/tracker.h"

using namespace Face::ObjectDetection;

Tracker::Tracker(Detector::Ptr detector) : detector(detector)
{
    init();
}

Tracker::Tracker(const Setup& setup, Detector::Ptr detector)
		: Tracker(detector) {
	this->setup = setup;
}

void Tracker::init()
{
    lastDetect = false;
    consecutiveDetects = 0;
    consecutiveNonDetects = 0;
    initialRegion = cv::Rect();
    lastRegion = cv::Rect();
    moveDisplacementThreshold = 7;
    areaDisplacementThreshold = 0.2;
    bigDisplacement = false;
    positionUnsteady_ = false;
}

bool Tracker::positionUnsteady() const {
	return positionUnsteady_;
}

cv::Point center(const cv::Rect &rect) {
    return cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

cv::Point Tracker::rectCenter(const cv::Rect &rect) {
	return center(rect);
}

cv::Rect Tracker::getBiggestArea(const std::vector<cv::Rect> &regions)
{
    if (regions.size() == 0) return cv::Rect();

    int biggest = INT_MIN;
    int biggestIndex = -1;
    for (size_t i = 0; i < regions.size(); ++i)
    {
        int area = regions[i].area();
        if (area > biggest)
        {
            biggest = area;
            biggestIndex = i;
        }
    }
    return regions[biggestIndex];
}

void displacement(const cv::Rect& initial, const cv::Rect& last, cv::Point& centerDiff, double& areaRatio) {
    cv::Point centerInitial = center(initial);
    cv::Point centerLast = center(last);
    centerDiff = centerInitial - centerLast;
    areaRatio = ((double)(initial.area())) / last.area();
}

cv::Rect Tracker::detect(const ImageGrayscale &img)
{
    std::vector<cv::Rect> regions = detector->detect(img);
    bigDisplacement = false;
    positionUnsteady_ = false;

    if (regions.size() == 0)
    {
        // Object not detected
        consecutiveDetects = 0;
        consecutiveNonDetects++;
        initialRegion = cv::Rect();
        lastRegion = cv::Rect();
        lastDetect = false;
        detectHistory.clear();
    }
    else
    {
        lastRegion = getBiggestArea(regions);
        detectHistory.push_back(lastRegion);
        lastDetect = true;
        consecutiveDetects++;
        consecutiveNonDetects = 0;

        if (consecutiveDetects == 1)
        {
            // Detected first object in row
            initialRegion = lastRegion;
        }
        else
        {
            // Calculate difference between first and last detected object
            cv::Point centerDiff;
            double areaRatio = 1.0;
            displacement(initialRegion, lastRegion, centerDiff, areaRatio);

            if (abs(centerDiff.x) > moveDisplacementThreshold || abs(centerDiff.y) > moveDisplacementThreshold)
            {
                //std::cout << std::dec << " --> Big displacement due to MOVE diff: " << centerDiff << std::endl;
            	bigDisplacement = true;
            }

            if (fabs(1.0 - areaRatio) > areaDisplacementThreshold)
            {
                //std::cout << std::dec << " --> Big displacement due to AREA ratio: " << areaRatio << std::endl;
                bigDisplacement = true;
            }

            if (bigDisplacement)
            {
                initialRegion = lastRegion;
            }

            std::cout << std::boolalpha << " --> movement span ACTIVE: " << setup.movementSpan.active << std::endl;

            if (setup.movementSpan.active) {
            	DetectInfo last = detectHistory.back();
            	DetectInfo init = last;
            	for (auto it = detectHistory.rbegin(); it != detectHistory.rend(); ++it) {
            		if ((last.time - it->time) > Poco::Timestamp::TimeDiff(1000) * setup.movementSpan.timeFrameMs) {
            			init = *it;
                        std::cout << std::dec << " --> movement span time frame: " << (last.time - it->time) / 1000 << std::endl;
            			break;
            		}
            	}

            	if (init.time != last.time) {
                    cv::Point centerDiff;
                    double areaRatio = 1.0;
                    displacement(init.region, last.region, centerDiff, areaRatio);

                    if (abs(centerDiff.x) > setup.movementSpan.moveThresh || abs(centerDiff.y) > setup.movementSpan.moveThresh) {
                        std::cout << std::dec << " --> Big span due to MOVE diff: " << centerDiff << std::endl;
                        positionUnsteady_ = true;
                    }

                    if (fabs(1.0 - areaRatio) > setup.movementSpan.areaThresh) {
                        std::cout << std::dec << " --> Big span due to AREA ratio: " << areaRatio << std::endl;
                        positionUnsteady_ = true;
                    }
            	}

            }
        }
    }

    return lastRegion;
}

