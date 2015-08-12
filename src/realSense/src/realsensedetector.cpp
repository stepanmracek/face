/*
 * realsensedetector.cpp
 *
 *  Created on: 21. 6. 2015
 *      Author: dron
 */

#include "realsensedetector.h"

namespace Face {
namespace Sensors {
namespace RealSense {

	RealSenseDetector::RealSenseDetector(const Setup& setup, RealSenseDepthFaceTracker::Ptr rsDepthFaceTracker)
		: OpenCVDetector(setup.cascadeClassifierPath), setup(setup), rsDepthFaceTracker(rsDepthFaceTracker), consecutiveDetects(0), consecutiveNonDetects(0),
		  thread("Face.RSDetector") {
		stopped = false;
		thread.start(*this);
	}

	RealSenseDetector::~RealSenseDetector() {
		stopped = true;
		wakeEvent.set();
		thread.join();
	}

	cv::Rect getBiggest(const std::vector<cv::Rect>& rects) {
		if (rects.empty()) {
			return cv::Rect();
		}

		cv::Rect biggest = rects.front();
		for (const auto& r: rects) {
			if (r.area() > biggest.area()) {
				biggest = r;
			}
		}

		return biggest;
	}

	std::vector<cv::Rect> RealSenseDetector::detect(const ImageGrayscale &img) {
		{
			Poco::Mutex::ScopedLock lock(imMutex);
			imageVector.push_back(img);
		}

		std::vector<cv::Rect> _rects;
		{
			Poco::Mutex::ScopedLock lock(rectMutex);
			_rects = this->rects;
		}

		wakeEvent.set();
		return _rects;
	}

	std::vector<cv::Rect> RealSenseDetector::detectImpl(const ImageGrayscale &img) {
		std::vector<cv::Rect> rects = OpenCVDetector::detect(img);

		if (rects.empty()) {
			consecutiveDetects = 0;
			consecutiveNonDetects++;

			// depth face tracker overlap test
			if (consecutiveNonDetects < 4 && rsDepthFaceTracker->hasLastRoi()) {
				cv::Rect croi = RealSenseDepthFaceTracker::convertScale(rsDepthFaceTracker->getLastCroi(), setup.scale, setup.detectRoi);
				cv::Rect intersection = lastRoi & croi;
				cv::Rect smallerRoi = croi.area() < lastRoi.area() ? croi : lastRoi;

				std::cout << std::dec << " *** RealSenseDetector::detectImpl, lastRoi: " << lastRoi << ", croi: " << croi << ", intersection: " << intersection << std::endl;
				std::cout << std::dec << " *** RealSenseDetector::detectImpl, intersection/smallerRoi/croi/lastRoi areas: " <<
						intersection.area() << "/" << smallerRoi.area() << "/" << croi.area() << "/" << lastRoi.area() << std::endl;

				if (intersection.area() > smallerRoi.area() * 0.5) {
					cv::Point croiCenter(croi.x + croi.width / 2, croi.y + croi.height / 2);
					cv::Point lregCenter(lastRoi.x + lastRoi.width / 2, lastRoi.y + lastRoi.height / 2);
					cv::Point displ(croiCenter - lregCenter);
					std::cout << std::dec << " *** RealSenseDetector::detectImpl, displ vector: " << displ << std::endl;
					lastRoi.x += displ.x;
					lastRoi.y += displ.y;
					rects.push_back(lastRoi);
				}
			} else {
				std::cout << std::dec << " *** RealSenseDetector::detectImpl, NO FACE, consecutiveNonDetects: " << consecutiveNonDetects << std::endl;
			}
		} else {
			consecutiveDetects++;
			consecutiveNonDetects = 0;
			lastRoi = getBiggest(rects);
		}

		return rects;
	}

	void RealSenseDetector::run() {
		while (!stopped){

			ImageGrayscale img;
			{
				Poco::Mutex::ScopedLock lock(imMutex);
				if (imageVector.size()) {
					img = imageVector.back();
					imageVector.clear();
				}
			}

			if (img.data == 0) {
				wakeEvent.wait();
				continue;
			}

			std::vector<cv::Rect> _rects = detectImpl(img);
			{
				Poco::Mutex::ScopedLock lock(rectMutex);
				this->rects = _rects;
			}
		}

	}

} /* namespace RealSense */
} /* namespace Sensors */
} /* namespace Face */
