/*
 * realsensedetector.h
 *
 *  Created on: 21. 6. 2015
 *      Author: dron
 */

#ifndef REALSENSEDETECTOR_H_
#define REALSENSEDETECTOR_H_

#include <Poco/Thread.h>
#include <Poco/Runnable.h>
#include <Poco/Event.h>
#include <Poco/Mutex.h>
#include <vector>
#include "faceCommon/objectdetection/detector.h"
#include "realsensedepthfacetracker.h"

namespace Face {
namespace Sensors {
namespace RealSense {

	class RealSenseDetector : public ObjectDetection::OpenCVDetector, public Poco::Runnable {
		public:
			struct Setup {
				std::string cascadeClassifierPath;
				double scale;
				cv::Rect detectRoi;
			};

			RealSenseDetector(const Setup& setup, RealSenseDepthFaceTracker::Ptr rsDepthFaceTracker);
			virtual ~RealSenseDetector();

			virtual std::vector<cv::Rect> detect(const ImageGrayscale &img);

		private:
			void run();

			std::vector<cv::Rect> detectImpl(const ImageGrayscale &img);

		private:
			Setup setup;
			RealSenseDepthFaceTracker::Ptr rsDepthFaceTracker;

			cv::Rect lastRoi;
			std::size_t consecutiveDetects;
			std::size_t consecutiveNonDetects;

			std::vector<ImageGrayscale> imageVector;
			Poco::Mutex imMutex;
			std::vector<cv::Rect> rects;
			Poco::Mutex rectMutex;
			Poco::Thread thread;
			Poco::Event wakeEvent;
			bool stopped;
	};

} /* namespace RealSense */
} /* namespace Sensors */
} /* namespace Face */

#endif /* 3D_FACE_FACELIB_REALSENSE_SRC_REALSENSEDETECTOR_H_ */
