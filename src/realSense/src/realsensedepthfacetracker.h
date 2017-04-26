/*
 * realsensedepthfacetracker.h
 *
 *  Created on: 21. 6. 2015
 *      Author: dron
 */

#ifndef REALSENSEDEPTHFACETRACKER_H_
#define REALSENSEDEPTHFACETRACKER_H_

#include <opencv2/opencv.hpp>
#include <pxcprojection.h>
#include <Poco/SharedPtr.h>

#include "faceCommon/linalg/common.h"
#include "realsenseprojection.h"

namespace Face {
namespace Sensors {
namespace RealSense {

	class RealSenseDepthFaceTracker {
		public:
			typedef Poco::SharedPtr<RealSenseDepthFaceTracker> Ptr;

			RealSenseDepthFaceTracker(RealSenseProjection::Ptr realsenseProjection);
			virtual ~RealSenseDepthFaceTracker();

			enum class Domain {
				Depth, Color
			};
			cv::Rect detectFaceRoi(const Matrix& depthMatrix, PXCImage* depth, Domain domain, bool debug = false);

			bool hasLastRoi() const;
			cv::Rect getLastDroi() const;
			cv::Rect getLastCroi() const;

			static cv::Rect convertScale(const cv::Rect& what, double scale, const cv::Rect& roi);

		private:
			struct BBoxSidePoints {
				cv::Point top;
				cv::Point bottom;
				cv::Point left;
				cv::Point right;

				void check() const {
					std::vector<cv::Point> vec = { top, bottom, left, right };
					for (auto p : vec) {
						if (p.x < 0 || p.y < 0) {
							throw FACELIB_EXCEPTION("BBox inconsistent");
						}
					}
				}

				cv::Rect toCvRect() const {
					check();
					return cv::Rect(left.x, top.y, right.x - left.x, bottom.y - top.y);
				}
			};

			PXCPointF32 toPXCPoint(const cv::Point& p);
			cv::Point toCvPoint(const PXCPointF32& p);

			BBoxSidePoints depthBBoxToColor(PXCImage *depth, const BBoxSidePoints& dbbox);

			std::vector<cv::Point> findTopContour(const std::vector<std::vector<cv::Point> >& contours, std::size_t minSize);

			BBoxSidePoints getRectSidePoints(const std::vector<cv::Point>& contour);

		private:
			RealSenseProjection::Ptr realsenseProjection;

			cv::Rect lastDroi;
			cv::Rect lastCroi;
	};

} /* namespace RealSense */
} /* namespace Sensors */
} /* namespace Face */

#endif /* 3D_FACE_FACELIB_REALSENSE_SRC_REALSENSEDEPTHFACETRACKER_H_ */
