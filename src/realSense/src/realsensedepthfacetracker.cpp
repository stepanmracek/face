/*
 * realsensedepthfacetracker.cpp
 *
 *  Created on: 21. 6. 2015
 *      Author: dron
 */

#include "realsensedepthfacetracker.h"
#include "faceCommon/linalg/common.h"

namespace Face {
	namespace Sensors {
		namespace RealSense {

			RealSenseDepthFaceTracker::RealSenseDepthFaceTracker(RealSenseProjection::Ptr realsenseProjection)
				: realsenseProjection(realsenseProjection) {
			}

			RealSenseDepthFaceTracker::~RealSenseDepthFaceTracker() {
			}

			cv::Rect RealSenseDepthFaceTracker::detectFaceRoi(const Matrix& depthMatrix, PXCImage* depth, Domain domain, bool debug) {
				lastCroi = cv::Rect();
				lastDroi = cv::Rect();

				ImageGrayscale d(depthMatrix.size(), CV_8UC1);
				depthMatrix.convertTo(d, CV_8UC1);
				ImageGrayscale threshed = d.clone();
				cv::threshold(d, threshed, 10, 255, cv::THRESH_BINARY);

				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(threshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

				std::vector<cv::Point> topC = findTopContour(contours, 50);
				BBoxSidePoints dbbox = getRectSidePoints(topC);
				lastDroi = dbbox.toCvRect();
				try {
					lastCroi = depthBBoxToColor(depth, dbbox).toCvRect();
				} catch (std::exception&) {}

				if (debug) {
					std::vector<std::vector<cv::Point> > contours2 = { topC };

					/// Draw contours
					cv::Mat drawing = cv::Mat::zeros(threshed.size(), CV_8UC1);
					for (int i = 0; i< contours2.size(); i++) {
						cv::Scalar color = cv::Scalar((100 + i * 10) % 256);
						cv::drawContours(drawing, contours2, i, color, 2, 8, hierarchy, 0, cv::Point());
					}

					cv::rectangle(drawing, lastDroi, cv::Scalar(255));
					cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
					cv::imshow("Contours", drawing);
				}

				if (domain == Domain::Color) {
					return lastCroi;
				}

				return lastDroi;
			}

			bool RealSenseDepthFaceTracker::hasLastRoi() const {
				return lastDroi.width > 0 && lastDroi.height > 0;
			}
			cv::Rect RealSenseDepthFaceTracker::getLastDroi() const {
				return lastDroi;
			}
			cv::Rect RealSenseDepthFaceTracker::getLastCroi() const {
				return lastCroi;
			}

			cv::Rect RealSenseDepthFaceTracker::convertScale(const cv::Rect& what, double scale, const cv::Rect& roi) {
				cv::Rect result(what);

				result.x = lrint(result.x * scale);
				result.y = lrint(result.y * scale);
				result.width = lrint(result.width * scale);
				result.height = lrint(result.height * scale);

				result -= roi.tl();

				return result & roi;
			}


			PXCPointF32 RealSenseDepthFaceTracker::toPXCPoint(const cv::Point& p) {
				PXCPointF32 p2;
				p2.x = p.x;
				p2.y = p.y;
				return p2;
			}

			cv::Point RealSenseDepthFaceTracker::toCvPoint(const PXCPointF32& p) {
				return cv::Point(p.x, p.y);
			}

			RealSenseDepthFaceTracker::BBoxSidePoints RealSenseDepthFaceTracker::depthBBoxToColor(PXCImage *depth, const BBoxSidePoints& dbbox) {
				std::vector<PXCPointF32> dvec = {
					toPXCPoint(dbbox.top),
					toPXCPoint(dbbox.bottom),
					toPXCPoint(dbbox.left),
					toPXCPoint(dbbox.right),
				};

				std::vector<PXCPointF32> cvec;
				realsenseProjection->depthToColor(depth, dvec, cvec);

				BBoxSidePoints cbbox;
				cbbox.top = toCvPoint(cvec.at(0));
				cbbox.bottom = toCvPoint(cvec.at(1));
				cbbox.left = toCvPoint(cvec.at(2));
				cbbox.right = toCvPoint(cvec.at(3));
				
				return cbbox;
			}

			std::vector<cv::Point> RealSenseDepthFaceTracker::findTopContour(const std::vector<std::vector<cv::Point> >& contours, std::size_t minSize) {
				if (contours.empty()) {
					return std::vector<cv::Point>();
				}

				std::vector<cv::Point>& topC = const_cast<std::vector<cv::Point>&>(contours.front());
				cv::Point topCTopPoint(-1, -1);

				for (const auto& c : contours) {
					if (c.size() < minSize) {
						continue;
					}

					auto topPoint = c.front();
					for (const auto& p : c) {
						if (p.y < topPoint.y) {
							topPoint = p;
						}
					}

					if (topCTopPoint.y < 0 || topPoint.y < topCTopPoint.y) {
						topC = c;
						topCTopPoint = topPoint;
					}
				}

				if (topC.size() < minSize) {
					return std::vector<cv::Point>();
				}

				return topC;
			}

			RealSenseDepthFaceTracker::BBoxSidePoints RealSenseDepthFaceTracker::getRectSidePoints(const std::vector<cv::Point>& contour) {
				if (contour.size() < 4) {
					throw FACELIB_EXCEPTION("Insufficient contour size");
				}

				BBoxSidePoints sidePoints;
				sidePoints.top = sidePoints.bottom = sidePoints.left = sidePoints.right = contour.front();
				for (const auto& p : contour) {
					if (p.y < sidePoints.top.y) {
						sidePoints.top = p;
					}
					if (p.y > sidePoints.bottom.y) {
						sidePoints.bottom = p;
					}
					if (p.x < sidePoints.left.x) {
						sidePoints.left = p;
					}
					if (p.x > sidePoints.right.x) {
						sidePoints.right = p;
					}
				}

				return sidePoints;
			}

		} /* namespace RealSense */
	} /* namespace Sensors */
} /* namespace Face */
