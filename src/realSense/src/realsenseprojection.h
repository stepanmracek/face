/*
 * realsenseprojection.h
 *
 *  Created on: 25. 4. 2015
 *      Author: dron
 */

#ifndef FACELIB_REALSENSEPROJECTION_H_
#define FACELIB_REALSENSEPROJECTION_H_

#include <pxcsensemanager.h>
#include <pxcprojection.h>
#include <opencv2/opencv.hpp>
#include <Poco/SharedPtr.h>

namespace Face {
	namespace Sensors {
		namespace RealSense {

			class RealSenseProjection {
			public:
				typedef Poco::SharedPtr<RealSenseProjection> Ptr;

				RealSenseProjection(PXCSenseManager* senseManager, PXCProjection* projection);
				virtual ~RealSenseProjection();

				static void convert(const std::vector<cv::Point2d>& cvPoints, std::vector<PXCPointF32>& rsPoints);
				static void convert(const std::vector<PXCPointF32>& rsPoints, std::vector<cv::Point2d>& cvPoints);

				static void convert(const std::vector<PXCPoint3DF32>& rsPoints3d, std::vector<cv::Point3d>& cvPoints3d);
				static void convert(const std::vector<cv::Point3d>& cvPoints3d, std::vector<PXCPoint3DF32>& rsPoints3d);

				enum class Transformation { None, OpenGL };
				std::vector<cv::Point3d> colorToWorld(const std::vector<cv::Point2d>& ccords, Transformation transformation) const;
				std::vector<cv::Point3d> colorToWorld(PXCImage* depth, const std::vector<cv::Point2d>& ccords, Transformation transformation) const;
				std::vector<cv::Point2d> worldToColor(const std::vector<cv::Point3d>& wcords, Transformation transformation) const;

				void depthToColor(PXCImage *depth, const std::vector<PXCPointF32>& dcords, std::vector<PXCPointF32>& ccords) const;
				void colorToDepth(PXCImage *depth, const std::vector<PXCPointF32>& ccords, std::vector<PXCPointF32>& dcords) const;

			private:
				void worldToDepth(PXCImage *depth, const std::vector<PXCPoint3DF32>& wcords3d, std::vector<PXCPointF32>& dcords) const;
				void worldToColorImpl(PXCImage* depth, const std::vector<PXCPoint3DF32>& wcords3d, std::vector<PXCPointF32>& ccords) const;
				
				void depthToWorld(PXCImage *depth, const std::vector<PXCPointF32>& dcords, std::vector<PXCPoint3DF32>& wcords3d) const;
				void colorToWorldImpl(PXCImage *depth, const std::vector<PXCPointF32>& ccords, std::vector<PXCPoint3DF32>& wcords3d) const;

				std::vector<cv::Point3d> openglTransform(const std::vector<cv::Point3d>& rsPoints) const;

			private:
				PXCSenseManager* senseManager;
				PXCProjection* projection;
			};

		}
	}
}


#endif /* FACELIB_REALSENSEPROJECTION_H_ */
