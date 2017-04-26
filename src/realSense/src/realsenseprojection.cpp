/*
 * realsenseprojection.cpp
 *
 *  Created on: 25. 4. 2015
 *      Author: dron
 */

#include "realsenseprojection.h"
#include "faceCommon/linalg/common.h"

namespace Face {
	namespace Sensors {
		namespace RealSense {

			RealSenseProjection::RealSenseProjection(PXCSenseManager* senseManager, PXCProjection* projection)
				: senseManager(senseManager), projection(projection) {
				if (senseManager == 0) {
					throw FACELIB_EXCEPTION("Zero PXCSenseManager pointer provided");
				}
				if (projection == 0) {
					throw FACELIB_EXCEPTION("Zero PXCProjection pointer provided");
				}
			}
			RealSenseProjection::~RealSenseProjection() {}

			void RealSenseProjection::convert(const std::vector<cv::Point2d>& cvPoints, std::vector<PXCPointF32>& rsPoints) {
				PXCPointF32 zeroP = { 0.f, 0.f };
				rsPoints.resize(cvPoints.size(), zeroP);

				int size = cvPoints.size();
				for (int i = 0; i < size; ++i) {
					rsPoints[i].x = static_cast<float>(cvPoints[i].x);
					rsPoints[i].y = static_cast<float>(cvPoints[i].y);
				}
			}

			void RealSenseProjection::convert(const std::vector<PXCPointF32>& rsPoints, std::vector<cv::Point2d>& cvPoints) {
				cv::Point2d zeroP(0.0, 0.0);
				cvPoints.resize(rsPoints.size(), zeroP);

				int size = rsPoints.size();
				for (int i = 0; i < size; ++i) {
					cvPoints[i].x = rsPoints[i].x;
					cvPoints[i].y = rsPoints[i].y;
				}
			}

			void RealSenseProjection::convert(const std::vector<PXCPoint3DF32>& rsPoints3d, std::vector<cv::Point3d>& cvPoints3d) {
				cv::Point3d zeroP(0.0, 0.0, 0.0);
				cvPoints3d.resize(rsPoints3d.size(), zeroP);

				int size = rsPoints3d.size();
				for (int i = 0; i < size; ++i) {
					cvPoints3d[i].x = rsPoints3d[i].x;
					cvPoints3d[i].y = rsPoints3d[i].y;
					cvPoints3d[i].z = rsPoints3d[i].z;
				}
			}

			void RealSenseProjection::convert(const std::vector<cv::Point3d>& cvPoints3d, std::vector<PXCPoint3DF32>& rsPoints3d) {
				PXCPoint3DF32 zeroP = { 0.f, 0.f, 0.f };
				rsPoints3d.resize(cvPoints3d.size(), zeroP);

				int size = cvPoints3d.size();
				for (int i = 0; i < size; ++i) {
					rsPoints3d[i].x = static_cast<float>(cvPoints3d[i].x);
					rsPoints3d[i].y = static_cast<float>(cvPoints3d[i].y);
					rsPoints3d[i].z = static_cast<float>(cvPoints3d[i].z);
				}
			}

			void RealSenseProjection::worldToDepth(PXCImage *depth, const std::vector<PXCPoint3DF32>& wcords3d, std::vector<PXCPointF32>& dcords) const {
				int wsize = wcords3d.size();
				dcords.resize(wsize);

				projection->ProjectCameraToDepth(wsize, &const_cast<std::vector<PXCPoint3DF32>&>(wcords3d)[0], &dcords[0]);
			}

			void RealSenseProjection::depthToColor(PXCImage *depth, const std::vector<PXCPointF32>& dcords, std::vector<PXCPointF32>& ccords) const {
				if (!depth) {
					throw FACELIB_EXCEPTION("Null depth image");
				}

				PXCImage::ImageData ddata;
				if (PXC_STATUS_NO_ERROR > depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &ddata)) {
					throw FACELIB_EXCEPTION("Cannot acquire access to depth image -> did you release frame already?");
				}

				int dsize = dcords.size();
				PXCPoint3DF32 invP3D = { -1.f, -1.f, 0.f };
				std::vector<PXCPoint3DF32> dcords3d(dsize, invP3D);
				PXCPointF32 invP = { -1.f, -1.f };
				ccords.resize(dsize, invP);

				for (int i = 0; i < dsize; i++) {
					if (dcords[i].x < 0) {
						continue;
					}
					dcords3d[i].x = dcords[i].x;
					dcords3d[i].y = dcords[i].y;
					dcords3d[i].z = (float)(((pxcI16*)(ddata.planes[0] + (int)dcords3d[i].y * ddata.pitches[0]))[(int)dcords3d[i].x]);
				}
				depth->ReleaseAccess(&ddata);

				projection->MapDepthToColor(dsize, &dcords3d[0], &ccords[0]);
			}

			void RealSenseProjection::worldToColorImpl(PXCImage* depth, const std::vector<PXCPoint3DF32>& wcords3d, std::vector<PXCPointF32>& ccords) const {
				//				std::vector<PXCPointF32> dcords;
				//				worldToDepth(depth, wcords3d, dcords);
				//				depthToColor(depth, dcords, ccords);

				int wsize = wcords3d.size();
				ccords.resize(wsize);

				std::vector<PXCPoint3DF32> _wcords3d(wcords3d);
				for (int i = 0; i < wsize; i++) {
					_wcords3d[i].x -= 55;
				}
				projection->ProjectCameraToColor(wsize, &_wcords3d[0], &ccords[0]);

			}

			std::vector<cv::Point2d> RealSenseProjection::worldToColor(const std::vector<cv::Point3d>& wcords, Transformation transformation) const {
				std::vector<PXCPoint3DF32> rsWcords3d;

				if (transformation == Transformation::OpenGL) {
					convert(openglTransform(wcords), rsWcords3d);
				}
				else {
					convert(wcords, rsWcords3d);
				}

				PXCCapture::Sample* sample = senseManager->QuerySample();
				std::vector<PXCPointF32> rsCcords;
				worldToColorImpl(sample->depth, rsWcords3d, rsCcords);

				std::vector<cv::Point2d> ccords;
				convert(rsCcords, ccords);

				return ccords;
			}

			void RealSenseProjection::colorToDepth(PXCImage *depth, const std::vector<PXCPointF32>& ccords, std::vector<PXCPointF32> &dcords) const {
				int csize = ccords.size();
				dcords.resize(csize);

				projection->MapColorToDepth(depth, csize, &const_cast<std::vector<PXCPointF32>&>(ccords)[0], &dcords[0]);
			}

			void RealSenseProjection::depthToWorld(PXCImage *depth, const std::vector<PXCPointF32>& dcords, std::vector<PXCPoint3DF32> &wcords3d) const {
				PXCImage::ImageData ddata;

				if (PXC_STATUS_NO_ERROR > depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &ddata)) {
					throw FACELIB_EXCEPTION("Cannot acquire access to depth image -> did you release frame already?");
				}

				int dsize = dcords.size();
				PXCPoint3DF32 invP3D = { -1.f, -1.f, 0.f };
				std::vector<PXCPoint3DF32> dcords3d(dsize, invP3D);
				wcords3d.resize(dsize, invP3D);

				for (int i = 0; i < dsize; i++) {
					if (dcords[i].x < 0) {
						continue;
					}
					dcords3d[i].x = dcords[i].x;
					dcords3d[i].y = dcords[i].y;
					dcords3d[i].z = (float)(((pxcI16*)(ddata.planes[0] + (int)dcords3d[i].y * ddata.pitches[0]))[(int)dcords3d[i].x]);
				}
				depth->ReleaseAccess(&ddata);

				projection->ProjectDepthToCamera(dsize, &dcords3d[0], &wcords3d[0]);

				for (int i = 0; i < dsize; i++) {
					if (dcords3d[i].z <= 0) {
						wcords3d[i].x = wcords3d[i].y = wcords3d[i].z = 0.0;
					}
				}
			}

			void RealSenseProjection::colorToWorldImpl(PXCImage* depth, const std::vector<PXCPointF32>& ccords, std::vector<PXCPoint3DF32>& wcords3d) const {
				std::vector<PXCPointF32> dcords;
				colorToDepth(depth, ccords, dcords);
				depthToWorld(depth, dcords, wcords3d);
			}

			std::vector<cv::Point3d> RealSenseProjection::colorToWorld(const std::vector<cv::Point2d>& ccords, Transformation transformation) const {
				PXCCapture::Sample* sample = senseManager->QuerySample();

				if (sample == 0) {
					return std::vector<cv::Point3d>();
				}

				return this->colorToWorld(sample->depth, ccords, transformation);
			}

			std::vector<cv::Point3d> RealSenseProjection::colorToWorld(PXCImage* depth, const std::vector<cv::Point2d>& ccords, Transformation transformation) const {
				PXCCapture::Sample* sample = senseManager->QuerySample();

				std::vector<PXCPointF32> rsCcords;
				convert(ccords, rsCcords);
				std::vector<PXCPoint3DF32> rsWcords3d;

				colorToWorldImpl(depth, rsCcords, rsWcords3d);
				std::vector<cv::Point3d> wcords3d;
				convert(rsWcords3d, wcords3d);

				if (transformation == Transformation::OpenGL) {
					return openglTransform(wcords3d);
				}

				return wcords3d;
			}

			std::vector<cv::Point3d> RealSenseProjection::openglTransform(const std::vector<cv::Point3d>& rsPoints) const {
				std::vector<cv::Point3d> facelibPoints;
				for (const auto& point : rsPoints) {
					auto transformed = point;
					transformed.x *= -1.0;
					transformed.z *= -1.0;
					facelibPoints.push_back(transformed);
				}

				return facelibPoints;
			}

		}
	}
}



