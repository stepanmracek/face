/*
 * FacePoseEstimator.h
 *
 *  Created on: 25. 4. 2015
 *      Author: dron
 */

#ifndef FACELIB_REALSENSE_SRC_FACEPOSEESTIMATOR_H_
#define FACELIB_REALSENSE_SRC_FACEPOSEESTIMATOR_H_

#include "realsense/realsense.h"
#include "faceCommon/linalg/common.h"
#include "landmarkdetector.h"
#include "realsenseprojection.h"

namespace Face {
namespace Sensors {
namespace RealSense {

class REALSENSE_EXPORTS FacePoseEstimator {
	public:
		struct Line2d {
			cv::Point2d from;
			cv::Point2d to;

			Line2d(const cv::Point2d& from, const cv::Point2d& to);
		};

		struct Pose {
			double roll;  // ~ head rotate (around nose) left/right
			double pitch; // ~ head look up/down
			double yaw;   // ~ head look left/right

			cv::Vec3d worldAxisX;
			cv::Vec3d worldAxisY;
			cv::Vec3d worldAxisZ;

			Line2d imageAxisX; // left -> right
			Line2d imageAxisY; // bottom -> up

			double relativePitch;
			double relativeYaw;

			Pose(
				double roll, double pitch, double yaw,
				const cv::Vec3d& worldAxisX, const cv::Vec3d& worldAxisY, const cv::Vec3d& worldAxisZ,
				const Line2d& imageAxisX, const Line2d& imageAxisY
			);

			static Pose UnknownPose();
		};

		static const double DefaultAlignTolerance;
		static const double DefaultRollZeroAngle;
		static const double DefaultPitchZeroAngle; ///< camera above screen
		static const double DefaultYawZeroAngle;

		struct Settings {
			double alignTolerance;
			double rollZeroAngle;
			double pitchZeroAngle;
			double yawZeroAngle;

			Settings(double alignTolerance, double rollZeroAngle, double pitchZeroAngle, double yawZeroAngle);
		};

		FacePoseEstimator(const Settings& settings);
		virtual ~FacePoseEstimator();

		const Settings& getSettings() const;

		void restart();
		Pose facePose(const LandmarkDetector::LMPointsMap& lmMap, RealSenseProjection::Ptr rsProjection);

		enum class AlignInstruction {
			UNKNOWN, OK, LOOK_UP, LOOK_DOWN, LOOK_LEFT, LOOK_RIGHT, ROTATE_LEFT, ROTATE_RIGHT
		};
		AlignInstruction giveFaceAlignInstruction(const Pose& pose);
		static std::string toString(AlignInstruction instruction);

	private:
		// in degrees
		void calcPose(const cv::Point3d& point, double& pitch, double& yaw) const;

	private:
		Settings settings;

		std::deque<cv::Vec3d> lEyeCenterAcc;
		std::deque<cv::Vec3d> rEyeCenterAcc;
		std::deque<cv::Vec3d> mouthCenterAcc;
		AlignInstruction lastAlignInstruction;
};

REALSENSE_EXPORTS bool operator ==(const FacePoseEstimator::Pose& lhs, const FacePoseEstimator::Pose& rhs);
REALSENSE_EXPORTS bool operator !=(const FacePoseEstimator::Pose& lhs, const FacePoseEstimator::Pose& rhs);

}
}
}



#endif /* FACELIB_REALSENSE_SRC_FACEPOSEESTIMATOR_H_ */
