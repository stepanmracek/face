/*
 * faceposeestimator.cpp
 *
 *  Created on: 25. 4. 2015
 *      Author: dron
 */

#include "faceposeestimator.h"
#include <numeric>

namespace Face {
namespace Sensors {
namespace RealSense {

const double FacePoseEstimator::DefaultAlignTolerance = 8.0;
const double FacePoseEstimator::DefaultRollZeroAngle = 0.0;
//const double FacePoseEstimator::DefaultPitchZeroAngle = 12.0; // camera above screen
const double FacePoseEstimator::DefaultPitchZeroAngle = 16.0; // camera below screen
const double FacePoseEstimator::DefaultYawZeroAngle = 3.0;

FacePoseEstimator::Line2d::Line2d(const cv::Point2d& from, const cv::Point2d& to) :
	from(from), to(to) {
}

FacePoseEstimator::Pose::Pose(
	double roll, double pitch, double yaw,
	const cv::Vec3d& worldAxisX, const cv::Vec3d& worldAxisY, const cv::Vec3d& worldAxisZ,
	const Line2d& imageAxisX, const Line2d& imageAxisY) :
	roll(roll), pitch(pitch), yaw(yaw),
	worldAxisX(worldAxisX), worldAxisY(worldAxisY), worldAxisZ(worldAxisZ),
	imageAxisX(imageAxisX), imageAxisY(imageAxisY),
	relativePitch(0.0), relativeYaw(0.0) {
}

FacePoseEstimator::Pose FacePoseEstimator::Pose::UnknownPose() {
	return Pose(
		nan(""), nan(""), nan(""),
		cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
		Line2d(cv::Point2d(0, 0), cv::Point2d(0, 0)), Line2d(cv::Point2d(0, 0), cv::Point2d(0, 0))
	);
}

bool operator ==(const FacePoseEstimator::Pose& lhs, const FacePoseEstimator::Pose& rhs) {
	return lhs.roll == rhs.roll && lhs.pitch == rhs.pitch && lhs.yaw == rhs.yaw;
}

bool operator !=(const FacePoseEstimator::Pose& lhs, const FacePoseEstimator::Pose& rhs) {
	return !(lhs == rhs);
}

FacePoseEstimator::Settings::Settings(double alignTolerance, double rollZeroAngle, double pitchZeroAngle, double yawZeroAngle)
	: alignTolerance(alignTolerance), 
	rollZeroAngle(rollZeroAngle),
	pitchZeroAngle(pitchZeroAngle),
	yawZeroAngle(yawZeroAngle) {
}

FacePoseEstimator::FacePoseEstimator(const Settings& settings) : settings(settings), lastAlignInstruction(AlignInstruction::UNKNOWN) {
}

FacePoseEstimator::~FacePoseEstimator() {
}

const FacePoseEstimator::Settings& FacePoseEstimator::getSettings() const {
	return settings;
}

void FacePoseEstimator::restart() {
	lEyeCenterAcc.clear();
	rEyeCenterAcc.clear();
	mouthCenterAcc.clear();
	lastAlignInstruction = AlignInstruction::UNKNOWN;
}

double toDegrees(double radians) {
	return radians * 180.0 / M_PI;
};

FacePoseEstimator::Pose FacePoseEstimator::facePose(const LandmarkDetector::LMPointsMap& lmMap, RealSenseProjection::Ptr rsProjection) {

	auto center = [=](const LandmarkDetector::Point2dVec& points) -> cv::Point3d {
		std::vector<cv::Point3d> points3d = rsProjection->colorToWorld(points, RealSenseProjection::Transformation::None);

		cv::Point3d center(0, 0, 0);
		int size = 0;
		for (const auto& point : points3d) {
			if (point.x == 0 && point.y == 0 && point.z == 0) {
				continue;
				//throw std::runtime_error("ignore");
			}
			center += point;
			size++;
		}

		if (size < points.size() * 0.6) {
			throw std::runtime_error("ignore");
		}

		center.x /= size;
		center.y /= size;
		center.z /= size;

		return center;
	};

	const std::size_t BufferSize = 4;

	auto accCenter = [=](const std::string& lmGroupName, std::deque<cv::Vec3d>& acc, cv::Vec3d& c) -> void {
		try {
			acc.push_back(center(lmMap.at(lmGroupName)));
		} catch (...) {}

		if (!acc.empty()) {
			c = std::accumulate(acc.begin(), acc.end(), cv::Vec3d(0, 0, 0)) * (1.0 / acc.size());
			while (acc.size() > BufferSize) {
				acc.pop_front();
			}
		}
	};

	cv::Vec3d leftEyeCenter(0, 0, 0);
	cv::Vec3d rightEyeCenter(0, 0, 0);
	cv::Vec3d mouthCenter(0, 0, 0);

	accCenter(LandmarkGroup::LeftEye, lEyeCenterAcc, leftEyeCenter);
	accCenter(LandmarkGroup::RightEye, rEyeCenterAcc, rightEyeCenter);
	accCenter(LandmarkGroup::Mouth, mouthCenterAcc, mouthCenter);

	if (lEyeCenterAcc.empty() || rEyeCenterAcc.empty() || mouthCenterAcc.empty()) {
		return Pose::UnknownPose();
	}

	std::vector<cv::Point3d> worldAxes = { leftEyeCenter, rightEyeCenter, mouthCenter, (leftEyeCenter + rightEyeCenter) * 0.5 };
	std::vector<cv::Point2d> imageAxes = rsProjection->worldToColor(worldAxes, RealSenseProjection::Transformation::None);

	cv::Vec3d X = cv::normalize(cv::Vec3d(worldAxes.at(1) - worldAxes.at(0)));
	cv::Vec3d Y = cv::normalize(cv::Vec3d(worldAxes.at(3) - worldAxes.at(2)));
	cv::Vec3d Z = cv::normalize(X.cross(Y));

	cv::Vec3d X_ = cv::normalize(cv::Vec3d(X[0], X[1], 0));
	double alpha_roll = X_[1] > 0 ? -acos(X_[0]) : acos(X_[0]);

	cv::Vec3d Y_ = cv::normalize(cv::Vec3d(0, Y[1], Y[2]));
	double beta_pitch = Y_[2] > 0 ? acos(Y_[1]) : -acos(Y_[1]);

	cv::Vec3d Z_ = cv::normalize(cv::Vec3d(Z[0], 0, Z[2]));
	double gamma_yaw = Z_[0] > 0 ? -acos(Z_[2]) : acos(Z_[2]);
	

	Pose pose(
		toDegrees(alpha_roll), toDegrees(beta_pitch), toDegrees(gamma_yaw),
		X, Y, Z,
		Line2d(imageAxes.at(0), imageAxes.at(1)), Line2d(imageAxes.at(2), imageAxes.at(3))
	);

	calcPose((leftEyeCenter + rightEyeCenter) * 0.5, pose.relativePitch, pose.relativeYaw);

	return pose;
}

FacePoseEstimator::AlignInstruction FacePoseEstimator::giveFaceAlignInstruction(const Pose& pose) {
	AlignInstruction instruction = AlignInstruction::UNKNOWN;
	if (pose != Pose::UnknownPose()) {
		
		// add some hysteresis
		double targetYawTolerance = settings.alignTolerance;
		double targetPitchTolerance = settings.alignTolerance;
		if (lastAlignInstruction == AlignInstruction::LOOK_LEFT || lastAlignInstruction == AlignInstruction::LOOK_RIGHT) {
			targetYawTolerance *= 0.8;
		}
		if (lastAlignInstruction == AlignInstruction::LOOK_UP || lastAlignInstruction == AlignInstruction::LOOK_DOWN) {
			targetPitchTolerance *= 0.8;
		}

		double totalYaw = pose.yaw - pose.relativeYaw;
		double totalPitch = pose.pitch - pose.relativePitch;

		if (fabs(totalYaw - settings.yawZeroAngle) > targetYawTolerance) {
			if (totalYaw < settings.yawZeroAngle) {
				instruction = AlignInstruction::LOOK_RIGHT;
			} else {
				instruction = AlignInstruction::LOOK_LEFT;
			}
		} else if (fabs(totalPitch - settings.pitchZeroAngle) > targetPitchTolerance) {
			if (totalPitch < settings.pitchZeroAngle) {
				instruction = AlignInstruction::LOOK_UP;
			} else {
				instruction = AlignInstruction::LOOK_DOWN;
			}
		} else {
			instruction = AlignInstruction::OK;
		}
	}

	lastAlignInstruction = instruction;
	return instruction;
}

std::string FacePoseEstimator::toString(AlignInstruction instruction) {
	switch (instruction) {
		case AlignInstruction::UNKNOWN: return "Unknown";
		case AlignInstruction::OK: return "OK";
		case AlignInstruction::LOOK_LEFT: return "Look left";
		case AlignInstruction::LOOK_RIGHT: return "Look right";
		case AlignInstruction::LOOK_UP: return "Look up";
		case AlignInstruction::LOOK_DOWN: return "Look down";
		case AlignInstruction::ROTATE_LEFT: return "Rotate left";
		case AlignInstruction::ROTATE_RIGHT: return "Rotate right";
	}

	return "Unknown";
}

void FacePoseEstimator::calcPose(const cv::Point3d& point, double& pitch, double& yaw) const {

	cv::Vec3d Y_ = cv::normalize(cv::Vec3d(0, point.y, point.z));
	pitch = toDegrees((Y_[2] > 0 ? acos(Y_[1]) : -acos(Y_[1])) - (M_PI_2));

	cv::Vec3d Z_ = cv::normalize(cv::Vec3d(point.x, 0, point.z));
	yaw = toDegrees(Z_[0] > 0 ? -acos(Z_[2]) : acos(Z_[2]));
}

}
}
}
