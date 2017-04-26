#include "landmarkdetector.h"

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/image_processing/full_object_detection.h>
#include <dlib/opencv.h>
#include <numeric>

using namespace Face::Sensors::RealSense;

const unsigned long LANDMARK_BEGIN = 27;
const unsigned long LANDMARK_END = 48;


LandmarkDetector::LandmarkDetector(const std::string &pathToLandmarkModel)
{
	shapePredictor = std::make_shared<dlib::shape_predictor>();
	dlib::deserialize(pathToLandmarkModel) >> *shapePredictor;
}

LandmarkDetector::~LandmarkDetector()
{
}

LandmarkDetector::Point2dVec LandmarkDetector::getLandmarks(const ImageGrayscale &img, const cv::Rect &roi, bool filter)
{
	Point2dVec lm;

	dlib::cv_image<unsigned char> dimg(img);
	dlib::rectangle rect(roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);
	auto detectionResult = (*shapePredictor)(dimg, rect);
	//auto detectionResult = shapePredictor->operator()(dimg, rect);

	auto n = detectionResult.num_parts();
	decltype(n) begin = 0;
	decltype(n) end = n;
	if (filter) {
		begin = LANDMARK_BEGIN;
		end = LANDMARK_END;
	}
	for (decltype(n) i = begin; i < end; i++) {
		auto point = detectionResult.part(i);
		lm.push_back(cv::Point2d(point.x(), point.y()));
	}

	if (lm.empty()) std::cout << "WARNING: no landmarks found" << std::endl;

	return lm;
}

LandmarkDetector::LMPointsMap LandmarkDetector::lmGroupPoints(const Point2dVec& lmCoords) const {
	if (lmCoords.size() != 68) {
		std::cerr << "Wrong number of landmarks" << std::endl;
		throw FACELIB_EXCEPTION("Wrong number of landmarks");
	}

	auto idxs2points = [](const std::vector<cv::Point2d>& landmarks, std::vector<std::size_t> indexes) -> std::vector<cv::Point2d> {
		std::vector<cv::Point2d> points;
		for (const auto& index : indexes) {
			points.push_back(landmarks[index]);
		}
		return points;
	};

	std::vector<std::size_t> leftEyeBrowIndexes = { 17, 18, 19, 20, 21 };
	std::vector<std::size_t> rightEyeBrowIndexes = { 22, 23, 24, 25, 26 };
	std::vector<std::size_t> bothEyeBrowsIndexes;
	bothEyeBrowsIndexes.reserve(leftEyeBrowIndexes.size() + rightEyeBrowIndexes.size());
	bothEyeBrowsIndexes.insert(bothEyeBrowsIndexes.end(), leftEyeBrowIndexes.begin(), leftEyeBrowIndexes.end());
	bothEyeBrowsIndexes.insert(bothEyeBrowsIndexes.end(), rightEyeBrowIndexes.begin(), rightEyeBrowIndexes.end());
	std::vector<std::size_t> leftEyeIndexes = { 36, 37, 38, 39, 40, 41 };
	std::vector<std::size_t> rightEyeIndexes = { 42, 43, 44, 45, 46, 47 };
	std::vector<std::size_t> mouthIndexes = { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67 };
	std::vector<std::size_t> mouthCornerIndexes = { 48, 54 };
	std::vector<std::size_t> underNoseIndexes = { 31, 32, 33, 34, 35 };

	std::map<std::string, std::vector<cv::Point2d> > lmGroups;
	lmGroups[LandmarkGroup::LeftEyeBrow] = idxs2points(lmCoords, leftEyeBrowIndexes);
	lmGroups[LandmarkGroup::RightEyeBrow] = idxs2points(lmCoords, rightEyeBrowIndexes);
	lmGroups[LandmarkGroup::BothEyeBrows] = idxs2points(lmCoords, bothEyeBrowsIndexes);
	lmGroups[LandmarkGroup::LeftEye] = idxs2points(lmCoords, leftEyeIndexes);
	lmGroups[LandmarkGroup::RightEye] = idxs2points(lmCoords, rightEyeIndexes);
	lmGroups[LandmarkGroup::Mouth] = idxs2points(lmCoords, mouthIndexes);
	lmGroups[LandmarkGroup::MouthCorners] = idxs2points(lmCoords, mouthCornerIndexes);
	lmGroups[LandmarkGroup::UnderNose] = idxs2points(lmCoords, underNoseIndexes);

	return lmGroups;
}

