#ifndef REALSENSE_LMDETECTOR
#define REALSENSE_LMDETECTOR

#include <string>
#include <memory>
#include <deque>

#include "realsense/realsense.h"
#include "faceCommon/linalg/common.h"
#include "realsenseprojection.h"
#include "faceCommon/settings/settings.h"

namespace dlib
{
	class shape_predictor;
}

namespace Face {
namespace Sensors {
namespace RealSense {

	namespace LandmarkGroup {
		const std::string LeftEyeBrow = "LeftEyeBrow";
		const std::string RightEyeBrow = "RightEyeBrow";
		const std::string BothEyeBrows = "BothEyeBrows";
		const std::string LeftEye = "LeftEye";
		const std::string RightEye = "RightEye";
		const std::string Mouth = "Mouth";
		const std::string MouthCorners = "MouthCorners";
		const std::string UnderNose = "UnderNose";
	}

class REALSENSE_EXPORTS LandmarkDetector
{
private:
	std::shared_ptr<dlib::shape_predictor> shapePredictor;

public:
	typedef std::vector<cv::Point2d> Point2dVec;
	typedef std::map<std::string, Point2dVec> LMPointsMap;

	LandmarkDetector(const std::string &pathToLandmarkModel = Face::Settings::instance().settingsMap[Face::Settings::ShapePredictorPathKey]);
	~LandmarkDetector();

	Point2dVec getLandmarks(const ImageGrayscale &img, const cv::Rect &roi, bool filter);

	LMPointsMap lmGroupPoints(const Point2dVec& lmCoords) const;

};

}
}
}

#endif //REALSENSE_LMDETECTOR
