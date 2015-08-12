#ifndef DETECTOR_H
#define DETECTOR_H

#include <vector>
#include <memory>
#include <opencv/cv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <Poco/SharedPtr.h>

#include "faceCommon/settings/settings.h"
#include "faceCommon/linalg/common.h"
#include "faceCommon/faceCommon.h"

namespace Face {
namespace ObjectDetection {

class FACECOMMON_EXPORTS Detector
{
public:
	typedef Poco::SharedPtr<Detector> Ptr;

    virtual std::vector<cv::Rect> detect(const ImageGrayscale &img) = 0;
	virtual ~Detector() {}
};

class FACECOMMON_EXPORTS OpenCVDetector : public Detector
{
private:
	cv::CascadeClassifier classifier;

public:
	OpenCVDetector(const std::string &cascadeClassifierPath = Face::Settings::instance().settingsMap[Face::Settings::CascadeFaceDetectorPathKey]);

    std::vector<cv::Rect> detect(const ImageGrayscale &img);
};

class FACECOMMON_EXPORTS DlibDetector : public Detector
{
private:
	std::shared_ptr<dlib::frontal_face_detector> detector;

	std::vector<cv::Rect> processResult(const std::vector<dlib::rectangle> &in);

public:
	DlibDetector();

    std::vector<cv::Rect> detect(const ImageGrayscale &img);
};

}
}

#endif // DETECTOR_H
