#include "faceCommon/objectdetection/detector.h"

#include <dlib/opencv/cv_image.h>
#include <dlib/image_processing/frontal_face_detector.h>

#ifdef __linux__
#include <dlib/all/source.cpp>
#endif

using namespace Face::ObjectDetection;

OpenCVDetector::OpenCVDetector(const std::string &cascadeClassifierPath)
{
	if (!classifier.load(cascadeClassifierPath))
	{
		std::string msg = "can't load classifier " + cascadeClassifierPath;
		std::cerr << msg << std::endl;
		throw FACELIB_EXCEPTION(msg);
	}
}

std::vector<cv::Rect> OpenCVDetector::detect(const ImageGrayscale &img)
{
	std::vector<cv::Rect> regions;
	classifier.detectMultiScale(img, regions);
	return regions;
}

DlibDetector::DlibDetector()
{
	detector = std::make_shared<dlib::frontal_face_detector>(dlib::get_frontal_face_detector());
}

std::vector<cv::Rect> DlibDetector::processResult(const std::vector<dlib::rectangle> &in)
{
	std::vector<cv::Rect> out;
	for (const auto &i : in) {
		out.push_back(cv::Rect(i.left(), i.top(), i.right() - i.left(), i.bottom() - i.top()));
	}
	return out;
}

std::vector<cv::Rect> DlibDetector::detect(const ImageGrayscale &img)
{
	dlib::cv_image<unsigned char> dimg(img);
	return processResult((*detector)(dimg));
}
