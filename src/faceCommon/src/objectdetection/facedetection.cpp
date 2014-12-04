#include "faceCommon/objectdetection/facedetection.h"

#include "faceCommon/linalg/common.h"

using namespace Face::ObjectDetection;

FaceDetection::FaceDetection(const std::string &faceClassifierPath,
                             const std::string &leftEyeClassifierPath,
                             const std::string &rightEyeClassifierPath,
                             int inputImageScaleFactor) :
    inputImageScaleFactor(inputImageScaleFactor)
{
    if (!faceClassifier.load(faceClassifierPath))
        throw FACELIB_EXCEPTION("can't load face classifier " + faceClassifierPath);
    if (!leftEyeClassifier.load(leftEyeClassifierPath))
        throw FACELIB_EXCEPTION("can't load left eye classifier " + leftEyeClassifierPath);
    if (!rightEyeClassifier.load(rightEyeClassifierPath))
        throw FACELIB_EXCEPTION("can't load right eye classifier " + rightEyeClassifierPath);
}

FaceDetection::DetectionResult FaceDetection::detect(const ImageGrayscale &inputImg, bool normalizeFaces)
{
    DetectionResult result;
    ImageGrayscale scaled(inputImg.rows/inputImageScaleFactor, inputImg.cols/inputImageScaleFactor);
    cv::resize(inputImg, scaled, cv::Size(inputImg.cols/inputImageScaleFactor, inputImg.rows/inputImageScaleFactor));

    std::vector<cv::Rect> faceRegions;
    faceClassifier.detectMultiScale(scaled, faceRegions);
    for (const cv::Rect &fr : faceRegions)
    {
        cv::Rect fullFaceRegion(fr.x*inputImageScaleFactor, fr.y*inputImageScaleFactor,
                                fr.width*inputImageScaleFactor, fr.height*inputImageScaleFactor);

        std::vector<cv::Rect> leftEyes;
        cv::Rect leftEyeRoi = cv::Rect(fullFaceRegion.x + 0.33*fullFaceRegion.width, fullFaceRegion.y,
                                       0.66*fullFaceRegion.width, 0.66*fullFaceRegion.height);
        leftEyeClassifier.detectMultiScale(inputImg(leftEyeRoi), leftEyes);
        if (leftEyes.size() == 0) continue;

        std::vector<cv::Rect> rightEyes;
        cv::Rect rightEyeRoi = cv::Rect(fullFaceRegion.x, fullFaceRegion.y,
                                        0.66*fullFaceRegion.width, 0.66*fullFaceRegion.height);
        rightEyeClassifier.detectMultiScale(inputImg(rightEyeRoi), rightEyes);
        if (rightEyes.size() == 0) continue;

        result.faceRegions.push_back(fullFaceRegion);
        cv::Rect leftEye(leftEyeRoi.x + leftEyes[0].x, leftEyeRoi.y + leftEyes[0].y,
                leftEyes[0].width, leftEyes[0].height);
        result.leftEyes.push_back(leftEye);
        cv::Rect rightEye(rightEyeRoi.x + rightEyes[0].x, rightEyeRoi.y + rightEyes[0].y,
                rightEyes[0].width, rightEyes[0].height);
        result.rightEyes.push_back(rightEye);

        if (normalizeFaces)
        {
            cv::Point leftEyeCenter(leftEye.x + leftEye.width/2, leftEye.y + leftEye.height/2);
            cv::Point rightEyeCenter(rightEye.x + rightEye.width/2, rightEye.y + rightEye.height/2);
            cv::Point2f center((leftEyeCenter.x + rightEyeCenter.x)/2.0f, (leftEyeCenter.y + rightEyeCenter.y)/2.0f);
            double angle = atan2(leftEyeCenter.y - rightEyeCenter.y, leftEyeCenter.x - rightEyeCenter.x) * 180/M_PI;
            double scale = 100.0 / sqrt(pow(rightEyeCenter.x - leftEyeCenter.x, 2) + pow(rightEyeCenter.y - leftEyeCenter.y, 2));
            cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, scale);

            ImageGrayscale rotated;
            cv::warpAffine(inputImg, rotated, rotMat, cv::Size());
            ImageGrayscale normalized;
            cv::getRectSubPix(rotated, cv::Size(150, 200), center, normalized);
            result.normalizedFaceImages.push_back(normalized(cv::Rect(0, 50, 150, 150)));
        }
    }

    return result;
}
