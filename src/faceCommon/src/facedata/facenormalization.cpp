#include "faceCommon/facedata/facenormalization.h"
#include "faceCommon/linalg/procrustes.h"

using namespace Face::FaceData;

FaceNormalization::FaceNormalization()
{
    /*cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        throw FACELIB_EXCEPTION("Unable to open " + path);
    }

    width = fs["width"];
    height = fs["height"];
    cv::FileNode node = fs["points"];
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
    {
        std::vector<double> tmp;
        (*it) >> tmp;
        cv::Point2d p(tmp[0], tmp[1]);
        referencePoints.push_back(p);
    }

    //referenceVector = Face::LinAlg::Vector(referencePoints);*/
}

/*void FaceNormalization::learn(const ImageGrayscale &input, const std::vector<cv::Point2d> &landmarks, const cv::Rect &roi, const std::string &path)
{
    ImageGrayscale result;
    cv::Rect scaledRoi = cv::Rect(roi.x*2, roi.y*2, roi.width*2, roi.height*2);
    std::cout << scaledRoi.width << " " << scaledRoi.height << std::endl;
    cv::resize(input, result, cv::Size(input.cols*2, input.rows*2));

    for(size_t i = 0; i < landmarks.size(); i++)
    {
        const cv::Point2d &p = landmarks[i];
        cv::putText(result, std::to_string(i), cv::Point2d(p.x*2, p.y*2), cv::FONT_HERSHEY_SIMPLEX, 0.25, 255);
    }
    cv::rectangle(result, scaledRoi, 255);

    cv::imshow("all", result);
    cv::imshow("result", result(scaledRoi));
    char c = cv::waitKey(1);

    if (c == ' ')
    {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "width" << scaledRoi.width;
        fs << "height" << scaledRoi.height;
        fs << "points" << "[";
        for(size_t i = 0; i < landmarks.size(); i++)
        {
            const cv::Point2d &p = landmarks[i];
            fs << cv::Point2d(p.x*2 - scaledRoi.x, p.y*2 - scaledRoi.y);
        }
        fs << "]";

        fs.release();
    }
}*/

ImageGrayscale FaceNormalization::normalize(const ImageGrayscale &input, const std::vector<cv::Point2d> &landmarks)
{
    cv::Point2d leftEyeCenter(0,0);
    for (int i = 15; i <= 20; i++)
    {
        leftEyeCenter.x += landmarks[i].x;
        leftEyeCenter.y += landmarks[i].y;
    }
    leftEyeCenter.x /= 6.0;
    leftEyeCenter.y /= 6.0;

    cv::Point2d rightEyeCenter(0,0);
    for (int i = 9; i <= 14; i++)
    {
        rightEyeCenter.x += landmarks[i].x;
        rightEyeCenter.y += landmarks[i].y;
    }
    rightEyeCenter.x /= 6.0;
    rightEyeCenter.y /= 6.0;

    cv::Point2f center((leftEyeCenter.x + rightEyeCenter.x)/2.0f, (leftEyeCenter.y + rightEyeCenter.y)/2.0f);
    double angle = atan2(leftEyeCenter.y - rightEyeCenter.y, leftEyeCenter.x - rightEyeCenter.x) * 180/M_PI;
    double scale = 100.0 / sqrt(pow(rightEyeCenter.x - leftEyeCenter.x, 2) + pow(rightEyeCenter.y - leftEyeCenter.y, 2));
    cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, scale);

    ImageGrayscale rotated;
    cv::warpAffine(input, rotated, rotMat, cv::Size());
    ImageGrayscale normalized;
    cv::getRectSubPix(rotated, cv::Size(150, 200), center, normalized);
    return normalized(cv::Rect(0, 50, 150, 150)).clone();
    
    //ImageGrayscale result = input.clone();
    /*for (int i = 0; i < landmarks.size(); ++i)
    {
        cv::putText(result, std::to_string(i), landmarks[i], cv::FONT_HERSHEY_SIMPLEX, 0.25, 255, 1, CV_AA);
    }*/
    //cv::putText(result, "L", leftEyeCenter, cv::FONT_HERSHEY_SIMPLEX, 0.25, 255, 1, CV_AA);
   // cv::putText(result, "R", rightEyeCenter, cv::FONT_HERSHEY_SIMPLEX, 0.25, 255, 1, CV_AA);
    //return result;
    
    /*Face::LinAlg::Vector reference(referencePoints);
    Face::LinAlg::TranslationCoefs centralizeReference = Face::LinAlg::Procrustes2D::centralizedTranslation(reference);
    Face::LinAlg::Procrustes2D::translate(reference, centralizeReference);

    Face::LinAlg::Vector lmVector(landmarks);
    Face::LinAlg::TranslationCoefs centralizeLandmarks = Face::LinAlg::Procrustes2D::centralizedTranslation(lmVector);
    Face::LinAlg::Procrustes2D::translate(lmVector, centralizeLandmarks);

    Face::LinAlg::RotateAndScaleCoefs coef = Face::LinAlg::Procrustes2D::align(lmVector, reference);
    Face::LinAlg::Procrustes2D::rotateAndScale(lmVector, coef);

    ImageGrayscale result = ImageGrayscale::zeros(height, width);

    cv::Point2f center(-centralizeLandmarks.xt, -centralizeLandmarks.yt);
    cv::Mat rotation = cv::getRotationMatrix2D(center, -coef.theta/(2*M_PI)*360, coef.s);
    cv::circle(result, center, 3, 255, 1, CV_AA);

    cv::warpAffine(input, result, rotation, result.size());

    return result;*/


    /*ImageGrayscale result;

    for(size_t i = 0; i < landmarks.size(); i++)
    {
        const cv::Point2d &p = landmarks[i];
        cv::putText(result, std::to_string(i), cv::Point2d(p.x*2, p.y*2), cv::FONT_HERSHEY_SIMPLEX, 0.25, 255);
    }
    cv::rectangle(result, scaledRoi, 255);

    cv::imshow("all", result);
    cv::imshow("result", result(scaledRoi));
    char c = cv::waitKey(1);

    if (c == ' ')
    {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "width" << scaledRoi.width;
        fs << "height" << scaledRoi.height;
        fs << "points" << "[";
        for(size_t i = 0; i < landmarks.size(); i++)
        {
            const cv::Point2d &p = landmarks[i];
            fs << cv::Point2d(p.x*2 - scaledRoi.x, p.y*2 - scaledRoi.y);
        }
        fs << "]";

        fs.release();
    }*/
}
