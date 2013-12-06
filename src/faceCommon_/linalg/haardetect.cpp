#include "haardetect.h"

#include <vector>

void HaarDetect::detect(Matrix &img)
{
    cv::equalizeHist(img, img);
    std::vector<cv::Rect> faces;

    cascadeClassifier.detectMultiScale(img, faces, 1.1, 2, 0);
}
