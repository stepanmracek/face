#include "realtimetrack.h"

#include <QDebug>

RealTimeTracker::RealTimeTracker(const QString &path)
{
    // "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"
    qDebug() << "haar tracker loaded:" << classifier.load(path.toStdString());
}

std::vector<cv::Rect>  RealTimeTracker::detect(ImageGrayscale &img)
{
    std::vector<cv::Rect> result;
    classifier.detectMultiScale(img, result);
    return result;
}

