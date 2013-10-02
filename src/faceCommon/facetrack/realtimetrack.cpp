#include "realtimetrack.h"

#include <QDebug>

RealTimeTrack::RealTimeTrack(const QString &path)
{
    // "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"
    qDebug() << "face detect:" << faceDetect.load(path.toStdString());
}

std::vector<cv::Rect>  RealTimeTrack::trackFace(ImageGrayscale &img)
{
    std::vector<cv::Rect> result;
    faceDetect.detectMultiScale(img, result);
    return result;
}

