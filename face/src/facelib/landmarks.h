#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <QString>
#include <QVector>
#include <opencv2/core/core.hpp>

#include "linalg/common.h"

class Landmarks
{
public:
    QVector<cv::Point3d> points;

    Landmarks() { points = QVector<cv::Point3d>(8, cv::Point3d()); }
    Landmarks(QVector<cv::Point3d> points) { this->points = points; }
    Landmarks(const QString &path);

    enum LandmarkNames
    {
        LeftOuterEye = 0,
        LeftInnerEye = 1,
        NasalBridge = 2,
        RightInnerEye = 3,
        RightOuterEye = 4,
        LeftOuterNose = 5,
        Nosetip = 6,
        RightOuterNose = 7
    };

    void serialize(const QString &path);

    cv::Point3d get(LandmarkNames name) { return points[name]; }

    void set(LandmarkNames name, cv::Point3d value) { points[name] = value; }

    bool check();
};

#endif // LANDMARKS_H
