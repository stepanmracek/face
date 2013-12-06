#ifndef DELAUNAY_H
#define DELAUNAY_H

#include <QVector>

#include <opencv2/opencv.hpp>

class Delaunay
{
public:
    static QVector<cv::Vec3i> process(QVector<cv::Point2d> &points);
};

#endif // DELAUNAY_H
