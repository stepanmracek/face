#ifndef LINEPROCESSING_H
#define LINEPROCESSING_H

#include <QVector>

#include <opencv/cv.h>

#include "linalg/vector.h"
#include "linalg/common.h"
#include "activeshapemodelpoints.h"

class LineProcessing
{
public:
    static void line(int x, int y, double dx, double dy, int num, cv::Point *result);

    static void neighborhood(int x, int y, double dx, double dy, int num, cv::Point *result);

    //static cv::Point2d shapeNormal(int i, cv::Mat &shape);

    static cv::Point2d shapeNormal(int i, Matrix &shape, ActiveShapeModelPointDefinition &definition);
};

#endif // LINEPROCESSING_H
