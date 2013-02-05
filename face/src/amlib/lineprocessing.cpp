#include "lineprocessing.h"

#include <QtAlgorithms>

#include <cassert>

void LineProcessing::line(int startx, int starty, double dx, double dy, int num, cv::Point *result)
{
    assert(dx != 0 || dy != 0);

    double k = 0;
    if (dx != 0) k = dy/dx;

    double x = startx;
    double y = starty;

    double stepx = 0;
    double stepy = 0;

    // determine octant
    if (dx == 0)
    {
        stepx = 0;
        if (dy > 0)
            stepy = 1;
        else
            stepy = -1;
    }
    else if (dy == 0)
    {
        stepy = 0;
        if (dx > 0)
            stepx = 1;
        else
            stepx = -1;
    }
    else if (dx > 0 && dy > 0)
    {
        if (dx > dy)
        {
            stepx = 1;
            stepy = k;
            //qDebug() << "stepx" << stepx << "stepy" << stepy;
        }
        else
        {
            stepx = 1/k;
            stepy = 1;
        }
    }
    else if (dx > 0 && dy < 0)
    {
        if (fabs(dx) > fabs(dy))
        {
            stepx = 1;
            stepy = k;
        }
        else
        {
            stepx = -1/k;
            stepy = -1;
        }
    }
    else if (dx < 0 && dy > 0)
    {
        if (fabs(dx) > fabs(dy))
        {
            stepx = -1;
            stepy = -k;
        }
        else
        {
            stepx = 1/k;
            stepy = 1;
        }
    }
    else if (dx < 0 && dy < 0)
    {
        if (fabs(dx) > fabs(dy))
        {
            stepx = -1;
            stepy = -k;
        }
        else
        {
            stepx = -1/k;
            stepy = -1;
        }
    }

    // add points
    for (int i = 0; i < num; i++)
    {
        x += stepx;
        y += stepy;

        result[i].x = (int)x;
        result[i].y = (int)y;
        //result.append(p);
    }
}

void LineProcessing::neighborhood(int x, int y, double dx, double dy, int neighborhoodSize, cv::Point *result)
{
    // first part
    cv::Point firstPart[neighborhoodSize];
    line(x, y, -dx, -dy, neighborhoodSize, firstPart);
    int resultIndex = 0;
    for (int i = neighborhoodSize-1; i >= 0; i--)
    {
        result[resultIndex].x = firstPart[i].x;
        result[resultIndex].y = firstPart[i].y;
        resultIndex++;
        //result.append(firstPart.at(i));
    }

    // point itself
    result[resultIndex].x = x;
    result[resultIndex].y = y;
    resultIndex++;
    //cv::Point p; p.x = x; p.y = y;
    //result.append(p);

    // second part
    cv::Point secondPart[neighborhoodSize];
    line(x, y, dx, dy, neighborhoodSize, secondPart);
    for (int i = 0; i < neighborhoodSize; i++)
    {
        result[resultIndex].x = firstPart[i].x;
        result[resultIndex].y = firstPart[i].y;
        resultIndex++;
        //result.append(secondPart.at(i));
    }

    //qDebug() << "LineProcessing::neighborhood";
    //for (int i = 0; i < result.count(); i++)
    //    qDebug() << "  " << result.at(i).x << result.at(i).y;
}

cv::Point2d LineProcessing::shapeNormal(int i, Matrix &shape, ActiveShapeModelPointDefinition &definition)
{
    int n = shape.rows/2;
    assert (n >= 2);
    assert (n == definition.points.count());
    assert (i < n);

    ActiveShapeModelPoint &point = definition.points[i];
    cv::Point2d normal;
    double x1,x2,y1,y2;
    switch (point.type)
    {
    case pointType_begin:
        x1 = shape(i, 0);
        y2 = shape(i+n, 0);

        x2 = shape(point.point1, 0);
        y2 = shape(point.point1+n, 0);
        break;
    case pointType_end:
        x1 = shape(point.point1, 0);
        y1 = shape(point.point1+n, 0);

        x2 = shape(i, 0);
        y2 = shape(i+n, 0);
        break;
    case pointType_line:
    case pointType_tjunction:
        x1 = shape(point.point1, 0);
        y1 = shape(point.point1+n, 0);

        x2 = shape(point.point2, 0);
        y2 = shape(point.point2+n, 0);
        break;
    }

    normal.x = -(y2-y1);
    normal.y = (x2-x1);

    return normal;
}
