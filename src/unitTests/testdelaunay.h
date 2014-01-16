#ifndef TESTDELAUNAY_H
#define TESTDELAUNAY_H

#include <QVector>
#include <QDebug>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "linalg/delaunay.h"
#include "linalg/matrixconverter.h"

class TestDelaunay
{
public:
    static void testSubdivision()
    {
        srand(time(0));
        cv::Scalar color;
        QVector<cv::Point2d> points;
        int n = 100;
        for (int i = 0; i < n; i++)
        {
            cv::Point2d p;
            p.x = rand() % 700;
            p.y = rand() % 700;
            points.append(p);
        }

        QVector<cv::Vec3i> subdiv = Face::LinAlg::Delaunay::process(points);

        cv::Mat img = cv::Mat::zeros(700, 700, CV_8UC3);
        cv::Scalar white; white[0] = white[1] = white[2] = 255;
        cv::namedWindow("delaunay");
        foreach(cv::Vec3i t, subdiv)
        {
            cv::Point pts[3] = {points[t[0]], points[t[1]], points[t[2]]};

            color[0] = rand() % 100+100;
            color[1] = rand() % 100+100;
            color[2] = rand() % 100+100;
            cv::fillConvexPoly(img, pts, 3, color, CV_AA);

            cv::line(img, points[t[0]], points[t[1]], white, 1, CV_AA);
            cv::line(img, points[t[1]], points[t[2]], white, 1, CV_AA);
            cv::line(img, points[t[2]], points[t[0]], white, 1, CV_AA);

            qDebug() << "drawn" << t[0] << t[1] << t[2];

            cv::imshow("delaunay", img);
            cv::waitKey(1);
        }

        cv::waitKey();
    }
};

#endif // TESTDELAUNAY_H
