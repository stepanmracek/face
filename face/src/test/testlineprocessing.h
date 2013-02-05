#ifndef TESTLINEPROCESSING_H
#define TESTLINEPROCESSING_H

#include <QVector>
#include <QDebug>
#include <QFile>

#include <opencv/cv.h>
#include <cmath>

#include "amlib/lineprocessing.h"
#include "linalg/vector.h"

class TestLineProcessing
{
public:
    static void testLine()
    {
        int n = 20;
        cv::Point line[n];
        LineProcessing::line(10, 10, 4.0, 2.0, n, line);
        Matrix v(2*n, 1);
        for (int i = 0; i < n; i++)
        {
            v(i, 0) = line[i].x;
            v(i+n, 0) = line[i].y;
        }

        Vector::toFileTwoCols(v, "line");
    }

    static void testMultipleLines()
    {
        if (QFile::exists("lines"))
            QFile::remove("lines");

        int n = 100;
        for (double arg = 0; arg <= 2 * M_PI; arg += 0.1)
        {
            double dx = cos(arg);
            double dy = sin(arg);

            qDebug() << arg << dx << dy;
            cv::Point line[n];
            LineProcessing::line(10, 10, dx, dy, n, line);
            Matrix v(2*n, 1, CV_64F);
            for (int i = 0; i < n; i++)
            {
                v(i, 0) = line[i].x;
                v(i+n, 0) = line[i].y;
            }

            Vector::toFileTwoCols(v, "lines", true);
        }
    }
};

#endif // TESTLINEPROCESSING_H
