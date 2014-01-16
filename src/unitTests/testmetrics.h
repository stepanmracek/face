#ifndef TESTMETRICS_H
#define TESTMETRICS_H

#include <QVector>

#include <opencv/cv.h>

#include "linalg/metrics.h"

class TestMetrics
{
public:
    static void testMahalanobisDistance()
    {
        QVector<Face::LinAlg::Vector> samples;

        Face::LinAlg::Vector s1(4);
        s1(0) = 3;
        s1(1) = 2;
        s1(2) = 4;
        s1(3) = 5;

        Face::LinAlg::Vector s2(4);
        s2(0) = 3.1;
        s2(1) = 2;
        s2(2) = 4.1;
        s2(3) = 7.2;

        Face::LinAlg::Vector s3(4);
        s3(0) = 2.3;
        s3(1) = 2;
        s3(2) = 4.2;
        s3(3) = 4.6;

        samples << s1 << s2 << s3;

        Face::LinAlg::MahalanobisMetric m(samples);

        qDebug() << "mean:";
        for (int i = 0; i < m.mean.rows; i++)
        {
            qDebug() << m.mean(i, 0);
        }

        qDebug() << "inv. covar:" << m.invCov.rows << m.invCov.cols;


    }
};

#endif // TESTMETRICS_H
