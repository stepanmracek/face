#ifndef TESTDISTANCE_H
#define TESTDISTANCE_H

#include "linalg/metrics.h"

class TestDistance
{
public:
    static void testCosine()
    {
        CosineMetric cos;

        Matrix first =  (Matrix(3,1) << 1.0, -2.0, 0.5 );
        Matrix second = -first;

        qDebug() << cos.distance(first, second);
    }
};

#endif // TESTDISTANCE_H
