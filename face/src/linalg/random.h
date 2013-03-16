#ifndef RANDOM_H
#define RANDOM_H

#include <QVector>

#include "common.h"
#include "vector.h"

class Random
{
public:
    static QVector<Vector> gauss(QVector<double> mean, QVector<double> sigma, int count);
};

#endif // RANDOM_H
