#ifndef RANDOM_H
#define RANDOM_H

#include <QVector>

#include "common.h"

class Random
{
public:
    static QVector<Matrix> gauss(QVector<double> mean, QVector<double> sigma, int count);
};

#endif // RANDOM_H
