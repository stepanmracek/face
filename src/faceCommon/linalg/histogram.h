#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <QVector>
#include <QString>

#include "linalg/common.h"

class Histogram
{
public:
    double minValue;
    double maxValue;
    double mean;
    double stdDev;
    QVector<double> histogramCounter;
    QVector<double> histogramValues;

    Histogram() {}
    Histogram(const QVector<double> &values, int bins, bool normalize, double minValue = 0, double maxValue = 0, bool ignoreOutliers = false);

    void savePlot(const QString &path) const;
    Matrix plot() const;
};

#endif // HISTOGRAM_H
