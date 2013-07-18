#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <QVector>
#include <QString>

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
    Histogram(const QVector<double> &values, int bins, bool normalize, double minValue = 0, double maxValue = 0);

    void savePlot(const QString &path) const;
};

#endif // HISTOGRAM_H
