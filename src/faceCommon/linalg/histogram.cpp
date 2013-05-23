#include "histogram.h"

#include <assert.h>

#include "vector.h"

Histogram::Histogram(const QVector<double> &values, int bins, bool normalize, double minValue, double maxValue)
{
    Vector vec(values);

    if (minValue == 0 && maxValue == 0)
    {
        minValue = vec.minValue();
        maxValue = vec.maxValue();
    }

    this->minValue = minValue;
    this->maxValue = maxValue;
    this->mean = vec.meanValue();
    this->stdDev = vec.stdDeviation();

    assert(maxValue > minValue);
    assert(bins > 1);

    histogramCounter.resize(bins);
    histogramValues.resize(bins);

    double delta = maxValue - minValue;
    foreach (double v, values)
    {
        if (v < minValue) v = minValue;
        if (v > maxValue) v = maxValue;
        double distance = v - minValue;
        double ratio = distance/delta;
        int bin = ratio*bins;
        if (bin == bins) bin = bins - 1;
        histogramCounter[bin] += 1;
    }

    for (int i = 0; i < bins; i++)
    {
        if (normalize)
        {
            histogramCounter[i] /= values.count();
        }

        histogramValues[i] = delta/bins*i + minValue;
    }
}
