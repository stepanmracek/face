#include "histogram.h"

#include <assert.h>

#include "vector.h"

Histogram::Histogram(const QVector<double> &values, int bins, bool normalize, double minValue, double maxValue, bool ignoreOutliers)
{
    histogramCounter.resize(bins);
    histogramValues.resize(bins);

    if (values.count() == 0)
    {
        this->minValue = 0;
        this->maxValue = 0;
        this->mean = 0;
        this->stdDev = 0;
        return;
    }
    else if (values.count() == 1)
    {
        this->minValue = values[0];
        this->maxValue = values[0];
        this->mean = values[0];
        this->stdDev = 0;
        return;
    }

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

    double delta = maxValue - minValue;
    int validCount = 0;
    foreach (double v, values)
    {
        if (ignoreOutliers)
        {
            if (v < minValue || v > maxValue)
            {
                continue;
            }
        }
        validCount++;
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
            histogramCounter[i] /= validCount;
        }

        histogramValues[i] = delta/bins*i + minValue;
    }
}

void Histogram::savePlot(const QString &path) const
{
    Common::savePlot(histogramValues, histogramCounter, path);
}

Matrix Histogram::plot() const
{
    int n = histogramValues.count();
    Matrix result = Matrix::zeros(100, n*10);
    for (int i = 0; i < n; i++)
    {
        cv::line(result, cv::Point(i*10, 100), cv::Point2i(i*10, 100 - histogramCounter[i]*100), 1.0);
    }
    return result;
}
