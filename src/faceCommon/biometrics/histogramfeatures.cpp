#include "histogramfeatures.h"

#include <assert.h>

HistogramFeatures::HistogramFeatures(const Map &depthmap, int stripes, int binsPerStripe, double minValue, double maxValue)
{
    QVector<QVector<double> > valuesInStripes(stripes);
    for (int y = 0; y < depthmap.h; y++)
    {
        int histogramIndex = ((double)y)/depthmap.h * stripes;
        if (histogramIndex == stripes) histogramIndex--;

        for (int x = 0; x < depthmap.w; x++)
        {
            if (!depthmap.isSet(x, y)) continue;

            valuesInStripes[histogramIndex] << depthmap.get(x, y);
        }
    }

    for (int i = 0; i < stripes; i++)
    {
        histograms << Histogram(valuesInStripes[i], binsPerStripe, true, minValue, maxValue);
    }
}

HistogramFeatures::HistogramFeatures(const Map &depthmap, int stripes, int binsPerStripe,
                                     const QVector<double> &minValues, const QVector<double> &maxValues)
{
    assert(stripes == minValues.count());
    assert(stripes == maxValues.count());

    QVector<QVector<double> > valuesInStripes(stripes);
    for (int y = 0; y < depthmap.h; y++)
    {
        int histogramIndex = ((double)y)/depthmap.h * stripes;
        if (histogramIndex == stripes) histogramIndex--;

        for (int x = 0; x < depthmap.w; x++)
        {
            if (!depthmap.isSet(x, y)) continue;

            valuesInStripes[histogramIndex] << depthmap.get(x, y);
        }
    }

    for (int i = 0; i < stripes; i++)
    {
        histograms << Histogram(valuesInStripes[i], binsPerStripe, true, minValues[i], maxValues[i]);
    }
}

HistogramFeatures::HistogramFeatures(const ImageGrayscale &depthmap, int stripes, int binsPerStripe)
{
    QVector<QVector<double> > valuesInStripes(stripes);
    for (int y = 0; y < depthmap.rows; y++)
    {
        int histogramIndex = (((double)y)/depthmap.rows) * stripes;
        if (histogramIndex == stripes) histogramIndex--;

        for (int x = 0; x < depthmap.cols; x++)
        {
            valuesInStripes[histogramIndex] << depthmap(y, x);
        }
    }

    for (int i = 0; i < stripes; i++)
    {
        histograms << Histogram(valuesInStripes[i], binsPerStripe, true, 0, 255);
    }
}

Vector HistogramFeatures::toVector()
{
    QVector<double> allValues;
    for (int i = 0; i < histograms.count(); i++)
    {
        allValues << histograms[i].histogramCounter;
    }

    return Vector(allValues);
}

QPair<QVector<double>, QVector<double> > HistogramFeatures::trainMinMaxValues(const QVector<Map> &depthmaps, int stripes, double stdMultiplier)
{
    QPair<QVector<double>, QVector<double> > result;
    QVector<QVector<double> > valuesInStripes(stripes);

    foreach (const Map &depthmap, depthmaps)
    {
        for (int y = 0; y < depthmap.h; y++)
        {
            int stripeIndex = ((double)y)/depthmap.h * stripes;
            if (stripeIndex == stripes) stripeIndex--;

            for (int x = 0; x < depthmap.w; x++)
            {
                if (!depthmap.isSet(x, y)) continue;

                valuesInStripes[stripeIndex] << depthmap.get(x, y);
            }
        }
    }

    int stripeIndex = 0;
    foreach(const QVector<double> &stripeValues, valuesInStripes)
    {
        Vector v(stripeValues);
        double mean = v.meanValue();
        double std = v.stdDeviation();
        double min = v.minValue();
        double max = v.maxValue();
        qDebug() << stripeIndex << mean << std << (mean - stdMultiplier*std) << (mean + stdMultiplier*std) << min << max;

        result.first << (mean - stdMultiplier*std);
        result.second << (mean + stdMultiplier*std);

        Histogram h(stripeValues, 50, true);
        Common::savePlot(h.histogramValues, h.histogramCounter, QString::number(stripeIndex));

        stripeIndex++;
    }

    return result;
}
