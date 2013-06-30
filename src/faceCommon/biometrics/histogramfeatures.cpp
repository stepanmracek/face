#include "histogramfeatures.h"

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
