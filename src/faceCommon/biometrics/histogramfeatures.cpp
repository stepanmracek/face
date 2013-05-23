#include "histogramfeatures.h"

HistogramFeatures::HistogramFeatures(Map &depthmap, int stripes, int binsPerStripe)
{
    histograms.resize(stripes);
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
        histograms << Histogram(valuesInStripes[i], binsPerStripe, true);
    }
}
