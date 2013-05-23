#ifndef HISTOGRAMFEATURES_H
#define HISTOGRAMFEATURES_H

#include <QVector>

#include "linalg/histogram.h"
#include "facelib/mesh.h"
#include "facelib/map.h"

class HistogramFeatures
{
public:
    QVector<Histogram> histograms;

    HistogramFeatures(Map &depthmap, int stripes, int binsPerStripe);
};

#endif // HISTOGRAMFEATURES_H
