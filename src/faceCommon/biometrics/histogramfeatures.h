#ifndef HISTOGRAMFEATURES_H
#define HISTOGRAMFEATURES_H

#include <QVector>

#include "linalg/vector.h"
#include "linalg/histogram.h"
#include "facelib/mesh.h"
#include "facelib/map.h"

class HistogramFeatures
{
public:
    QVector<Histogram> histograms;

    //HistogramFeatures(Map &depthmap, int stripes, int binsPerStripe);
    HistogramFeatures(ImageGrayscale &depthmap, int stripes, int binsPerStripe);

    Vector toVector();
};

#endif // HISTOGRAMFEATURES_H
