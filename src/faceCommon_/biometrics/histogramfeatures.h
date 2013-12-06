#ifndef HISTOGRAMFEATURES_H
#define HISTOGRAMFEATURES_H

#include <QVector>
#include <QPair>

#include "linalg/vector.h"
#include "linalg/histogram.h"
#include "facelib/mesh.h"
#include "facelib/map.h"

class HistogramFeatures
{
public:
    QVector<Histogram> histograms;

    HistogramFeatures(const Map &depthmap, int stripes, int binsPerStripe,
                      double minValue = 0, double maxValue = 0);
    HistogramFeatures(const Map &depthmap, int stripes, int binsPerStripe,
                      const QVector<double> &minValues, const QVector<double> &maxValues);

    HistogramFeatures(const ImageGrayscale &depthmap, int stripes, int binsPerStripe);

    Vector toVector();

    static QPair<QVector<double>, QVector<double> > trainMinMaxValues(const QVector<Map> &depthmaps,
                                                                      int stripes, double stdMultiplier = 2);
};

#endif // HISTOGRAMFEATURES_H
