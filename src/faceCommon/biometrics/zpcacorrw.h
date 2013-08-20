#ifndef ZPCACORRW_H
#define ZPCACORRW_H

#include <QVector>

#include "linalg/vector.h"
#include "linalg/metrics.h"
#include "featureextractor.h"
#include "eerpotential.h"

class ZPCACorrW
{
public:
    ZScorePCAExtractor extractor;
    CorrelationWeightedMetric metric;

    ZPCACorrW(const QVector<Vector> &trainPCAVectors, double pcaSelThreshold, const QVector<Vector> &trainZscoreVectors);

    ZPCACorrW(const QVector<Vector> &trainPCAVectors, double pcaSelThreshold, const QVector<Vector> &trainZscoreVectors,
              const QVector<int> &trainFeatureSelectionClasses, const QVector<Vector> &trainFeatureSelectionVectors,
              double featureSelThresholdStart, double featureSelThresholdEnd, double featureSelThresholdStep);

private:
    void init(const QVector<Vector> &trainPCAVectors, double pcaSelThreshold, const QVector<Vector> &trainZscoreVectors);
};

#endif // ZPCACORRW_H
