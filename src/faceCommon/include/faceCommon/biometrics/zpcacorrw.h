#ifndef ZPCACORRW_H
#define ZPCACORRW_H

#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/metrics.h"
#include "faceCommon/biometrics/featureextractor.h"

namespace Face {
namespace Biometrics {

class ZScorePCAExtractor;

class ZPCACorrW
{
public:
    ZScorePCAExtractor extractor;
    Face::LinAlg::CorrelationWeightedMetric metric;

    ZPCACorrW() {}

    ZPCACorrW(const std::vector<Face::LinAlg::Vector> &trainPCAVectors, double pcaSelThreshold,
              const std::vector<Face::LinAlg::Vector> &trainZscoreVectors);

    ZPCACorrW(const std::vector<Face::LinAlg::Vector> &trainPCAVectors, double pcaSelThreshold,
              const std::vector<Face::LinAlg::Vector> &trainZscoreVectors,
              const std::vector<int> &trainFeatureSelectionClasses,
              const std::vector<Face::LinAlg::Vector> &trainFeatureSelectionVectors,
              double featureSelThresholdStart, double featureSelThresholdEnd, double featureSelThresholdStep);

private:
    void init(const std::vector<Face::LinAlg::Vector> &trainPCAVectors, double pcaSelThreshold,
              const std::vector<Face::LinAlg::Vector> &trainZscoreVectors);
};

}
}

#endif // ZPCACORRW_H
