#ifndef FEATUREPOTENTIALBASE_H
#define FEATUREPOTENTIALBASE_H

#include "faceCommon/linalg/vector.h"

namespace Face {
namespace Biometrics {

class FACECOMMON_EXPORTS FeaturePotentialBase
{
public:
    Face::LinAlg::Vector scores;
    double minScore;
    double maxScore;

    Face::LinAlg::Vector createSelectionWeightsBasedOnRelativeThreshold(double threshold) const;
    Face::LinAlg::Vector createSelectionWeights(double threshold) const;
    Face::LinAlg::Vector createWeights() const;
};

}
}

#endif // FEATUREPOTENTIALBASE_H
