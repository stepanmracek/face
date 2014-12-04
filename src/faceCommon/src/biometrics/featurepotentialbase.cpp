#include "faceCommon/biometrics/featurepotentialbase.h"

using namespace Face::Biometrics;

Face::LinAlg::Vector FeaturePotentialBase::createSelectionWeights(double threshold) const
{
    int r = scores.rows;
    Face::LinAlg::Vector result(r);
    for (int i = 0; i < r; i++)
    {
        if (scores(i) >= threshold)
            result(i) = 1.0;
    }
    return result;
}

Face::LinAlg::Vector FeaturePotentialBase::createSelectionWeightsBasedOnRelativeThreshold(double threshold) const
{
    if (threshold >= 1 || threshold < 0)
        throw FACELIB_EXCEPTION("threshold should be between in range [0; 1)");

    threshold = threshold * (maxScore - minScore) + minScore;
    return createSelectionWeights(threshold);
}

Face::LinAlg::Vector FeaturePotentialBase::createWeights() const
{
    Face::LinAlg::Vector result(scores);
    result = result.normalizeComponents();
    return result;
}
