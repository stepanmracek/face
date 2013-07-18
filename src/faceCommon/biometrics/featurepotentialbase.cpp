#include "featurepotentialbase.h"

#include <cassert>

Vector FeaturePotentialBase::createSelectionWeights(double threshold) const
{
    int r = scores.rows;
    Vector result(r);
    for (int i = 0; i < r; i++)
    {
        if (scores(i) >= threshold)
            result(i) = 1.0;
    }
    return result;
}

Vector FeaturePotentialBase::createSelectionWeightsBasedOnRelativeThreshold(double threshold) const
{
    assert(threshold >= 0);
    assert(threshold < 1);

    threshold = threshold * (maxScore - minScore) + minScore;
    return createSelectionWeights(threshold);
}

Vector FeaturePotentialBase::createWeights() const
{
    Vector result(scores);
    result = result.normalizeComponents();
    return result;
}
