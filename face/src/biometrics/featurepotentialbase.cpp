#include "featurepotentialbase.h"

Vector FeaturePotentialBase::createSelectionWeights(double threshold)
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

Vector FeaturePotentialBase::createWeights()
{
    Vector result(scores);
    result = result.normalizeComponents();
    return result;
}
