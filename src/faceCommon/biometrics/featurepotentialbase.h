#ifndef FEATUREPOTENTIALBASE_H
#define FEATUREPOTENTIALBASE_H

#include <QVector>

#include "linalg/common.h"
#include "linalg/vector.h"

class FeaturePotentialBase
{
public:
    Vector scores;
    double minScore;
    double maxScore;

    Vector createSelectionWeights(double threshold);
    Vector createWeights();
};

#endif // FEATUREPOTENTIALBASE_H
