#ifndef FEATUREPOTENTIALBASE_H
#define FEATUREPOTENTIALBASE_H

#include <QVector>

#include "linalg/common.h"

class FeaturePotentialBase
{
public:
    QVector<double> scores;
    double minScore;
    double maxScore;

    Matrix createSelectionWeights(double threshold);
    Matrix createWeights();
};

#endif // FEATUREPOTENTIALBASE_H
