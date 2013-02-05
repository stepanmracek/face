#include "featurepotentialbase.h"

#include "linalg/vector.h"

Matrix FeaturePotentialBase::createSelectionWeights(double threshold)
{
    int r = scores.count();
    Matrix result = Matrix::zeros(r, 1);
    for (int i = 0; i < r; i++)
    {
        if (scores[i] >= threshold)
            result(i) = 1.0;
    }
    return result;
}

Matrix FeaturePotentialBase::createWeights()
{
    Matrix result = Vector::fromQVector(scores);
    return Vector::normalizeComponents(result);
}
