#ifndef NORMALIZATION_H
#define NORMALIZATION_H

#include <QVector>

#include "linalg/common.h"

class LinearNormalizationResult
{
public:
    QVector<double> minValues;
    QVector<double> maxValues;
};

class ZScoreNormalizationResult
{
public:
    QVector<double> mean;
    QVector<double> stdDev;
};

class Normalization
{
public:
    static LinearNormalizationResult linearNormalization(QVector<Matrix> &vectors);
    static void linearNormalization(QVector<Matrix> &vectors, LinearNormalizationResult &params);

    static ZScoreNormalizationResult zScoreNormalization(QVector<Matrix> &vectors);
    static void zScoreNormalization(QVector<Matrix> &vectors, ZScoreNormalizationResult &params);
    static void zScoreNormalization(Matrix &vector, ZScoreNormalizationResult &params);
};

#endif // NORMALIZATION_H
