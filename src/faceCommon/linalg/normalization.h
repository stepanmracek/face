#ifndef NORMALIZATION_H
#define NORMALIZATION_H

#include <QVector>

#include "linalg/common.h"
#include "linalg/vector.h"

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

    ZScoreNormalizationResult() {}
    ZScoreNormalizationResult(const QString &path);

    void serialize(const QString &path);
};

class Normalization
{
public:
    static LinearNormalizationResult linearNormalization(QVector<Vector> &vectors);
    static void linearNormalization(QVector<Vector> &vectors, LinearNormalizationResult &params);

    static ZScoreNormalizationResult zScoreNormalization(QVector<Vector> &vectors);
    static void zScoreNormalization(QVector<Vector> &vectors, ZScoreNormalizationResult &params);
    static void zScoreNormalization(Vector &vector, ZScoreNormalizationResult &params);
};

#endif // NORMALIZATION_H
