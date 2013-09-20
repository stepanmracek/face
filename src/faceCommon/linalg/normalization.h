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

class MeanNormalizationResult
{
public:
    QVector<double> mean;
};

class ZScoreNormalizationResult
{
public:
    QVector<double> mean;
    QVector<double> stdDev;

    ZScoreNormalizationResult() {}
    ZScoreNormalizationResult(const QString &path);

    void serialize(const QString &path) const;
};

class Normalization
{
public:
    static LinearNormalizationResult linearNormalization(QVector<Vector> &vectors);
    static void linearNormalization(QVector<Vector> &vectors, const LinearNormalizationResult &params);

    static ZScoreNormalizationResult zScoreNormalization(QVector<Vector> &vectors);
    static void zScoreNormalization(QVector<Vector> &vectors, const ZScoreNormalizationResult &params);
    static void zScoreNormalization(Vector &vector, const ZScoreNormalizationResult &params);

    static MeanNormalizationResult meanNormalization(QVector<Vector> &vectors);
    static void meanNormalization(QVector<Vector> &vectors, const MeanNormalizationResult &params);
    static void meanNormalization(Vector &vector, const MeanNormalizationResult &params);
};

#endif // NORMALIZATION_H
