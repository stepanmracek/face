#include "normalization.h"

#include <cassert>

#include "linalg/vector.h"

LinearNormalizationResult Normalization::linearNormalization(QVector<Vector> &vectors)
{
    int n = vectors.count();
    assert(n > 0);
    int m = vectors[0].rows;
    assert(m > 0);

    LinearNormalizationResult res;
    res.maxValues = QVector<double>(m, -1e300);
    res.minValues = QVector<double>(m, 1e300);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            double val = vectors[i](j);
            if (val > res.maxValues[j])
                res.maxValues[j] = val;
            if (val < res.minValues[j])
                res.minValues[j] = val;
        }
    }

    linearNormalization(vectors, res);
    return res;
}

void Normalization::linearNormalization(QVector<Vector> &vectors, const LinearNormalizationResult &params)
{
    int n = vectors.count();
    assert(n > 0);
    int m = vectors[0].rows;
    assert(m > 0);
    assert(params.minValues.count() == m);
    assert(params.maxValues.count() == m);

    for (int i = 0; i < n; i++)
    {
        vectors[i] = vectors[i].normalizeComponents(params.minValues, params.maxValues);
    }
}


ZScoreNormalizationResult Normalization::zScoreNormalization(QVector<Vector> &vectors)
{
    int n = vectors.count();
    assert(n > 0);
    int m = vectors[0].rows;

    // calculate mean and stdDev
    ZScoreNormalizationResult result;
    for (int componentIndex = 0; componentIndex < m; componentIndex++)
    {
        QVector<double> values;
        for (int vectorIndex = 0; vectorIndex < n; vectorIndex++)
        {
            values.append(vectors[vectorIndex](componentIndex));
        }

        Vector valuesVec(values);
        result.mean.append(valuesVec.meanValue());
        result.stdDev.append(valuesVec.stdDeviation());
    }

    // normalize
    zScoreNormalization(vectors, result);
    return result;
}

void Normalization::zScoreNormalization(Vector &vector, const ZScoreNormalizationResult &params)
{
    for (int componentIndex = 0; componentIndex < vector.rows; componentIndex++)
    {
        double val = vector(componentIndex);
        double norm;
        if (params.stdDev[componentIndex] == 0 || params.stdDev[componentIndex] != params.stdDev[componentIndex])
        	norm = 0.0;
        else
        	norm = (val - params.mean[componentIndex])/params.stdDev[componentIndex];
        vector(componentIndex) = norm;
    }
}

void Normalization::zScoreNormalization(QVector<Vector> &vectors, const ZScoreNormalizationResult &params)
{
    for (int vectorIndex = 0; vectorIndex < vectors.count(); vectorIndex++)
    {
        zScoreNormalization(vectors[vectorIndex], params);
    }
}

void ZScoreNormalizationResult::serialize(const QString &path)
{
    Vector means(mean);
    Vector stdDevs(stdDev);

    cv::FileStorage storage(path.toStdString(), cv::FileStorage::WRITE);
    assert(storage.isOpened());
    storage << "mean" << means;
    storage << "stdDev" << stdDevs;
    storage.release();
}

ZScoreNormalizationResult::ZScoreNormalizationResult(const QString &path)
{
    cv::FileStorage storage(path.toStdString(), cv::FileStorage::READ);
    assert(storage.isOpened());

    Vector means;
    storage["mean"] >> means;
    Vector stdDevs;
    storage["stdDev"] >> stdDevs;
    mean = means.toQVector();
    stdDev = stdDevs.toQVector();
    storage.release();
}

MeanNormalizationResult Normalization::meanNormalization(QVector<Vector> &vectors)
{
    int n = vectors.count();
    assert(n > 0);
    int m = vectors[0].rows;

    // calculate mean and stdDev
    MeanNormalizationResult result;
    for (int componentIndex = 0; componentIndex < m; componentIndex++)
    {
        QVector<double> values;
        for (int vectorIndex = 0; vectorIndex < n; vectorIndex++)
        {
            values.append(vectors[vectorIndex](componentIndex));
        }

        Vector valuesVec(values);
        result.mean.append(valuesVec.meanValue());
    }

    // normalize
    meanNormalization(vectors, result);
    return result;
}

void Normalization::meanNormalization(Vector &vector, const MeanNormalizationResult &params)
{
    for (int componentIndex = 0; componentIndex < vector.rows; componentIndex++)
    {
        vector(componentIndex) = vector(componentIndex) - params.mean[componentIndex];
    }
}

void Normalization::meanNormalization(QVector<Vector> &vectors, const MeanNormalizationResult &params)
{
    for (int vectorIndex = 0; vectorIndex < vectors.count(); vectorIndex++)
    {
        meanNormalization(vectors[vectorIndex], params);
    }
}
