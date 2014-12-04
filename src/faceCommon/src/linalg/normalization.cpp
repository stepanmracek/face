#include "faceCommon/linalg/normalization.h"

#include "faceCommon/linalg/vector.h"

using namespace Face::LinAlg;

LinearNormalizationResult Normalization::linearNormalization(std::vector<Vector> &vectors)
{
    int n = vectors.size();
    int m = vectors[0].rows;

    LinearNormalizationResult res;
    res.maxValues = std::vector<double>(m, -1e300);
    res.minValues = std::vector<double>(m, 1e300);

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

void Normalization::linearNormalization(std::vector<Vector> &vectors, const LinearNormalizationResult &params)
{
	unsigned int n = vectors.size();
    unsigned int m = vectors[0].rows;
    if (params.minValues.size() != m || params.maxValues.size() != m)
        throw FACELIB_EXCEPTION("invalid params");

    for (unsigned int i = 0; i < n; i++)
    {
        vectors[i] = vectors[i].normalizeComponents(params.minValues, params.maxValues);
    }
}


ZScoreNormalizationResult Normalization::zScoreNormalization(std::vector<Vector> &vectors)
{
    int n = vectors.size();
    int m = vectors[0].rows;

    // calculate mean and stdDev
    ZScoreNormalizationResult result;
    for (int componentIndex = 0; componentIndex < m; componentIndex++)
    {
        std::vector<double> values;
        for (int vectorIndex = 0; vectorIndex < n; vectorIndex++)
        {
            values.push_back(vectors[vectorIndex](componentIndex));
        }

        Vector valuesVec(values);
        result.means.push_back(valuesVec.meanValue());
        result.stdDevs.push_back(valuesVec.stdDeviation());
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
        if (params.stdDevs[componentIndex] == 0 || params.stdDevs[componentIndex] != params.stdDevs[componentIndex])
        	norm = 0.0;
        else
            norm = (val - params.means[componentIndex])/params.stdDevs[componentIndex];
        vector(componentIndex) = norm;
    }
}

void Normalization::zScoreNormalization(std::vector<Vector> &vectors, const ZScoreNormalizationResult &params)
{
    for (unsigned int vectorIndex = 0; vectorIndex < vectors.size(); vectorIndex++)
    {
        zScoreNormalization(vectors[vectorIndex], params);
    }
}

void ZScoreNormalizationResult::serialize(cv::FileStorage &storage) const
{
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("ZScoreNormalizationResult::serialize - cv::FileStorage is not opened");
    }

    Vector meansVec(means);
    Vector stdDevsVec(stdDevs);

    storage << "means" << meansVec;
    storage << "stdDev" << stdDevsVec;
}

void ZScoreNormalizationResult::deserialize(cv::FileStorage &storage)
{
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("ZScoreNormalizationResult::deserialize - cv::FileStorage is not opened");
    }

    Vector meansVec;
    storage["means"] >> meansVec;
    means = meansVec.toStdVector();

    Vector stdDevsVec;
    storage["stdDev"] >> stdDevsVec;
    stdDevs = stdDevsVec.toStdVector();
}

ZScoreNormalizationResult::ZScoreNormalizationResult(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    deserialize(storage);
}

MeanNormalizationResult Normalization::meanNormalization(std::vector<Vector> &vectors)
{
    int n = vectors.size();
    int m = vectors[0].rows;

    // calculate mean and stdDev
    MeanNormalizationResult result;
    for (int componentIndex = 0; componentIndex < m; componentIndex++)
    {
        std::vector<double> values;
        for (int vectorIndex = 0; vectorIndex < n; vectorIndex++)
        {
            values.push_back(vectors[vectorIndex](componentIndex));
        }

        Vector valuesVec(values);
        result.means.push_back(valuesVec.meanValue());
    }

    // normalize
    meanNormalization(vectors, result);
    return result;
}

void Normalization::meanNormalization(Vector &vector, const MeanNormalizationResult &params)
{
    for (int componentIndex = 0; componentIndex < vector.rows; componentIndex++)
    {
        vector(componentIndex) = vector(componentIndex) - params.means[componentIndex];
    }
}

void Normalization::meanNormalization(std::vector<Vector> &vectors, const MeanNormalizationResult &params)
{
    for (unsigned int vectorIndex = 0; vectorIndex < vectors.size(); vectorIndex++)
    {
        meanNormalization(vectors[vectorIndex], params);
    }
}
