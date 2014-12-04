#ifndef NORMALIZATION_H
#define NORMALIZATION_H

#include "common.h"
#include "vector.h"
#include "iserializable.h"

namespace Face {
namespace LinAlg {

class LinearNormalizationResult
{
public:
    std::vector<double> minValues;
    std::vector<double> maxValues;
};

class MeanNormalizationResult
{
public:
    std::vector<double> means;
};

class ZScoreNormalizationResult : public ISerializable
{
public:
    std::vector<double> means;
    std::vector<double> stdDevs;

    ZScoreNormalizationResult() {}
    ZScoreNormalizationResult(const std::string &path);

    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};

class Normalization
{
public:
    static LinearNormalizationResult linearNormalization(std::vector<Vector> &vectors);
    static void linearNormalization(std::vector<Vector> &vectors, const LinearNormalizationResult &params);

    static ZScoreNormalizationResult zScoreNormalization(std::vector<Vector> &vectors);
    static void zScoreNormalization(std::vector<Vector> &vectors, const ZScoreNormalizationResult &params);
    static void zScoreNormalization(Vector &vector, const ZScoreNormalizationResult &params);

    static MeanNormalizationResult meanNormalization(std::vector<Vector> &vectors);
    static void meanNormalization(std::vector<Vector> &vectors, const MeanNormalizationResult &params);
    static void meanNormalization(Vector &vector, const MeanNormalizationResult &params);
};

}
}

#endif // NORMALIZATION_H
