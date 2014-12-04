#include "faceCommon/biometrics/featureextractor.h"

using namespace Face;
using namespace Face::Biometrics;

FeatureExtractor::Ptr FeatureExtractorFactory::create(const std::string &name)
{
    if (name == PCAExtractor::name())
        return new PCAExtractor();
    else if (name == ZScorePCAExtractor::name())
        return new ZScorePCAExtractor();
    else if (name == PassExtractor::name())
        return new PassExtractor();
    else if (name == ZScorePassExtractor::name())
        return new ZScorePassExtractor();
    /*else if (name == NormPCAExtractor::name())
        return new NormPCAExtractor();
    else if (name == ICAofPCAExtractor::name())
        return new ICAofPCAExtractor();
    else if (name == ZScoreICAofPCAExtractor::name())
        return new ZScoreICAofPCAExtractor();
    else if (name == ICAofPCAWhiteningExtractor::name())
        return new ICAofPCAWhiteningExtractor();*/

    throw FACELIB_EXCEPTION("unknown extractor " + name);
}

void PCAExtractor::serialize(cv::FileStorage &storage) const
{
    pca.serialize(storage);
}

void PCAExtractor::deserialize(cv::FileStorage &storage)
{
    pca.deserialize(storage);
}

void PCAExtractor::train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data)
{
    train(ids, data, 0.995);
}

void PCAExtractor::train(const std::vector<int> &/*ids*/, const std::vector<LinAlg::Vector> &data, double selThreshold)
{
    pca = Face::LinAlg::PCA(data);
    pca.modesSelectionThreshold(selThreshold);
}

void ICAofPCAExtractor::train(const std::vector<int> &/*ids*/, const std::vector<LinAlg::Vector> &data)
{
    icaOfpca.learn(data, 0.995);
}

void LDAofPCAExtractor::train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data)
{
    ldaOfpca.learn(data, ids, 0.995);
}

void ZScorePCAExtractor::serialize(cv::FileStorage &storage) const
{
    pca.serialize(storage);
    normParams.serialize(storage);
}

void ZScorePCAExtractor::deserialize(cv::FileStorage &storage)
{
    pca.deserialize(storage);
    normParams.deserialize(storage);
}

void ZScorePCAExtractor::train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data)
{
    train(ids, data, 0.995);
}

void ZScorePCAExtractor::train(const std::vector<int> &/*ids*/, const std::vector<LinAlg::Vector> &data, double selThreshold)
{
    pca = Face::LinAlg::PCA(data);
    pca.modesSelectionThreshold(selThreshold);

    std::vector<Face::LinAlg::Vector> projectedData = pca.batchProject(data);
    normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
}

void ZScorePassExtractor::train(const std::vector<int> &/*ids*/, const std::vector<LinAlg::Vector> &rawData)
{
    std::vector<Face::LinAlg::Vector> projectedData;
    for (unsigned int i = 0; i < rawData.size(); i++)
    {
         projectedData.push_back(Face::LinAlg::Vector(rawData[i]));
    }
    normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
}

void ZScorePassExtractor::serialize(cv::FileStorage &storage) const
{
    normParams.serialize(storage);
}

void ZScorePassExtractor::deserialize(cv::FileStorage &storage)
{
    normParams.deserialize(storage);
}

ZScorePassExtractor::ZScorePassExtractor(const std::string &normParamsPath)
{
    normParams = Face::LinAlg::ZScoreNormalizationResult(normParamsPath);
}

Face::LinAlg::Vector ZScorePCAExtractor::extract(const Face::LinAlg::Vector &rawData) const
{
    Face::LinAlg::Vector projected = pca.project(rawData);
    Face::LinAlg::Normalization::zScoreNormalization(projected, normParams);
    return projected;
}
