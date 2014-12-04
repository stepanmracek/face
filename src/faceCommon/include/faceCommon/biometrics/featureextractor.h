#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/pca.h"
#include "faceCommon/linalg/icaofpca.h"
#include "faceCommon/linalg/ldaofpca.h"
#include "faceCommon/linalg/normalization.h"
#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/iserializable.h"

namespace Face {
namespace Biometrics {

class FeatureExtractor : public Face::LinAlg::ISerializable
{
public:
    typedef cv::Ptr<FeatureExtractor> Ptr;

    virtual ~FeatureExtractor() {}

    virtual void train(const std::vector<int> &ids, const std::vector<Face::LinAlg::Vector> & data) = 0;
    virtual Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const = 0;
    virtual int outputLen() const = 0;
    virtual std::string writeParams() const = 0;

    std::vector<Face::LinAlg::Vector> batchExtract(const std::vector<Face::LinAlg::Vector> &rawData)
    {
        std::vector<Face::LinAlg::Vector> result;
        for (auto i = rawData.begin(); i != rawData.end(); ++i)
        {
            Face::LinAlg::Vector extracted = extract(*i);
            result.push_back(extracted);
        }
        return std::move(result);
    }
};

class FeatureExtractorFactory
{
public:
    static FeatureExtractor::Ptr create(const std::string &name);
};

class ZScoreFeatureExtractor : public FeatureExtractor
{

};

class PCAExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::PCA pca;

public:
    PCAExtractor() {}
    PCAExtractor(const Face::LinAlg::PCA &pca) : pca(pca) {}

    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);

    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data);
    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data, double selThreshold);

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        return pca.project(rawData);
    }

    int outputLen() const { return pca.getModes(); }

    static std::string name() { return "pca"; }
    std::string writeParams() const { return name(); }
};

class ZScorePCAExtractor : public ZScoreFeatureExtractor
{
public:
    Face::LinAlg::PCA pca;
    Face::LinAlg::ZScoreNormalizationResult normParams;

    ZScorePCAExtractor() {}

    ZScorePCAExtractor(const Face::LinAlg::PCA &pca, const std::vector<Face::LinAlg::Vector> &rawData ) : pca(pca)
    {
        std::vector<Face::LinAlg::Vector> projectedData = pca.batchProject(rawData);
        normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
    }

    ZScorePCAExtractor(const std::string &path);

    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);

    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data);
    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data, double selThreshold);

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const;

    int outputLen() const { return pca.getModes(); }

    static std::string name() { return "zpca"; }
    std::string writeParams() const { return name(); }
};

class NormPCAExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::PCA pca;

public:
    NormPCAExtractor(const Face::LinAlg::PCA &pca) : pca(pca) {}

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        return pca.scaledProject(rawData);
    }

    int outputLen() const { return pca.getModes(); }

    static std::string name() { return "npca"; }
    std::string writeParams() const { return name(); }
};

class ICAofPCAExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::ICAofPCA icaOfpca;

public:
    ICAofPCAExtractor() {}
    ICAofPCAExtractor(const Face::LinAlg::ICAofPCA &icaOfpca) : icaOfpca(icaOfpca) {}

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        return icaOfpca.project(rawData);
    }

    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data);

    int outputLen() const { return icaOfpca.ica.getModes(); }

    static std::string name() { return "icaofpca"; }
    std::string writeParams() const { return name(); }

    void serialize(cv::FileStorage &/*storage*/) const { throw FACELIB_EXCEPTION("not implemented"); }
    void deserialize(cv::FileStorage &/*storage*/) { throw FACELIB_EXCEPTION("not implemented"); }
};

class ZScoreICAofPCAExtractor : public ZScoreFeatureExtractor
{
private:
    Face::LinAlg::ICAofPCA icaOfpca;
    Face::LinAlg::ZScoreNormalizationResult normParams;

public:
    ZScoreICAofPCAExtractor(const Face::LinAlg::ICAofPCA &icaOfpca, const std::vector<Face::LinAlg::Vector> &rawData )
    {
        this->icaOfpca = icaOfpca;
        std::vector<Face::LinAlg::Vector> projectedData = icaOfpca.batchProject(rawData);
        normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
    }

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector projected = icaOfpca.project(rawData);
        Face::LinAlg::Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() const { return icaOfpca.ica.getModes(); }

    static std::string name() { return "zicaofpca"; }
    std::string writeParams() const { return name(); }
};

class ICAofPCAWhiteningExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::ICAofPCA icaOfpca;

public:
    ICAofPCAWhiteningExtractor(const Face::LinAlg::ICAofPCA &icaOfpca) : icaOfpca(icaOfpca) {}

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector pcaProjected = icaOfpca.pca.project(rawData);
        return icaOfpca.ica.whiten(pcaProjected);
    }

    int outputLen() const { return icaOfpca.ica.getModes(); }

    static std::string name() { return "wicaofpca"; }
    std::string writeParams() const { return name(); }
};

class ZScoreICAofPCAWhiteningExtractor : public ZScoreFeatureExtractor
{
private:
    Face::LinAlg::ICAofPCA icaOfpca;
    Face::LinAlg::ZScoreNormalizationResult normParams;

public:
    ZScoreICAofPCAWhiteningExtractor(const Face::LinAlg::ICAofPCA &icaOfpca, const std::vector<Face::LinAlg::Vector> &rawData )
    {
        this->icaOfpca = icaOfpca;
        std::vector<Face::LinAlg::Vector> projectedData = icaOfpca.whiten(rawData);
        normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
    }

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector projected = icaOfpca.whiten(rawData);
        Face::LinAlg::Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() const { return icaOfpca.ica.getModes(); }

    static std::string name() { return "zwpca"; }
    std::string writeParams() const { return name(); }
};

class LDAofPCAExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::LDAofPCA ldaOfpca;

public:
    LDAofPCAExtractor() {}
    LDAofPCAExtractor(const Face::LinAlg::LDAofPCA &ldaOfpca) : ldaOfpca(ldaOfpca) {}

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        return ldaOfpca.project(rawData);
    }

    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data);

    int outputLen() const { return ldaOfpca.lda.Wt.rows; }

    static std::string name() { return "lda"; }
    std::string writeParams() const { return name(); }

    void serialize(cv::FileStorage &/*storage*/) const { throw FACELIB_EXCEPTION("not implemented"); }
    void deserialize(cv::FileStorage &/*storage*/) { throw FACELIB_EXCEPTION("not implemented"); }
};

class ZScoreLDAofPCAExtractor : public ZScoreFeatureExtractor
{
private:
    Face::LinAlg::LDAofPCA ldaOfpca;
    Face::LinAlg::ZScoreNormalizationResult normParams;

public:
    ZScoreLDAofPCAExtractor(const Face::LinAlg::LDAofPCA &ldaOfpca, const std::vector<Face::LinAlg::Vector> &rawData )
    {
        this->ldaOfpca = ldaOfpca;
        std::vector<Face::LinAlg::Vector> projectedData = ldaOfpca.batchProject(rawData);
        normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
    }

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector projected = ldaOfpca.project(rawData);
        Face::LinAlg::Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() const { return ldaOfpca.lda.Wt.rows; }

    static std::string name() { return "zlda"; }
    std::string writeParams() const { return name(); }
};

class LDAExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::LDA lda;

public:
    LDAExtractor(const Face::LinAlg::LDA &lda) : lda(lda) {}

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        return lda.project(rawData);
    }

    int outputLen() const { return lda.Wt.rows; }

    virtual ~LDAExtractor() {}
};

class ZScoreLDAExtractor : public ZScoreFeatureExtractor
{
private:
    Face::LinAlg::LDA lda;
    Face::LinAlg::ZScoreNormalizationResult normParams;

public:
    ZScoreLDAExtractor(const Face::LinAlg::LDA &lda, const std::vector<Face::LinAlg::Vector> &rawData )
    {
        this->lda = lda;
        std::vector<Face::LinAlg::Vector> projectedData = lda.batchProject(rawData);
        normParams = Face::LinAlg::Normalization::zScoreNormalization(projectedData);
    }

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector projected = lda.project(rawData);
        Face::LinAlg::Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() const { return lda.Wt.rows; }

    virtual ~ZScoreLDAExtractor() {}
};

class PassExtractor : public FeatureExtractor
{
public:
    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        return Face::LinAlg::Vector(rawData);
    }

    int outputLen() const { return -1; }

    static std::string name() { return "pass"; }
    std::string writeParams() const { return name(); }

    void serialize(cv::FileStorage &) const {}
    void deserialize(cv::FileStorage &) {}

    void train(const std::vector<int> &, const std::vector<LinAlg::Vector> &) {}
};

class ZScorePassExtractor : public ZScoreFeatureExtractor
{
private:
    Face::LinAlg::ZScoreNormalizationResult normParams;

public:
    ZScorePassExtractor() {}

    ZScorePassExtractor(const std::string &normParamsPath);

    ZScorePassExtractor(const std::vector<Face::LinAlg::Vector> &rawData)
    {
        std::vector<int> dummyIds;
        train(dummyIds, rawData);
    }

    void train(const std::vector<int> &ids, const std::vector<LinAlg::Vector> &data);

    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector projected(rawData);
        Face::LinAlg::Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() const { return normParams.means.size(); }

    static std::string name() { return "zpass"; }
    std::string writeParams() const { return name(); }
};

class MeanNormalizationExtractor : public FeatureExtractor
{
private:
    Face::LinAlg::MeanNormalizationResult params;

public:
    MeanNormalizationExtractor(const std::vector<Face::LinAlg::Vector> &rawData)
    {
        std::vector<Face::LinAlg::Vector> projectedData;
        for (auto i = rawData.begin(); i != rawData.end(); ++i)
        {
             projectedData.push_back(Face::LinAlg::Vector(*i));
        }
        params = Face::LinAlg::Normalization::meanNormalization(projectedData);
    }

    Face::LinAlg::Vector extract(const Face::LinAlg::Vector &rawData) const
    {
        Face::LinAlg::Vector projected(rawData);
        Face::LinAlg::Normalization::meanNormalization(projected, params);
        return projected;
    }

    int outputLen() const { return params.means.size(); }
};

}
}

#endif // FEATUREEXTRACTOR_H
