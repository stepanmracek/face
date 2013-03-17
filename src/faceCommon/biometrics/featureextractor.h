#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include "linalg/common.h"
#include "linalg/pca.h"
#include "linalg/icaofpca.h"
#include "linalg/ldaofpca.h"
#include "linalg/normalization.h"
#include "linalg/vector.h"

class FeatureExtractor
{
public:
    virtual Vector extract(Vector &rawData) = 0;
    virtual int outputLen() = 0;

    QVector<Vector> batchExtract(QVector<Vector> &rawData)
    {
        QVector<Vector> result;
        for (int i = 0; i < rawData.count(); i++)
        {
            Vector extracted = extract(rawData[i]);
            result.append(extracted);
        }
        return result;
    }

    virtual ~FeatureExtractor() {}
};

class ZScoreFeatureExtractor : public FeatureExtractor
{

};

class PCAExtractor : public FeatureExtractor
{
private:
    PCA pca;

public:
    PCAExtractor(PCA pca) : pca(pca) {}

    Vector extract(Vector &rawData)
    {
        return pca.project(rawData);
    }

    int outputLen() { return pca.getModes(); }

    virtual ~PCAExtractor() {}
};

class ZScorePCAExtractor : public ZScoreFeatureExtractor
{
private:
    PCA pca;
    ZScoreNormalizationResult normParams;

public:
    ZScorePCAExtractor(PCA pca, QVector<Vector> &rawData )
    {
        this->pca = pca;
        QVector<Vector> projectedData = pca.batchProject(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Vector extract(Vector &rawData)
    {
        Vector projected = pca.project(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return pca.getModes(); }

    virtual ~ZScorePCAExtractor() {}
};

class NormPCAExtractor : public FeatureExtractor
{
private:
    PCA pca;

public:
    NormPCAExtractor(PCA pca) : pca(pca) {}

    Vector extract(Vector &rawData)
    {
        return pca.scaledProject(rawData);
    }

    int outputLen() { return pca.getModes(); }

    virtual ~NormPCAExtractor() {}
};

class ICAofPCAExtractor : public FeatureExtractor
{
private:
    ICAofPCA icaOfpca;

public:
    ICAofPCAExtractor(ICAofPCA icaOfpca) : icaOfpca(icaOfpca) {}

    Vector extract(Vector &rawData)
    {
        return icaOfpca.project(rawData);
    }

    int outputLen() { return icaOfpca.ica.getModes(); }

    virtual ~ICAofPCAExtractor() {}
};

class ZScoreICAofPCAExtractor : public ZScoreFeatureExtractor
{
private:
    ICAofPCA icaOfpca;
    ZScoreNormalizationResult normParams;

public:
    ZScoreICAofPCAExtractor(ICAofPCA icaOfpca, QVector<Vector> &rawData )
    {
        this->icaOfpca = icaOfpca;
        QVector<Vector> projectedData = icaOfpca.batchProject(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Vector extract(Vector &rawData)
    {
        Vector projected = icaOfpca.project(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return icaOfpca.ica.getModes(); }

    virtual ~ZScoreICAofPCAExtractor() {}
};

class ICAofPCAWhiteningExtractor : public FeatureExtractor
{
private:
    ICAofPCA icaOfpca;

public:
    ICAofPCAWhiteningExtractor(ICAofPCA icaOfpca) : icaOfpca(icaOfpca) {}

    Vector extract(Vector &rawData)
    {
        Vector pcaProjected = icaOfpca.pca.project(rawData);
        return icaOfpca.ica.whiten(pcaProjected);
    }

    int outputLen() { return icaOfpca.ica.getModes(); }

    virtual ~ICAofPCAWhiteningExtractor() {}
};

class ZScoreICAofPCAWhiteningExtractor : public ZScoreFeatureExtractor
{
private:
    ICAofPCA icaOfpca;
    ZScoreNormalizationResult normParams;

public:
    ZScoreICAofPCAWhiteningExtractor(ICAofPCA icaOfpca, QVector<Vector> &rawData )
    {
        this->icaOfpca = icaOfpca;
        QVector<Vector> projectedData = icaOfpca.whiten(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Vector extract(Vector &rawData)
    {
        Vector projected = icaOfpca.whiten(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return icaOfpca.ica.getModes(); }

    virtual ~ZScoreICAofPCAWhiteningExtractor() {}
};

class LDAofPCAExtractor : public FeatureExtractor
{
private:
    LDAofPCA ldaOfpca;

public:
    LDAofPCAExtractor(LDAofPCA ldaOfpca) : ldaOfpca(ldaOfpca) {}

    Vector extract(Vector &rawData)
    {
        return ldaOfpca.project(rawData);
    }

    int outputLen() { return ldaOfpca.lda.Wt.rows; }

    virtual ~LDAofPCAExtractor() {}
};

class ZScoreLDAofPCAExtractor : public ZScoreFeatureExtractor
{
private:
    LDAofPCA ldaOfpca;
    ZScoreNormalizationResult normParams;

public:
    ZScoreLDAofPCAExtractor(LDAofPCA ldaOfpca, QVector<Vector> &rawData )
    {
        this->ldaOfpca = ldaOfpca;
        QVector<Vector> projectedData = ldaOfpca.batchProject(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Vector extract(Vector &rawData)
    {
        Vector projected = ldaOfpca.project(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return ldaOfpca.lda.Wt.rows; }

    virtual ~ZScoreLDAofPCAExtractor() {}
};

class LDAExtractor : public FeatureExtractor
{
private:
    LDA lda;

public:
    LDAExtractor(LDA lda) : lda(lda) {}

    Vector extract(Vector &rawData)
    {
        return lda.project(rawData);
    }

    int outputLen() { return lda.Wt.rows; }

    virtual ~LDAExtractor() {}
};

class ZScoreLDAExtractor : public ZScoreFeatureExtractor
{
private:
    LDA lda;
    ZScoreNormalizationResult normParams;

public:
    ZScoreLDAExtractor(LDA lda, QVector<Vector> &rawData )
    {
        this->lda = lda;
        QVector<Vector> projectedData = lda.batchProject(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Vector extract(Vector &rawData)
    {
        Vector projected = lda.project(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return lda.Wt.rows; }

    virtual ~ZScoreLDAExtractor() {}
};

class PassExtractor : public FeatureExtractor
{
public:
    Vector extract(Vector &rawData)
    {
        return Vector(rawData);
    }

    int outputLen() { return -1; }
};

class ZScorePassExtractor : public ZScoreFeatureExtractor
{
private:
    ZScoreNormalizationResult normParams;

public:
    ZScorePassExtractor(QVector<Vector> &rawData )
    {
        QVector<Vector> projectedData;
        for (int i = 0; i < rawData.count(); i++)
        {
             projectedData.append(Vector(rawData[i]));
        }
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Vector extract(Vector &rawData)
    {
        Vector projected(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return -1; }
};

#endif // FEATUREEXTRACTOR_H
