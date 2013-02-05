#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include "linalg/common.h"
#include "linalg/pca.h"
#include "linalg/icaofpca.h"
#include "linalg/ldaofpca.h"
#include "linalg/normalization.h"

class FeatureExtractor
{
public:
    virtual Matrix extract(Matrix &rawData) = 0;
    virtual int outputLen() = 0;

    QVector<Matrix> batchExtract(QVector<Matrix> &rawData)
    {
        QVector<Matrix> result;
        for (int i = 0; i < rawData.count(); i++)
        {
            Matrix extracted = extract(rawData[i]);
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

    Matrix extract(Matrix &rawData)
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
    ZScorePCAExtractor(PCA pca, QVector<Matrix> &rawData )
    {
        this->pca = pca;
        QVector<Matrix> projectedData = pca.project(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Matrix extract(Matrix &rawData)
    {
        Matrix projected = pca.project(rawData);
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

    Matrix extract(Matrix &rawData)
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

    Matrix extract(Matrix &rawData)
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
    ZScoreICAofPCAExtractor(ICAofPCA icaOfpca, QVector<Matrix> &rawData )
    {
        this->icaOfpca = icaOfpca;
        QVector<Matrix> projectedData = icaOfpca.project(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Matrix extract(Matrix &rawData)
    {
        Matrix projected = icaOfpca.project(rawData);
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

    Matrix extract(Matrix &rawData)
    {
        Matrix pcaProjected = icaOfpca.pca.project(rawData);
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
    ZScoreICAofPCAWhiteningExtractor(ICAofPCA icaOfpca, QVector<Matrix> &rawData )
    {
        this->icaOfpca = icaOfpca;
        QVector<Matrix> projectedData = icaOfpca.whiten(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Matrix extract(Matrix &rawData)
    {
        Matrix projected = icaOfpca.whiten(rawData);
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

    Matrix extract(Matrix &rawData)
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
    ZScoreLDAofPCAExtractor(LDAofPCA ldaOfpca, QVector<Matrix> &rawData )
    {
        this->ldaOfpca = ldaOfpca;
        QVector<Matrix> projectedData = ldaOfpca.project(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Matrix extract(Matrix &rawData)
    {
        Matrix projected = ldaOfpca.project(rawData);
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

    Matrix extract(Matrix &rawData)
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
    ZScoreLDAExtractor(LDA lda, QVector<Matrix> &rawData )
    {
        this->lda = lda;
        QVector<Matrix> projectedData = lda.project(rawData);
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Matrix extract(Matrix &rawData)
    {
        Matrix projected = lda.project(rawData);
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return lda.Wt.rows; }

    virtual ~ZScoreLDAExtractor() {}
};

class PassExtractor : public FeatureExtractor
{
public:
    Matrix extract(Matrix &rawData) { return rawData.clone(); }

    int outputLen() { return -1; }
};

class ZScorePassExtractor : public ZScoreFeatureExtractor
{
private:
    ZScoreNormalizationResult normParams;

public:
    ZScorePassExtractor(QVector<Matrix> &rawData )
    {
        QVector<Matrix> projectedData;
        for (int i = 0; i < rawData.count(); i++)
        {
             Matrix projected = rawData[i].clone();
             projectedData.append(projected);
        }
        normParams = Normalization::zScoreNormalization(projectedData);
    }

    Matrix extract(Matrix &rawData)
    {
        Matrix projected = rawData.clone();
        Normalization::zScoreNormalization(projected, normParams);
        return projected;
    }

    int outputLen() { return -1; }
};

#endif // FEATUREEXTRACTOR_H
