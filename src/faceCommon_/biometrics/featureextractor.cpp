#include "featureextractor.h"

void ZScorePCAExtractor::serialize(const QString &pcaPath, const QString &normParamsPath) const
{
    pca.serialize(pcaPath);
    normParams.serialize(normParamsPath);
}

ZScorePCAExtractor::ZScorePCAExtractor(const QString &pcaPath, const QString &normParamsPath)
{
    pca = PCA(pcaPath);
    normParams = ZScoreNormalizationResult(normParamsPath);
}

void ZScorePassExtractor::serialize(const QString &normParamsPath)
{
    normParams.serialize(normParamsPath);
}

ZScorePassExtractor::ZScorePassExtractor(const QString &normParamsPath)
{
    normParams = ZScoreNormalizationResult(normParamsPath);
}

Vector ZScorePCAExtractor::extract(const Vector &rawData) const
{
    //qDebug() << "input" << rawData.minValue() << rawData.maxValue();
    //double min, max;
    //cv::minMaxIdx(pca.cvPca.mean, &min, &max);
    //qDebug() << "mean" << min << max;

    Vector projected = pca.project(rawData);
    Normalization::zScoreNormalization(projected, normParams);
    return projected;
}
