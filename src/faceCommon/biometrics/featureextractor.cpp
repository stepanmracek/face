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
