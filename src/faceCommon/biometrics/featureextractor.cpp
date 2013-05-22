#include "featureextractor.h"

void ZScorePCAExtractor::serialize(const QString &pcaPath, const QString &normParamsPath)
{
    pca.serialize(pcaPath);
    normParams.serialize(normParamsPath);
}

ZScorePCAExtractor::ZScorePCAExtractor(const QString &pcaPath, const QString &normParamsPath)
{
    pca = PCA(pcaPath);
    normParams = ZScoreNormalizationResult(normParamsPath);
}
