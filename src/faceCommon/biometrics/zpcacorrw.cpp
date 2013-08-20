#include "zpcacorrw.h"

#include "evaluation.h"

void ZPCACorrW::init(const QVector<Vector> &trainPCAVectors, double pcaSelThreshold, const QVector<Vector> &trainZscoreVectors)
{
    PCA pca(trainPCAVectors);
    pca.modesSelectionThreshold(pcaSelThreshold);
    extractor = ZScorePCAExtractor(pca, trainZscoreVectors);
    metric.w = Vector(pca.getModes(), 1.0);
}

ZPCACorrW::ZPCACorrW(const QVector<Vector> &trainPCAVectors, double pcaSelThreshold, const QVector<Vector> &trainZscoreVectors)
{
    init(trainPCAVectors, pcaSelThreshold, trainZscoreVectors);
}

ZPCACorrW::ZPCACorrW(const QVector<Vector> &trainPCAVectors, double pcaSelThreshold, const QVector<Vector> &trainZscoreVectors,
                     const QVector<int> &trainFeatureSelectionClasses, const QVector<Vector> &trainFeatureSelectionVectors,
                     double featureSelThresholdStart, double featureSelThresholdEnd, double featureSelThresholdStep)
{
    init(trainPCAVectors, pcaSelThreshold, trainZscoreVectors);

    if (featureSelThresholdEnd > featureSelThresholdStart)
    {
        qDebug() << "eer potential";
        Templates selectionTemplates = Template::createTemplates(trainFeatureSelectionVectors, trainFeatureSelectionClasses, extractor);
        EERPotential potential(selectionTemplates);

        qDebug() << "feature selection";
        double initialEer = Evaluation(selectionTemplates, metric).eer;
        double bestEer = initialEer;
        double bestThreshold = 0;
        for (double t = featureSelThresholdStart; t <= featureSelThresholdEnd; t += featureSelThresholdStep)
        {
            metric.w = potential.createSelectionWeightsBasedOnRelativeThreshold(t);
            double eer = Evaluation(selectionTemplates, metric).eer;
            if (eer < bestEer)
            {
                bestEer = eer;
                bestThreshold = t;
            }
        }

        if (bestEer < initialEer)
        {
            qDebug() << "Feature selection threshold:" << bestThreshold;
            metric.w = potential.createSelectionWeightsBasedOnRelativeThreshold(bestThreshold);
        }
        else
        {
            metric.w = Vector(extractor.pca.getModes(), 1.0);
        }
    }
}
