#include "faceCommon/biometrics/zpcacorrw.h"

#include "faceCommon/biometrics/evaluation.h"
#include "faceCommon/biometrics/template.h"
#include "faceCommon/biometrics/eerpotential.h"

using namespace Face::Biometrics;

void ZPCACorrW::init(const std::vector<Face::LinAlg::Vector> &trainPCAVectors, double pcaSelThreshold, const std::vector<Face::LinAlg::Vector> &trainZscoreVectors)
{
    LinAlg::PCA pca(trainPCAVectors);
    pca.modesSelectionThreshold(pcaSelThreshold);
    extractor = ZScorePCAExtractor(pca, trainZscoreVectors);
    metric.w = Face::LinAlg::Vector(pca.getModes(), 1.0);
}

ZPCACorrW::ZPCACorrW(const std::vector<Face::LinAlg::Vector> &trainPCAVectors, double pcaSelThreshold,
                     const std::vector<Face::LinAlg::Vector> &trainZscoreVectors)
{
    init(trainPCAVectors, pcaSelThreshold, trainZscoreVectors);
}

ZPCACorrW::ZPCACorrW(const std::vector<Face::LinAlg::Vector> &trainPCAVectors, double pcaSelThreshold,
                     const std::vector<Face::LinAlg::Vector> &trainZscoreVectors, const std::vector<int> &trainFeatureSelectionClasses,
                     const std::vector<Face::LinAlg::Vector> &trainFeatureSelectionVectors,
                     double featureSelThresholdStart, double featureSelThresholdEnd, double featureSelThresholdStep)
{
    init(trainPCAVectors, pcaSelThreshold, trainZscoreVectors);

    if (featureSelThresholdEnd > featureSelThresholdStart)
    {
        std::cout << "eer potential" << std::endl;
        std::vector<Face::Biometrics::Template> selectionTemplates = Template::createTemplates(trainFeatureSelectionVectors,
                                                                                           trainFeatureSelectionClasses,
                                                                                           extractor);
        EERPotential potential(selectionTemplates);

        std::cout << "feature selection" << std::endl;
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
            std::cout << "Feature selection threshold: " << bestThreshold << std::endl;
            metric.w = potential.createSelectionWeightsBasedOnRelativeThreshold(bestThreshold);
        }
        else
        {
            metric.w = Face::LinAlg::Vector(extractor.pca.getModes(), 1.0);
        }
    }
}
