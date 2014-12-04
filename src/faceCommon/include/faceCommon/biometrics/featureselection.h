#ifndef FEATURESELECTION_H
#define FEATURESELECTION_H

#include "faceCommon/linalg/metrics.h"

namespace Face {
namespace Biometrics {

class Template;

class FeatureSelection
{
public:
    FeatureSelection(std::vector<Template> &trainTemplates, Face::LinAlg::WeightedMetric &metrics,
                     std::vector<std::vector<Template> > *validationTemplates = 0);

    Matrix resultingWeight;

    Matrix bestTrainWeight;
    double bestTrainEER;
    int bestTrainStep;
    std::vector<int> selectedIndicies;

    //Matrix bestValidationWeight;
    //double bestValidationEER;
    //int bestValidationStep;

    //std::vector<std::vector<double> > validationEER;
};

}
}

#endif // FEATURESELECTION_H
