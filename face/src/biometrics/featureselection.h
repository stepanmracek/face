#ifndef FEATURESELECTION_H
#define FEATURESELECTION_H

#include <QVector>

#include "template.h"
#include "linalg/metrics.h"

class FeatureSelection
{
public:
    FeatureSelection(QVector<Template> &trainTemplates, WeightedMetric &metrics,
                     QList<QVector<Template> > *validationTemplates = 0);

    Matrix resultingWeight;

    Matrix bestTrainWeight;
    double bestTrainEER;
    int bestTrainStep;
    QVector<int> selectedIndicies;

    //Matrix bestValidationWeight;
    //double bestValidationEER;
    //int bestValidationStep;

    //QList<QVector<double> > validationEER;
};

#endif // FEATURESELECTION_H
