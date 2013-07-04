#ifndef DISCRIMINATIVEPOTENTIAL_H
#define DISCRIMINATIVEPOTENTIAL_H

#include <QVector>
#include <QList>

#include "template.h"
#include "evaluation.h"
#include "featurepotentialbase.h"

class DiscriminativePotential : public FeaturePotentialBase
{
public:
    DiscriminativePotential(const QVector<Template> &templates);
};

#endif // DISCRIMINATIVEPOTENTIAL_H
