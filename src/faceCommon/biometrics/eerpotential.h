#ifndef EERPOTENTIAL_H
#define EERPOTENTIAL_H

#include <QVector>
#include <QDebug>

#include "template.h"
#include "linalg/metrics.h"
#include "linalg/common.h"
#include "featurepotentialbase.h"

class EERPotential : public FeaturePotentialBase
{
public:
    EERPotential(QVector<Template> &templates);
};

#endif // EERPOTENTIAL_H
