#ifndef EERPOTENTIAL_H
#define EERPOTENTIAL_H

#include "featurepotentialbase.h"

namespace Face {
namespace Biometrics {

class Template;

class EERPotential : public FeaturePotentialBase
{
public:
    EERPotential(const std::vector<Template> &templates);
};

}
}

#endif // EERPOTENTIAL_H
