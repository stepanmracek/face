#ifndef DISCRIMINATIVEPOTENTIAL_H
#define DISCRIMINATIVEPOTENTIAL_H

#include <vector>

#include "featurepotentialbase.h"

namespace Face {
namespace Biometrics {

class Template;

class DiscriminativePotential : public FeaturePotentialBase
{
public:
    DiscriminativePotential(const std::vector<Template> &templates);
};

}
}

#endif // DISCRIMINATIVEPOTENTIAL_H
